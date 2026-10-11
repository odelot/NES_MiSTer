// RetroAchievements RAM Mirror for NES — BRAM write-snoop architecture (v2)
//
// The game RAM regions RA can watch (CPU RAM, Cart WRAM, FDS Work RAM) live in
// SDRAM. Instead of reading them back through SDRAM ch2 during gameplay (which
// contends with CPU/PPU fetches — the core's memory scheduling has no stall
// mechanism), this module keeps a live BRAM mirror fed by a write snoop on the
// SDRAM arbiter. The snoop fires once per granted write on any channel, so CPU
// writes, loader clears, backup-RAM loads and savestate restores are all
// captured with zero impact on game memory timing.
//
// Per frame (pre-vblank trigger, scanline 239 / cycle 340, matching RANes):
//   1. CAPTURE: values for the cached address list are read from the mirror
//      BRAM into a double-buffered value BRAM (~3 cycles/address, no DDR
//      dependency — the snapshot lands before NMI starts at scanline 241).
//   2. FLUSH: the captured buffer is written to the DDR valcache. DDR stalls
//      only delay delivery; they can no longer tear the snapshot or spill
//      SDRAM traffic into active display. If the ARM stalls for over a frame,
//      newer captures overwrite the un-flushed pending bank (freshest wins).
//   3. LIST SYNC: the DDR address list header is checked once per pass; on a
//      request_id change the list is re-fetched and pre-mapped into a local
//      BRAM cache (one-time cost per list change).
//
// The mirror is swept to zero on game load (downloading), covering the FDS
// region too, so leftover bytes from a previous game cannot leak into the
// valcache before the new game initializes its RAM.
//
// NES address mapping (unchanged semantics vs v1):
//   addr < 0x2000  → CPU RAM   (SDRAM 0x380000, mirrored every 0x800)
//   0x6000-0x7FFF  → Cart WRAM (SDRAM 0x3C0000, first 8KB bank)
//   0x6000-0xDFFF  → FDS Work RAM when fds_mode (SDRAM 0x208000 | addr[14:0])
//
// DDRAM protocol (identical layout to v1, version bumped to 0x0301):
//   [0x00000] Header:   magic(32) + 0(8) + flags(8) + version(16)
//   [0x00008] Frame:    frame_counter(32) + reserved(32)
//   [0x00010] Debug1:   {ver(8), dispatch(8), first_dout(16), timeout(16), ok(16)}
//   [0x00018] Debug2:   {first_addr(16), wram(16), cart(16), max_timeout(16)}
//
//   [0x40000] AddrReq:  addr_count(32) + request_id(32)       (ARM → FPGA)
//   [0x40008] Addrs:    addr[0](32) + addr[1](32), ...        (2 per 64-bit word)
//
//   [0x48000] ValResp:  response_id(32) + response_frame(32)  (FPGA → ARM)
//   [0x48008] Values:   val[0..7](8b each), ...               (8 per 64-bit word)
//
//   [0x50000] RTQuery control + request/response mailbox
//   (rtquery is served from the mirror BRAM: instant, any point in the frame)

module ra_ram_mirror_nes #(
	parameter [28:0] DDRAM_BASE = 29'h07A00000  // ARM phys 0x3D000000 >> 3
)(
	input             clk,           // clk_sys (NES ~21.477 MHz)
	input             reset,
	input             vblank,        // pre-vblank pulse (scanline 239, cycle 340)
	input             fds_mode,      // 1 when FDS mapper (mapper 20) is active

	// SDRAM write snoop (clk85 domain, from sdram.sv arbiter)
	input             clk85,
	input             snoop_we,
	input      [24:0] snoop_addr,
	input       [7:0] snoop_din,

	// Mirror clear trigger (game load in progress)
	input             downloading,

	// DDRAM write interface (toggle req/ack)
	output reg [28:0] ddram_wr_addr,
	output reg [63:0] ddram_wr_din,
	output reg  [7:0] ddram_wr_be,
	output reg        ddram_wr_req,
	input             ddram_wr_ack,

	// DDRAM read interface (toggle req/ack)
	output reg [28:0] ddram_rd_addr,
	output reg        ddram_rd_req,
	input             ddram_rd_ack,
	input      [63:0] ddram_rd_dout,

	// Status
	output reg        active,
	output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [12:0] MAX_ADDRS     = 13'd4096;

// Realtime query mailbox
localparam [28:0] QUERY_CTRL_ADDR = DDRAM_BASE + 29'hA000;
localparam [28:0] QUERY_REQ_BASE  = DDRAM_BASE + 29'hA001;
localparam [28:0] QUERY_RESP_BASE = DDRAM_BASE + 29'hA011;
localparam [28:0] ARM_CFG_ADDR    = DDRAM_BASE + 29'd8;   // byte offset 0x40: ARM-written config
localparam [7:0]  MAX_RT_QUERIES  = 8'd16;  // [3:0]=4'd16 in v1 truncated to 0

// Mirror layout: 2KB CPU RAM + 32KB region shared by Cart WRAM (8KB) / FDS WRAM (32KB)
localparam [15:0] MIR_REGION2   = 16'h0800;
localparam [16:0] MIRROR_SIZE   = 17'h08800;              // 34816 bytes

// ======================================================================
// Mirror BRAM: written by SDRAM snoop @clk85, read by FSM @clk
// ======================================================================
reg [7:0] mirror [0:MIRROR_SIZE-1];

// --- clk85 side: snoop filter + clear sweep ---
reg fds_s1, fds_s2;
always @(posedge clk85) begin
	fds_s1 <= fds_mode;
	fds_s2 <= fds_s1;
end

wire snoop_hit_cpu  = (snoop_addr[24:11] == 14'h0700);            // 0x380000, 2KB
wire snoop_hit_cart = ~fds_s2 && (snoop_addr[24:13] == 12'h1E0);  // 0x3C0000, 8KB
wire snoop_hit_fds  =  fds_s2 && (snoop_addr[24:15] == 10'h041);  // 0x208000, 32KB
wire snoop_hit      = snoop_we && (snoop_hit_cpu | snoop_hit_cart | snoop_hit_fds);

wire [15:0] snoop_mir_addr = snoop_hit_cpu ? {5'd0, snoop_addr[10:0]} :
                             snoop_hit_fds ? (MIR_REGION2 + {1'd0, snoop_addr[14:0]}) :
                                             (MIR_REGION2 + {3'd0, snoop_addr[12:0]});

// Clear sweep: zero the whole mirror when a game load starts
reg        sweep_run = 0;
reg [16:0] sweep_addr;
reg        dl_s1, dl_s2, dl_s3;
always @(posedge clk85) begin
	dl_s1 <= downloading;
	dl_s2 <= dl_s1;
	dl_s3 <= dl_s2;
	if (dl_s2 & ~dl_s3) begin
		sweep_run  <= 1;
		sweep_addr <= 0;
	end
	else if (sweep_run && !snoop_hit) begin
		sweep_addr <= sweep_addr + 1'd1;
		if (sweep_addr == MIRROR_SIZE-1) sweep_run <= 0;
	end
end

wire        mir_we    = snoop_hit | sweep_run;
wire [15:0] mir_waddr = snoop_hit ? snoop_mir_addr : sweep_addr[15:0];
wire  [7:0] mir_wdata = snoop_hit ? snoop_din : 8'h00;

always @(posedge clk85) if (mir_we) mirror[mir_waddr] <= mir_wdata;

// --- clk side: FSM read port ---
reg [15:0] mir_raddr;
reg  [7:0] mirror_q;
always @(posedge clk) mirror_q <= mirror[mir_raddr];

// ======================================================================
// Address cache BRAM: pre-mapped list {valid, mirror_addr[15:0]}
// ======================================================================
reg [16:0] addr_cache [0:4095];
reg [11:0] ac_raddr, ac_waddr;
reg [16:0] ac_wdata;
reg        ac_we;
reg [16:0] ac_q;
always @(posedge clk) begin
	if (ac_we) addr_cache[ac_waddr] <= ac_wdata;
	ac_q <= addr_cache[ac_raddr];
end

// ======================================================================
// Value buffer BRAM: double-buffered capture (bank bit + 4096 bytes)
// ======================================================================
reg [7:0] val_buf [0:8191];
reg [12:0] vb_raddr, vb_waddr;
reg  [7:0] vb_wdata;
reg        vb_we;
reg  [7:0] vb_q;
always @(posedge clk) begin
	if (vb_we) val_buf[vb_waddr] <= vb_wdata;
	vb_q <= val_buf[vb_raddr];
end

// ======================================================================
// NES logical address → {valid, mirror addr}
// ======================================================================
function [16:0] nes_to_mirror;
	input [31:0] a;
	begin
		if (a < 32'h2000)
			nes_to_mirror = {1'b1, 5'd0, a[10:0]};
		else if (fds_mode && a >= 32'h6000 && a < 32'hE000)
			nes_to_mirror = {1'b1, MIR_REGION2 + {1'd0, a[14:0]}};
		else if (!fds_mode && a >= 32'h6000 && a < 32'h8000)
			nes_to_mirror = {1'b1, MIR_REGION2 + {3'd0, a[12:0]}};
		else
			nes_to_mirror = 17'd0;  // invalid → value 0
	end
endfunction

// ======================================================================
// CDC synchronizers for DDRAM ack signals
// ======================================================================
reg dwr_ack_s1, dwr_ack_s2;
reg drd_ack_s1, drd_ack_s2;
always @(posedge clk) begin
	dwr_ack_s1 <= ddram_wr_ack; dwr_ack_s2 <= dwr_ack_s1;
	drd_ack_s1 <= ddram_rd_ack; drd_ack_s2 <= drd_ack_s1;
end

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

// ======================================================================
// State machine
// ======================================================================
// BRAM reads have 2-cycle latency (registered address + registered output),
// so every BRAM consumer state is preceded by an address state and a wait state.
localparam S_IDLE         = 6'd0;
localparam S_DD_WR_WAIT   = 6'd1;
localparam S_DD_RD_WAIT   = 6'd2;
// capture (mirror → val_buf)
localparam S_CAP_A        = 6'd3;
localparam S_CAP_AW       = 6'd4;
localparam S_CAP_B        = 6'd5;
localparam S_CAP_BW       = 6'd6;
localparam S_CAP_C        = 6'd7;
// flush (val_buf → DDR valcache)
localparam S_FL_BUSY_HDR  = 6'd8;
localparam S_FL_VRD_A     = 6'd9;
localparam S_FL_VRD_W     = 6'd10;
localparam S_FL_VRD_B     = 6'd11;
localparam S_FL_VWR       = 6'd12;
localparam S_FL_RESP      = 6'd13;
localparam S_FL_HDR_DONE  = 6'd14;
localparam S_FL_FRAME     = 6'd15;
localparam S_FL_DBG       = 6'd16;
localparam S_FL_DBG2      = 6'd17;
// address list check + refill
localparam S_CHK_HDR      = 6'd18;
localparam S_CHK_PARSE    = 6'd19;
localparam S_RF_RD        = 6'd20;
localparam S_RF_MAP_LO    = 6'd21;
localparam S_RF_MAP_HI    = 6'd22;
// ARM config
localparam S_RD_ARMCFG    = 6'd23;
localparam S_PARSE_ARMCFG = 6'd24;
// RTQuery
localparam S_QRY_PARSE    = 6'd25;
localparam S_QRY_RD_REQ   = 6'd26;
localparam S_QRY_FETCH    = 6'd27;
localparam S_QRY_B0       = 6'd28;
localparam S_QRY_BW       = 6'd29;
localparam S_QRY_B1       = 6'd30;
localparam S_QRY_WR_RESP  = 6'd31;
localparam S_QRY_WR_CTRL  = 6'd32;

reg [5:0]  state;
reg [5:0]  return_state;

// Capture may also be entered from the DDR wait states (nested capture), so a
// stalled flush can never delay the frame-boundary snapshot. The pending flag
// is consumed in the same cycle any of those states branches on it.
reg vblank_pending;
always @(posedge clk) begin
	if (reset)
		vblank_pending <= 1'b0;
	else if (vblank_rising)
		vblank_pending <= 1'b1;
	else if ((state == S_IDLE || state == S_DD_WR_WAIT || state == S_DD_RD_WAIT) && vblank_pending)
		vblank_pending <= 1'b0;
end

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;
reg [19:0] ddram_wait_timeout;

// Cached address list
reg [31:0] cached_req_id;
reg [12:0] cached_count;
reg        cache_valid;

// Refill
reg [12:0] rf_idx;
reg [12:0] rf_count;
reg [31:0] rf_req_id;
reg [63:0] addr_word;

// Capture / banks
reg        cap_bank;
reg [12:0] cap_idx;
reg        cap_valid_lat;
reg [31:0] bank_frame  [0:1];
reg [31:0] bank_req_id [0:1];
reg [12:0] bank_count  [0:1];
reg        pending_valid;

// Nested-capture return state (S_IDLE or the interrupted DDR wait state)
reg [5:0]  cap_return_state;

// Flush
reg        flush_busy;
reg        flush_bank;
reg [12:0] fl_byte_ptr;
reg [12:0] fl_word_idx;
reg  [3:0] fl_bytes_in_word;
reg [63:0] collect_buf;

// Debug counters
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_timeout_cnt;
reg  [7:0] dbg_dispatch_cnt;
reg [15:0] dbg_cpuram_cnt;
reg [15:0] dbg_cartram_cnt;
reg [15:0] dbg_first_addr;
reg [15:0] dbg_max_timeout;

// RTQuery registers
reg  [7:0] qry_request_seq;
reg  [7:0] qry_last_seen_seq;
reg  [7:0] qry_num;
reg  [3:0] qry_idx;
reg [31:0] qry_addr;
reg  [7:0] qry_num_bytes;
reg [31:0] qry_value;
reg  [2:0] qry_byte_idx;
reg [10:0] qry_poll_timer;
reg        rtquery_armed = 1'b0;
reg        qv_valid;
wire [16:0] qry_map = nes_to_mirror(qry_addr);

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	ac_we <= 1'b0;
	vb_we <= 1'b0;

	if (reset) begin
		state          <= S_IDLE;
		active         <= 1'b0;
		frame_counter  <= 32'd0;
		ddram_wr_req   <= dwr_ack_s2;
		ddram_rd_req   <= drd_ack_s2;
		cache_valid    <= 1'b0;
		cached_req_id  <= 32'd0;
		cached_count   <= 13'd0;
		pending_valid  <= 1'b0;
		flush_busy     <= 1'b0;
		cap_bank       <= 1'b0;
		cap_return_state <= S_IDLE;
		qry_last_seen_seq <= 8'd0;
		qry_poll_timer <= 11'd0;
		rtquery_armed  <= 1'b0;
		ddram_wait_timeout <= 20'd0;
	end
	else begin
		case (state)

		// =============================================================
		// IDLE: capture on vblank, then flush pending bank, then rtquery
		// =============================================================
		S_IDLE: begin
			active <= 1'b0;
			if (vblank_pending) begin
				// Reset per-pass debug counters
				dbg_ok_cnt       <= 16'd0;
				dbg_timeout_cnt  <= 16'd0;
				dbg_dispatch_cnt <= 8'd0;
				dbg_cpuram_cnt   <= 16'd0;
				dbg_cartram_cnt  <= 16'd0;
				dbg_first_addr   <= 16'd0;
				dbg_max_timeout  <= 16'd0;
				qry_poll_timer   <= 11'd0;

				if (cache_valid && cached_count != 13'd0) begin
					active           <= 1'b1;
					cap_idx          <= 13'd0;
					cap_return_state <= S_IDLE;
					state            <= S_CAP_A;
				end
				else begin
					// No list: empty capture so the frame counter still advances
					bank_frame[cap_bank]  <= frame_counter + 32'd1;
					bank_req_id[cap_bank] <= cached_req_id;
					bank_count[cap_bank]  <= 13'd0;
					frame_counter         <= frame_counter + 32'd1;
					pending_valid         <= 1'b1;
				end
			end
			else if (pending_valid && !flush_busy) begin
				flush_busy    <= 1'b1;
				flush_bank    <= cap_bank;
				pending_valid <= 1'b0;
				cap_bank      <= ~cap_bank;
				state         <= S_FL_BUSY_HDR;
			end
			else if (qry_poll_timer < 11'd2000) begin
				qry_poll_timer <= qry_poll_timer + 11'd1;
			end
			else if (rtquery_armed) begin
				qry_poll_timer <= 11'd0;
				ddram_rd_addr <= QUERY_CTRL_ADDR;
				ddram_rd_req  <= ~ddram_rd_req;
				return_state  <= S_QRY_PARSE;
				state         <= S_DD_RD_WAIT;
			end
			else begin
				qry_poll_timer <= 11'd0;
			end
		end

		// =============================================================
		// Generic DDRAM write wait
		// (vblank triggers a nested capture; the wait resumes after it —
		//  req/ack is level-compared, so an ack arriving meanwhile is kept)
		// =============================================================
		S_DD_WR_WAIT: begin
			if (vblank_pending) begin
				if (cache_valid && cached_count != 13'd0) begin
					active           <= 1'b1;
					cap_idx          <= 13'd0;
					cap_return_state <= S_DD_WR_WAIT;
					state            <= S_CAP_A;
				end
				else begin
					bank_frame[cap_bank]  <= frame_counter + 32'd1;
					bank_req_id[cap_bank] <= cached_req_id;
					bank_count[cap_bank]  <= 13'd0;
					frame_counter         <= frame_counter + 32'd1;
					pending_valid         <= 1'b1;
				end
			end
			else begin
				ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
				if (ddram_wait_timeout[19:4] > dbg_max_timeout)
					dbg_max_timeout <= ddram_wait_timeout[19:4];
				if (ddram_wr_req == dwr_ack_s2) begin
					ddram_wait_timeout <= 20'd0;
					state <= return_state;
				end else if (ddram_wait_timeout >= 20'hFFFFF) begin
					ddram_wait_timeout <= 20'd0;
					dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;
					flush_busy <= 1'b0;
					state <= S_IDLE;
				end
			end
		end

		// =============================================================
		// Generic DDRAM read wait (nested capture: same as above)
		// =============================================================
		S_DD_RD_WAIT: begin
			if (vblank_pending) begin
				if (cache_valid && cached_count != 13'd0) begin
					active           <= 1'b1;
					cap_idx          <= 13'd0;
					cap_return_state <= S_DD_RD_WAIT;
					state            <= S_CAP_A;
				end
				else begin
					bank_frame[cap_bank]  <= frame_counter + 32'd1;
					bank_req_id[cap_bank] <= cached_req_id;
					bank_count[cap_bank]  <= 13'd0;
					frame_counter         <= frame_counter + 32'd1;
					pending_valid         <= 1'b1;
				end
			end
			else begin
				ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
				if (ddram_wait_timeout[19:4] > dbg_max_timeout)
					dbg_max_timeout <= ddram_wait_timeout[19:4];
				if (ddram_rd_req == drd_ack_s2) begin
					ddram_wait_timeout <= 20'd0;
					rd_data <= ddram_rd_dout;
					state   <= return_state;
				end else if (ddram_wait_timeout >= 20'hFFFFF) begin
					ddram_wait_timeout <= 20'd0;
					dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;
					flush_busy <= 1'b0;
					state <= S_IDLE;
				end
			end
		end

		// =============================================================
		// CAPTURE: addr_cache[idx] → mirror → val_buf[cap_bank][idx]
		// =============================================================
		S_CAP_A: begin
			ac_raddr <= cap_idx[11:0];
			state    <= S_CAP_AW;
		end

		S_CAP_AW: state <= S_CAP_B;  // ac_q loads

		S_CAP_B: begin
			mir_raddr     <= ac_q[15:0];
			cap_valid_lat <= ac_q[16];
			if (ac_q[16]) begin
				if (ac_q[15:0] < MIR_REGION2)
					dbg_cpuram_cnt  <= dbg_cpuram_cnt + 16'd1;
				else
					dbg_cartram_cnt <= dbg_cartram_cnt + 16'd1;
			end
			if (!dbg_dispatch_cnt) dbg_first_addr <= ac_q[15:0];
			dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
			state <= S_CAP_BW;
		end

		S_CAP_BW: state <= S_CAP_C;  // mirror_q loads

		S_CAP_C: begin
			vb_waddr <= {cap_bank, cap_idx[11:0]};
			vb_wdata <= cap_valid_lat ? mirror_q : 8'h00;
			vb_we    <= 1'b1;
			if (cap_valid_lat) dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
			cap_idx <= cap_idx + 13'd1;
			if (cap_idx + 13'd1 >= cached_count) begin
				bank_frame[cap_bank]  <= frame_counter + 32'd1;
				bank_req_id[cap_bank] <= cached_req_id;
				bank_count[cap_bank]  <= cached_count;
				frame_counter         <= frame_counter + 32'd1;
				pending_valid         <= 1'b1;
				active                <= 1'b0;
				state                 <= cap_return_state;
			end
			else state <= S_CAP_A;
		end

		// =============================================================
		// FLUSH: busy header, values, response, done header, debug
		// =============================================================
		S_FL_BUSY_HDR: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'h0301, 8'h01, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			fl_byte_ptr      <= 13'd0;
			fl_word_idx      <= 13'd0;
			fl_bytes_in_word <= 4'd0;
			collect_buf      <= 64'd0;
			return_state  <= (bank_count[flush_bank] == 13'd0) ? S_FL_RESP : S_FL_VRD_A;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_VRD_A: begin
			vb_raddr <= {flush_bank, fl_byte_ptr[11:0]};
			state    <= S_FL_VRD_W;
		end

		S_FL_VRD_W: state <= S_FL_VRD_B;  // vb_q loads

		S_FL_VRD_B: begin
			collect_buf[fl_bytes_in_word[2:0]*8 +: 8] <= vb_q;
			fl_bytes_in_word <= fl_bytes_in_word + 4'd1;
			fl_byte_ptr      <= fl_byte_ptr + 13'd1;
			if (fl_bytes_in_word[2:0] == 3'd7 || (fl_byte_ptr + 13'd1 >= bank_count[flush_bank]))
				state <= S_FL_VWR;
			else
				state <= S_FL_VRD_A;
		end

		S_FL_VWR: begin
			ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, fl_word_idx};
			ddram_wr_din  <= collect_buf;
			ddram_wr_be   <= (fl_bytes_in_word == 4'd8) ? 8'hFF
			                 : ((8'd1 << fl_bytes_in_word[2:0]) - 8'd1);
			ddram_wr_req  <= ~ddram_wr_req;
			fl_word_idx      <= fl_word_idx + 13'd1;
			fl_bytes_in_word <= 4'd0;
			collect_buf      <= 64'd0;
			return_state  <= (fl_byte_ptr >= bank_count[flush_bank]) ? S_FL_RESP : S_FL_VRD_A;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_RESP: begin
			ddram_wr_addr <= VALCACHE_BASE;
			ddram_wr_din  <= {bank_frame[flush_bank], bank_req_id[flush_bank]};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_FL_HDR_DONE;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_HDR_DONE: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'h0301, 8'h00, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_FL_FRAME;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_FRAME: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd1;
			ddram_wr_din  <= {32'd0, bank_frame[flush_bank]};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_FL_DBG;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h03, dbg_dispatch_cnt, 16'd0, dbg_timeout_cnt, dbg_ok_cnt};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_FL_DBG2;
			state         <= S_DD_WR_WAIT;
		end

		S_FL_DBG2: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd3;
			ddram_wr_din  <= {dbg_first_addr, dbg_cpuram_cnt, dbg_cartram_cnt, dbg_max_timeout};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_CHK_HDR;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Address list check: re-fetch and pre-map on request_id change
		// =============================================================
		S_CHK_HDR: begin
			flush_busy    <= 1'b0;
			ddram_rd_addr <= ADDRLIST_BASE;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_CHK_PARSE;
			state         <= S_DD_RD_WAIT;
		end

		S_CHK_PARSE: begin
			if (!cache_valid || rd_data[63:32] != cached_req_id) begin
				rf_req_id <= rd_data[63:32];
				if (rd_data[31:0] == 32'd0) begin
					cached_req_id <= rd_data[63:32];
					cached_count  <= 13'd0;
					cache_valid   <= 1'b1;
					state         <= S_RD_ARMCFG;
				end else begin
					rf_count <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
					            MAX_ADDRS : rd_data[12:0];
					rf_idx   <= 13'd0;
					// Invalidate while refilling: an aborted refill must not
					// leave a cache mixing old and new list entries
					cache_valid <= 1'b0;
					state    <= S_RF_RD;
				end
			end
			else state <= S_RD_ARMCFG;
		end

		S_RF_RD: begin
			ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, rf_idx[12:1]};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_RF_MAP_LO;
			state         <= S_DD_RD_WAIT;
		end

		S_RF_MAP_LO: begin
			addr_word <= rd_data;
			ac_waddr  <= rf_idx[11:0];
			ac_wdata  <= nes_to_mirror(rd_data[31:0]);
			ac_we     <= 1'b1;
			rf_idx    <= rf_idx + 13'd1;
			if (rf_idx + 13'd1 >= rf_count) begin
				cached_req_id <= rf_req_id;
				cached_count  <= rf_count;
				cache_valid   <= 1'b1;
				state         <= S_RD_ARMCFG;
			end
			else state <= S_RF_MAP_HI;
		end

		S_RF_MAP_HI: begin
			ac_waddr <= rf_idx[11:0];
			ac_wdata <= nes_to_mirror(addr_word[63:32]);
			ac_we    <= 1'b1;
			rf_idx   <= rf_idx + 13'd1;
			if (rf_idx + 13'd1 >= rf_count) begin
				cached_req_id <= rf_req_id;
				cached_count  <= rf_count;
				cache_valid   <= 1'b1;
				state         <= S_RD_ARMCFG;
			end
			else state <= S_RF_RD;
		end

		// =============================================================
		// Read ARM-written config byte once per pass
		// =============================================================
		S_RD_ARMCFG: begin
			ddram_rd_addr <= ARM_CFG_ADDR;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ARMCFG;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_ARMCFG: begin
			rtquery_armed <= rd_data[0];
			state <= S_IDLE;
		end

		// =============================================================
		// Realtime Query: served from the mirror BRAM (instant, no SDRAM)
		// =============================================================
		S_QRY_PARSE: begin
			if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
				qry_request_seq <= rd_data[7:0];
				qry_num         <= (rd_data[15:8] > MAX_RT_QUERIES) ?
				                   MAX_RT_QUERIES : rd_data[15:8];
				qry_idx         <= 4'd0;
				state           <= S_QRY_RD_REQ;
			end else begin
				state <= S_IDLE;
			end
		end

		S_QRY_RD_REQ: begin
			ddram_rd_addr <= QUERY_REQ_BASE + {25'd0, qry_idx};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_QRY_FETCH;
			state         <= S_DD_RD_WAIT;
		end

		S_QRY_FETCH: begin
			qry_addr      <= rd_data[31:0];
			qry_num_bytes <= (rd_data[39:32] == 8'd0) ? 8'd1 : rd_data[39:32];
			qry_value     <= 32'd0;
			qry_byte_idx  <= 3'd0;
			state         <= S_QRY_B0;
		end

		S_QRY_B0: begin
			mir_raddr <= qry_map[15:0];
			qv_valid  <= qry_map[16];
			state     <= S_QRY_BW;
		end

		S_QRY_BW: state <= S_QRY_B1;  // mirror_q loads

		S_QRY_B1: begin
			qry_value    <= qry_value | ({24'd0, (qv_valid ? mirror_q : 8'h00)} << (qry_byte_idx * 8));
			qry_byte_idx <= qry_byte_idx + 3'd1;
			qry_addr     <= qry_addr + 32'd1;
			if (qry_byte_idx + 3'd1 >= qry_num_bytes[2:0])
				state <= S_QRY_WR_RESP;
			else
				state <= S_QRY_B0;
		end

		S_QRY_WR_RESP: begin
			ddram_wr_addr <= QUERY_RESP_BASE + {25'd0, qry_idx};
			ddram_wr_din  <= {32'd0, qry_value};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			qry_idx       <= qry_idx + 4'd1;
			if ({4'd0, qry_idx} + 8'd1 >= qry_num)
				return_state <= S_QRY_WR_CTRL;
			else
				return_state <= S_QRY_RD_REQ;
			state <= S_DD_WR_WAIT;
		end

		S_QRY_WR_CTRL: begin
			qry_last_seen_seq <= qry_request_seq;
			ddram_wr_addr     <= QUERY_CTRL_ADDR;
			ddram_wr_din      <= {24'd0, qry_request_seq, 16'd0, qry_num[7:0], qry_request_seq};
			ddram_wr_be       <= 8'hFF;
			ddram_wr_req      <= ~ddram_wr_req;
			return_state      <= S_IDLE;
			state             <= S_DD_WR_WAIT;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
