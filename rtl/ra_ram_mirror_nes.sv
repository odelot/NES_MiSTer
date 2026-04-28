// RetroAchievements RAM Mirror for NES — Option C + RTQuery (Tier 1)
//
// Each VBlank, reads a list of specific addresses from DDRAM (written by ARM),
// fetches byte values from SDRAM (CPU RAM or CART RAM), and writes them back
// to DDRAM for the ARM to read via rcheevos.
//
// NES address mapping:
//   addr < 0x0800  → CPU RAM  → SDRAM 0x380000 + addr
//   addr < 0x2000  → CPU RAM  → SDRAM 0x380000 + (addr & 0x7FF)  (mirrors)
//   0x6000-0x7FFF  → CART RAM → SDRAM 0x3C0000 + (addr - 0x6000)
//
// DDRAM Layout (at DDRAM_BASE, ARM phys 0x3D000000):
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

module ra_ram_mirror_nes #(
	parameter [28:0] DDRAM_BASE = 29'h07A00000  // ARM phys 0x3D000000 >> 3
)(
	input             clk,           // clk_sys (NES ~21.477 MHz)
	input             reset,
	input             vblank,

	// SDRAM read interface (ch2)
	output reg [24:0] sdram_addr,
	output reg        sdram_rd,
	input       [7:0] sdram_dout,
	input             sdram_busy,

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

// NES SDRAM address bases
localparam [24:0] CPURAM_BASE  = 25'h380000;   // 2KB CPU RAM
localparam [24:0] CARTRAM_BASE = 25'h3C0000;   // 8KB CART RAM

// Realtime query mailbox
localparam [28:0] QUERY_CTRL_ADDR = DDRAM_BASE + 29'hA000;
localparam [28:0] QUERY_REQ_BASE  = DDRAM_BASE + 29'hA001;
localparam [28:0] QUERY_RESP_BASE = DDRAM_BASE + 29'hA011;
localparam [3:0]  MAX_RT_QUERIES  = 4'd16;

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
// VBlank edge detection + sticky pending
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

reg vblank_pending;
always @(posedge clk) begin
	if (reset)
		vblank_pending <= 1'b0;
	else if (vblank_rising)
		vblank_pending <= 1'b1;
	else if (state == S_IDLE && vblank_pending)
		vblank_pending <= 1'b0;
end

// ======================================================================
// SDRAM busy edge detection (data valid when busy falls)
// ======================================================================
reg sdram_busy_prev;
wire sdram_data_valid = sdram_busy_prev & ~sdram_busy;
always @(posedge clk) sdram_busy_prev <= sdram_busy;

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE        = 5'd0;
localparam S_DD_WR_WAIT  = 5'd1;
localparam S_DD_RD_WAIT  = 5'd2;
localparam S_READ_HDR    = 5'd3;
localparam S_PARSE_HDR   = 5'd4;
localparam S_READ_PAIR   = 5'd5;
localparam S_PARSE_ADDR  = 5'd6;
localparam S_DISPATCH    = 5'd7;
localparam S_FETCH_SDRAM = 5'd8;
localparam S_SDRAM_WAIT  = 5'd9;
localparam S_STORE_VAL   = 5'd10;
localparam S_FLUSH_BUF   = 5'd11;
localparam S_WRITE_RESP  = 5'd12;
localparam S_WR_HDR0     = 5'd13;
localparam S_WR_HDR1     = 5'd14;
localparam S_WR_DBG      = 5'd15;
localparam S_WR_DBG2     = 5'd16;
// RTQuery states
localparam S_QRY_PARSE   = 5'd17;
localparam S_QRY_RD_REQ  = 5'd18;
localparam S_QRY_FETCH   = 5'd19;
localparam S_QRY_WAIT    = 5'd20;
localparam S_QRY_WR_RESP = 5'd21;
localparam S_QRY_WR_CTRL = 5'd22;

reg [4:0]  state;
reg [4:0]  return_state;

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;
reg [31:0] req_count;
reg [31:0] req_id;
reg [12:0] addr_idx;
reg [63:0] addr_word;
reg [31:0] cur_addr;
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;
reg [12:0] val_word_idx;

reg [15:0] timeout;
reg  [7:0] fetch_byte;

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
reg  [9:0] qry_poll_timer;

// DDRAM wait timeout
reg [19:0] ddram_wait_timeout;

// ======================================================================
// NES address → SDRAM address mapping
// ======================================================================
function [24:0] nes_to_sdram;
	input [31:0] addr;
	begin
		if (addr < 32'h2000)
			nes_to_sdram = CPURAM_BASE + {14'd0, addr[10:0] & 11'h7FF};
		else if (addr >= 32'h6000 && addr < 32'h8000)
			nes_to_sdram = CARTRAM_BASE + {12'd0, addr[12:0] - 13'h6000};
		else
			nes_to_sdram = 25'h0; // Invalid — should not happen
	end
endfunction

wire addr_is_cpuram  = (cur_addr < 32'h2000);
wire addr_is_cartram = (cur_addr >= 32'h6000 && cur_addr < 32'h8000);
wire addr_valid      = addr_is_cpuram | addr_is_cartram;

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	// Default: deassert SDRAM rd (pulse-based)
	sdram_rd <= 1'b0;

	if (reset) begin
		state         <= S_IDLE;
		active        <= 1'b0;
		frame_counter <= 32'd0;
		ddram_wr_req  <= dwr_ack_s2;
		ddram_rd_req  <= drd_ack_s2;
		qry_last_seen_seq <= 8'd0;
		qry_poll_timer <= 10'd0;
	end
	else begin
		case (state)

		// =============================================================
		// IDLE: Wait for VBlank
		// =============================================================
		S_IDLE: begin
			active <= 1'b0;
			if (vblank_pending) begin
				active <= 1'b1;
				qry_poll_timer <= 10'd0;
				// Reset debug counters
				dbg_ok_cnt       <= 16'd0;
				dbg_timeout_cnt  <= 16'd0;
				dbg_dispatch_cnt <= 8'd0;
				dbg_cpuram_cnt   <= 16'd0;
				dbg_cartram_cnt  <= 16'd0;
				dbg_first_addr   <= 16'd0;
				dbg_max_timeout  <= 16'd0;
				// Write header with busy=1
				ddram_wr_addr <= DDRAM_BASE;
				ddram_wr_din  <= {16'h0300, 8'h01, 8'd0, 32'h52414348};
				ddram_wr_be   <= 8'hFF;
				ddram_wr_req  <= ~ddram_wr_req;
				return_state  <= S_READ_HDR;
				state         <= S_DD_WR_WAIT;
			end
			else if (qry_poll_timer < 10'd1000) begin
				qry_poll_timer <= qry_poll_timer + 10'd1;
			end
			else begin
				// Poll RTQuery mailbox
				qry_poll_timer <= 10'd0;
				ddram_rd_addr <= QUERY_CTRL_ADDR;
				ddram_rd_req  <= ~ddram_rd_req;
				return_state  <= S_QRY_PARSE;
				state         <= S_DD_RD_WAIT;
			end
		end

		// =============================================================
		// Generic DDRAM write wait
		// =============================================================
		S_DD_WR_WAIT: begin
			ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
			if (ddram_wr_req == dwr_ack_s2) begin
				ddram_wait_timeout <= 20'd0;
				state <= return_state;
			end else if (ddram_wait_timeout >= 20'hFFFFF) begin
				ddram_wait_timeout <= 20'd0;
				state <= S_IDLE;
			end
		end

		// =============================================================
		// Generic DDRAM read wait
		// =============================================================
		S_DD_RD_WAIT: begin
			ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
			if (ddram_rd_req == drd_ack_s2) begin
				ddram_wait_timeout <= 20'd0;
				rd_data <= ddram_rd_dout;
				state   <= return_state;
			end else if (ddram_wait_timeout >= 20'hFFFFF) begin
				ddram_wait_timeout <= 20'd0;
				state <= S_IDLE;
			end
		end

		// =============================================================
		// Read address list header from DDRAM
		// =============================================================
		S_READ_HDR: begin
			ddram_rd_addr <= ADDRLIST_BASE;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_HDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_HDR: begin
			req_id <= rd_data[63:32];
			if (rd_data[31:0] == 32'd0) begin
				req_count <= 32'd0;
				state     <= S_WRITE_RESP;
			end else begin
				req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
				                {19'd0, MAX_ADDRS} : rd_data[31:0];
				addr_idx     <= 13'd0;
				collect_cnt  <= 4'd0;
				collect_buf  <= 64'd0;
				val_word_idx <= 13'd0;
				state        <= S_READ_PAIR;
			end
		end

		// =============================================================
		// Read address pair (2 addrs per 64-bit word)
		// =============================================================
		S_READ_PAIR: begin
			ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, addr_idx[12:1]};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ADDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_ADDR: begin
			if (!addr_idx[0]) begin
				addr_word <= rd_data;
				cur_addr  <= rd_data[31:0];
			end else begin
				cur_addr <= addr_word[63:32];
			end
			state <= S_DISPATCH;
		end

		// =============================================================
		// Dispatch: map NES addr to SDRAM and fetch
		// =============================================================
		S_DISPATCH: begin
			dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
			if (!dbg_dispatch_cnt)
				dbg_first_addr <= cur_addr[15:0];

			if (addr_valid) begin
				if (addr_is_cpuram)
					dbg_cpuram_cnt <= dbg_cpuram_cnt + 16'd1;
				else
					dbg_cartram_cnt <= dbg_cartram_cnt + 16'd1;
				state <= S_FETCH_SDRAM;
			end else begin
				// Invalid address: store 0
				fetch_byte <= 8'd0;
				state <= S_STORE_VAL;
			end
		end

		// =============================================================
		// SDRAM fetch: pulse rd, wait for data
		// =============================================================
		S_FETCH_SDRAM: begin
			sdram_addr <= nes_to_sdram(cur_addr);
			sdram_rd   <= 1'b1;
			timeout    <= 16'd0;
			state      <= S_SDRAM_WAIT;
		end

		S_SDRAM_WAIT: begin
			timeout <= timeout + 16'd1;

			// Re-pulse every 16 cycles if not yet accepted
			if (~sdram_busy && ~sdram_busy_prev && timeout != 16'd0 && timeout[3:0] == 4'd0)
				sdram_rd <= 1'b1;

			if (timeout >= 16'hFFFF) begin
				fetch_byte <= 8'd0;
				dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;
				state <= S_STORE_VAL;
			end
			else if (sdram_data_valid) begin
				fetch_byte <= sdram_dout;
				dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
				if (timeout > dbg_max_timeout)
					dbg_max_timeout <= timeout;
				state <= S_STORE_VAL;
			end
		end

		// =============================================================
		// Store byte in collect buffer
		// =============================================================
		S_STORE_VAL: begin
			case (collect_cnt[2:0])
				3'd0: collect_buf[ 7: 0] <= fetch_byte;
				3'd1: collect_buf[15: 8] <= fetch_byte;
				3'd2: collect_buf[23:16] <= fetch_byte;
				3'd3: collect_buf[31:24] <= fetch_byte;
				3'd4: collect_buf[39:32] <= fetch_byte;
				3'd5: collect_buf[47:40] <= fetch_byte;
				3'd6: collect_buf[55:48] <= fetch_byte;
				3'd7: collect_buf[63:56] <= fetch_byte;
			endcase
			collect_cnt <= collect_cnt + 4'd1;
			addr_idx    <= addr_idx + 13'd1;

			if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0]))
				state <= S_FLUSH_BUF;
			else if (addr_idx[0])
				state <= S_READ_PAIR;
			else
				state <= S_PARSE_ADDR;
		end

		// =============================================================
		// Flush collect buffer to DDRAM
		// =============================================================
		S_FLUSH_BUF: begin
			ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, val_word_idx};
			ddram_wr_din  <= collect_buf;
			ddram_wr_be   <= (collect_cnt == 4'd8) ? 8'hFF
			                 : ((8'd1 << collect_cnt[2:0]) - 8'd1);
			ddram_wr_req  <= ~ddram_wr_req;
			val_word_idx  <= val_word_idx + 13'd1;
			collect_cnt   <= 4'd0;
			collect_buf   <= 64'd0;

			if (addr_idx >= req_count[12:0])
				return_state <= S_WRITE_RESP;
			else if (!addr_idx[0])
				return_state <= S_READ_PAIR;
			else
				return_state <= S_PARSE_ADDR;
			state <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write response header
		// =============================================================
		S_WRITE_RESP: begin
			ddram_wr_addr <= VALCACHE_BASE;
			ddram_wr_din  <= {frame_counter + 32'd1, req_id};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR0;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR0: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'h0300, 8'h00, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR1;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR1: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd1;
			ddram_wr_din  <= {32'd0, frame_counter + 32'd1};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			frame_counter <= frame_counter + 32'd1;
			return_state  <= S_WR_DBG;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h03, dbg_dispatch_cnt, 16'd0, dbg_timeout_cnt, dbg_ok_cnt};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG2;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_DBG2: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd3;
			ddram_wr_din  <= {dbg_first_addr, dbg_cpuram_cnt, dbg_cartram_cnt, dbg_max_timeout};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_IDLE;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Realtime Query States
		// =============================================================
		S_QRY_PARSE: begin
			if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
				qry_request_seq <= rd_data[7:0];
				qry_num         <= (rd_data[15:8] > {4'd0, MAX_RT_QUERIES}) ?
				                   {4'd0, MAX_RT_QUERIES} : rd_data[15:8];
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
			// Map NES addr and issue SDRAM read
			sdram_addr <= nes_to_sdram(rd_data[31:0]);
			sdram_rd   <= 1'b1;
			timeout    <= 16'd0;
			state      <= S_QRY_WAIT;
		end

		S_QRY_WAIT: begin
			sdram_rd <= 1'b0;
			timeout  <= timeout + 16'd1;

			// Re-pulse
			if (~sdram_busy && ~sdram_busy_prev && timeout != 16'd0 && timeout[3:0] == 4'd0)
				sdram_rd <= 1'b1;

			if (sdram_data_valid) begin
				qry_value    <= qry_value | ({24'd0, sdram_dout} << (qry_byte_idx * 8));
				qry_byte_idx <= qry_byte_idx + 3'd1;
				if (qry_byte_idx + 3'd1 >= qry_num_bytes[2:0]) begin
					state <= S_QRY_WR_RESP;
				end else begin
					qry_addr   <= qry_addr + 32'd1;
					sdram_addr <= nes_to_sdram(qry_addr + 32'd1);
					sdram_rd   <= 1'b1;
					timeout    <= 16'd0;
				end
			end
			if (timeout >= 16'hFFFF) begin
				qry_byte_idx <= qry_byte_idx + 3'd1;
				if (qry_byte_idx + 3'd1 >= qry_num_bytes[2:0])
					state <= S_QRY_WR_RESP;
				else begin
					qry_addr   <= qry_addr + 32'd1;
					sdram_addr <= nes_to_sdram(qry_addr + 32'd1);
					sdram_rd   <= 1'b1;
					timeout    <= 16'd0;
				end
			end
		end

		S_QRY_WR_RESP: begin
			ddram_wr_addr <= QUERY_RESP_BASE + {25'd0, qry_idx};
			ddram_wr_din  <= {32'd0, qry_value};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			qry_idx       <= qry_idx + 4'd1;
			if (qry_idx + 4'd1 >= qry_num[3:0])
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
