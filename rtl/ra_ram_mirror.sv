// RetroAchievements RAM Mirror Module
// Copyright (c) 2026
//
// Copies emulated system RAM from SDRAM to DDRAM every VBlank,
// making it readable by the ARM/HPS side via shared DDR3 memory.
//
// This module is designed to be console-agnostic. Each core configures:
//   - REGION_COUNT: how many SDRAM regions to mirror
//   - Region addresses and sizes via parameters
//   - VBlank signal source
//
// DDRAM Layout (written by this module, read by ARM):
//   Offset 0x00: Header (64 bits)
//     [31:0]  = magic number 0x52414348 ("RACH")
//     [39:32] = region_count
//     [47:40] = flags (bit 0 = transfer in progress)
//     [63:48] = reserved
//   Offset 0x08: Frame counter (64 bits)
//     [31:0]  = frame counter (increments each VBlank)
//     [63:32] = reserved
//   Offset 0x10: Region descriptor table (8 bytes per region)
//     [31:0]  = SDRAM source address (25-bit, zero-extended)
//     [47:32] = size in bytes (16-bit, max 65535 per region)
//     [63:48] = DDRAM destination offset from base (relative byte offset)
//     Repeat for each region...
//   Offset 0x10 + REGION_COUNT*8: Padding to 0x100
//   Offset 0x100: Region 0 data
//   Offset 0x100 + region0_size (aligned to 8): Region 1 data
//   ...
//
// Protocol:
//   ARM reads frame_counter. If changed since last read, new data is available.
//   The 'busy' flag (header bit 40) is set during transfer and cleared after.
//   ARM should avoid reading while busy=1 (optional, data is still coherent
//   within each 64-bit write).

module ra_ram_mirror #(
	parameter REGION_COUNT    = 2,          // Number of SDRAM regions to mirror
	parameter [24:0] REGION0_SDRAM_ADDR = 25'h380000,  // CPU-RAM
	parameter [15:0] REGION0_SIZE       = 16'd2048,     // 2 KB
	parameter [24:0] REGION1_SDRAM_ADDR = 25'h3C0000,  // CARTRAM
	parameter [15:0] REGION1_SIZE       = 16'd8192,     // 8 KB (typical, configurable)
	parameter [24:0] REGION2_SDRAM_ADDR = 25'h0,
	parameter [15:0] REGION2_SIZE       = 16'd0,
	parameter [24:0] REGION3_SDRAM_ADDR = 25'h0,
	parameter [15:0] REGION3_SIZE       = 16'd0
)(
	input             clk,           // System clock (same as DDRAM_CLK)
	input             reset,

	// VBlank trigger (directly from the emulated system)
	input             vblank,

	// SDRAM read interface (directly connects to an SDRAM channel)
	output reg [24:0] sdram_addr,
	output reg        sdram_rd,
	input       [7:0] sdram_dout,
	input             sdram_busy,

	// DDRAM write interface (directly to ddram arbiter channel)
	output reg [27:1] ddram_addr,
	output reg [63:0] ddram_din,
	output reg        ddram_req,
	output reg  [7:0] ddram_be,
	output            ddram_rnw,     // Always 0 (write-only)
	input             ddram_ready,

	// Configuration
	input      [27:1] ddram_base_addr, // Base address in DDRAM (e.g., 0x3A00000 >> 1)

	// Status
	output reg        busy,
	output reg [31:0] dbg_frame_counter  // Debug: readable frame counter for LED/status
);

assign ddram_rnw = 1'b0; // Write-only to DDRAM

// Region configuration stored in arrays for flexibility
reg [24:0] region_sdram_addr [0:3];
reg [15:0] region_size       [0:3];
reg [15:0] region_ddram_off  [0:3]; // Byte offset within DDRAM data area

// Frame counter
reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

// State machine
localparam S_IDLE          = 4'd0;
localparam S_WRITE_HEADER  = 4'd1;
localparam S_WRITE_FRAME   = 4'd2;
localparam S_WRITE_DESCS   = 4'd3;
localparam S_READ_SDRAM    = 4'd4;
localparam S_WAIT_SDRAM    = 4'd5;
localparam S_COLLECT_BYTES = 4'd6;
localparam S_WRITE_DDRAM   = 4'd7;
localparam S_WAIT_DDRAM    = 4'd8;
localparam S_FINISH_HEADER = 4'd9;
localparam S_WAIT_DDRAM2   = 4'd10;
localparam S_NEXT_REGION   = 4'd11;
localparam S_ABORT         = 4'd12;  // Abort transfer (timeout recovery)

reg [3:0]  state;
reg [1:0]  region_idx;      // Current region being transferred
reg [15:0] byte_idx;        // Current byte index within region
reg [63:0] collect_buf;     // Accumulator for 8 bytes → 1 DDRAM word
reg [2:0]  collect_cnt;     // Bytes collected (0-7)
reg [3:0]  desc_idx;        // Descriptor index being written
reg [15:0] write_byte_off;  // Pre-computed byte offset for DDRAM write
reg [15:0] sdram_timeout;   // Safety timeout for SDRAM reads

// VBlank edge detection
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;

// Pre-compute region data area base offset (0x100 = header + descriptors)
localparam DATA_AREA_OFFSET = 16'h0100;

// SDRAM busy edge detection (data is valid when busy falls)
reg sdram_busy_prev;
wire sdram_data_valid = sdram_busy_prev & ~sdram_busy;

// Initialize region config from parameters
initial begin
	region_sdram_addr[0] = REGION0_SDRAM_ADDR;
	region_sdram_addr[1] = REGION1_SDRAM_ADDR;
	region_sdram_addr[2] = REGION2_SDRAM_ADDR;
	region_sdram_addr[3] = REGION3_SDRAM_ADDR;
	region_size[0]       = REGION0_SIZE;
	region_size[1]       = REGION1_SIZE;
	region_size[2]       = REGION2_SIZE;
	region_size[3]       = REGION3_SIZE;

	// Calculate DDRAM offsets for each region (8-byte aligned)
	region_ddram_off[0]  = DATA_AREA_OFFSET;
	region_ddram_off[1]  = DATA_AREA_OFFSET + ((REGION0_SIZE + 16'd7) & 16'hFFF8);
	region_ddram_off[2]  = DATA_AREA_OFFSET + ((REGION0_SIZE + 16'd7) & 16'hFFF8)
	                                        + ((REGION1_SIZE + 16'd7) & 16'hFFF8);
	region_ddram_off[3]  = DATA_AREA_OFFSET + ((REGION0_SIZE + 16'd7) & 16'hFFF8)
	                                        + ((REGION1_SIZE + 16'd7) & 16'hFFF8)
	                                        + ((REGION2_SIZE + 16'd7) & 16'hFFF8);

	frame_counter = 0;
	state = S_IDLE;
	busy = 0;
end

always @(posedge clk) begin
	vblank_prev     <= vblank;
	sdram_busy_prev <= sdram_busy;

	// Default: deassert requests (active for 1 cycle only)
	sdram_rd  <= 1'b0;
	ddram_req <= 1'b0;

	if (reset) begin
		state         <= S_IDLE;
		busy          <= 1'b0;
		frame_counter <= 32'd0;
	end
	else begin
		case (state)
		// -------------------------------------------------------
		// IDLE: Wait for VBlank rising edge
		// -------------------------------------------------------
		S_IDLE: begin
			if (vblank_rising) begin
				frame_counter <= frame_counter + 1'd1;
				busy          <= 1'b1;
				region_idx    <= 2'd0;
				state         <= S_WRITE_HEADER;
			end
		end

		// -------------------------------------------------------
		// Write header with busy flag SET (magic + region_count + busy=1)
		// -------------------------------------------------------
		S_WRITE_HEADER: begin
			ddram_addr <= ddram_base_addr;
			ddram_din  <= {16'd0, 8'h01, REGION_COUNT[7:0], 32'h52414348}; // "RACH", count, busy=1
			ddram_be   <= 8'hFF;
			ddram_req  <= 1'b1;
			state      <= S_WRITE_FRAME;
		end

		// Write frame counter
		S_WRITE_FRAME: begin
			if (ddram_ready) begin
				ddram_addr <= ddram_base_addr + 27'd4; // +8 bytes (addr is [27:1], so +4 = +8 bytes)
				ddram_din  <= {32'd0, frame_counter};
				ddram_be   <= 8'hFF;
				ddram_req  <= 1'b1;
				desc_idx   <= 4'd0;
				state      <= S_WRITE_DESCS;
			end
		end

		// Write region descriptors (one 64-bit word per region)
		S_WRITE_DESCS: begin
			if (ddram_ready) begin
				if (desc_idx < REGION_COUNT[3:0]) begin
					// Descriptor at byte offset 0x10 + desc_idx*8
					// In [27:1] units: base + 8 + desc_idx*4
					ddram_addr <= ddram_base_addr + 27'd8 + {21'd0, desc_idx[1:0], 2'b00};
					ddram_din  <= {region_ddram_off[desc_idx], region_size[desc_idx],
					               7'd0, region_sdram_addr[desc_idx][24:0]};
					ddram_be   <= 8'hFF;
					ddram_req  <= 1'b1;
					desc_idx   <= desc_idx + 4'd1;
				end
				else begin
					// All descriptors written, start copying region data
					byte_idx    <= 16'd0;
					collect_cnt <= 3'd0;
					collect_buf <= 64'd0;
					state       <= S_READ_SDRAM;
				end
			end
		end

		// -------------------------------------------------------
		// Read bytes from SDRAM, 1 byte at a time via ch2
		// -------------------------------------------------------
		S_READ_SDRAM: begin
			if (byte_idx >= region_size[region_idx]) begin
				// Region done. Flush remaining collected bytes if any.
				if (collect_cnt != 3'd0) begin
					// Compute write offset: start of the current 8-byte chunk
					write_byte_off <= region_ddram_off[region_idx] + (byte_idx & 16'hFFF8);
					state          <= S_WRITE_DDRAM;
				end
				else begin
					state <= S_NEXT_REGION;
				end
			end
			else begin
				sdram_addr <= region_sdram_addr[region_idx] + {9'd0, byte_idx};
				sdram_rd   <= 1'b1;
				sdram_timeout <= 16'd0;
				state      <= S_WAIT_SDRAM;
			end
		end

		// Wait for SDRAM to return data
		S_WAIT_SDRAM: begin
			sdram_timeout <= sdram_timeout + 16'd1;

			// RETRY LOGIC: The SDRAM arbiter uses rising-edge detection
			// (~old_rd & rd). It clears old_rd when rd goes LOW (old_rd <= old_rd & rd).
			// If we hold rd HIGH continuously, old_rd stays 1 after the first
			// detection and no subsequent edges are seen.
			//
			// Instead, we RE-PULSE sdram_rd every 16 clk cycles (~64 clk85 cycles).
			// Between pulses, sdram_rd is LOW (from the default at the top of the
			// always block), which lets old_rd clear back to 0. Each re-pulse then
			// creates a fresh rising edge that the arbiter can detect.
			//
			// We only retry if sdram_busy is still low (request not yet accepted).
			// Once busy goes high, the SDRAM has accepted our request — just wait
			// for it to complete (sdram_data_valid).
			if (~sdram_busy && ~sdram_busy_prev && sdram_timeout != 16'd0 && sdram_timeout[3:0] == 4'd0) begin
				sdram_rd <= 1'b1;
			end

			// Safety: abort if SDRAM doesn't respond within ~3ms (~65536 clk cycles)
			// This prevents permanent hang if SDRAM access is blocked by arbitration.
			if (sdram_timeout >= 16'hFFFF) begin
				state <= S_ABORT;
			end
			else if (sdram_data_valid) begin
				// Place byte into correct position in 64-bit word (little-endian)
				case (collect_cnt)
					3'd0: collect_buf[7:0]   <= sdram_dout;
					3'd1: collect_buf[15:8]  <= sdram_dout;
					3'd2: collect_buf[23:16] <= sdram_dout;
					3'd3: collect_buf[31:24] <= sdram_dout;
					3'd4: collect_buf[39:32] <= sdram_dout;
					3'd5: collect_buf[47:40] <= sdram_dout;
					3'd6: collect_buf[55:48] <= sdram_dout;
					3'd7: collect_buf[63:56] <= sdram_dout;
				endcase
				collect_cnt <= collect_cnt + 3'd1;
				byte_idx    <= byte_idx + 16'd1;

				if (collect_cnt == 3'd7) begin
					// 8 bytes collected. Compute DDRAM write offset.
					// byte_idx hasn't incremented yet in this cycle, so the chunk
					// started at (byte_idx - 7). After increment, byte_idx = old+1.
					// Chunk start byte offset = region_ddram_off + (byte_idx & ~7)
					// But byte_idx is old value here (pre-increment), so chunk
					// start = region_ddram_off + ((byte_idx + 1) - 8) = (byte_idx - 7)
					// Simpler: chunk starts at byte_idx & 0xFFF8 (8-byte aligned)
					write_byte_off <= region_ddram_off[region_idx] + (byte_idx & 16'hFFF8);
					state          <= S_WRITE_DDRAM;
				end
				else begin
					state <= S_READ_SDRAM;
				end
			end
		end

		// Write collected 64-bit word to DDRAM
		S_WRITE_DDRAM: begin
			// Convert byte offset to [27:1] half-word offset and add to base
			ddram_addr <= ddram_base_addr + {12'd0, write_byte_off[15:1]};

			// Set byte enables for partial writes (last chunk may be < 8 bytes)
			case (collect_cnt)
				3'd0: ddram_be <= 8'hFF;   // 0 means 8 (full word, wrapped)
				3'd1: ddram_be <= 8'h01;
				3'd2: ddram_be <= 8'h03;
				3'd3: ddram_be <= 8'h07;
				3'd4: ddram_be <= 8'h0F;
				3'd5: ddram_be <= 8'h1F;
				3'd6: ddram_be <= 8'h3F;
				3'd7: ddram_be <= 8'h7F;
			endcase

			ddram_din   <= collect_buf;
			ddram_req   <= 1'b1;
			state       <= S_WAIT_DDRAM;
		end

		// Wait for DDRAM write to complete
		S_WAIT_DDRAM: begin
			if (ddram_ready) begin
				collect_buf <= 64'd0;
				collect_cnt <= 3'd0;
				if (byte_idx < region_size[region_idx]) begin
					state <= S_READ_SDRAM;
				end
				else begin
					state <= S_NEXT_REGION;
				end
			end
		end

		// Move to next region or finish
		S_NEXT_REGION: begin
			if (region_idx + 2'd1 < REGION_COUNT) begin
				region_idx  <= region_idx + 2'd1;
				byte_idx    <= 16'd0;
				collect_cnt <= 3'd0;
				collect_buf <= 64'd0;
				state       <= S_READ_SDRAM;
			end
			else begin
				// All regions done, update header with busy=0
				state <= S_FINISH_HEADER;
			end
		end

		// -------------------------------------------------------
		// Write final header with busy=0
		// -------------------------------------------------------
		S_FINISH_HEADER: begin
			ddram_addr <= ddram_base_addr;
			ddram_din  <= {16'd0, 8'h00, REGION_COUNT[7:0], 32'h52414348}; // busy=0
			ddram_be   <= 8'hFF;
			ddram_req  <= 1'b1;
			state      <= S_WAIT_DDRAM2;
		end

		S_WAIT_DDRAM2: begin
			if (ddram_ready) begin
				busy  <= 1'b0;
				state <= S_IDLE;
			end
		end

		// -------------------------------------------------------
		// ABORT: SDRAM timeout — cancel transfer, clear busy, return to IDLE.
		// The header will still have busy=1 from S_WRITE_HEADER, so we
		// re-write it with busy=0 and a special frame_counter=0 to signal
		// an incomplete transfer. The next VBlank will start a fresh transfer.
		// -------------------------------------------------------
		S_ABORT: begin
			ddram_addr <= ddram_base_addr;
			ddram_din  <= {16'd0, 8'h00, REGION_COUNT[7:0], 32'h52414348}; // busy=0
			ddram_be   <= 8'hFF;
			ddram_req  <= 1'b1;
			frame_counter <= frame_counter - 1'd1; // Roll back: this frame was incomplete
			state      <= S_WAIT_DDRAM2;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
