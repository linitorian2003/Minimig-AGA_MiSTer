/********************************************/
/* sd_spi_controller.sv                     */
/* Secondary SD card SPI controller         */
/* For MiSTer Minimig-AGA secondary SD slot */
/* Handles card init, block read/write      */
/********************************************/

module sd_spi_controller
(
    input  wire        clk,          // System clock (28MHz)
    input  wire        reset,        // Active high reset

    // SPI physical interface (connects to SD_* pins in Minimig.sv)
    output reg         sd_sck,       // SPI clock to SD card
    output reg         sd_mosi,      // Master Out Slave In
    input  wire        sd_miso,      // Master In Slave Out
    output reg         sd_cs,        // Chip select (active low)

    // Block transfer interface (used by sd_ide_bridge)
    input  wire [31:0] block_addr,   // 32-bit block (sector) address
    input  wire        rd_req,       // Pulse high to request a block read
    input  wire        wr_req,       // Pulse high to request a block write
    input  wire  [7:0] wr_data,      // Write data byte
    output reg   [7:0] rd_data,      // Read data byte
    output reg         rd_data_valid,// High for one clock when rd_data is valid
    output reg         wr_data_req,  // High when controller needs next write byte
    output reg         busy,         // High while any operation is in progress
    output reg         error,        // High if last operation failed
    output reg         card_ready    // High once card is initialised and ready
);

// ============================================================
// SPI clock divider
// We use a slow clock (~400kHz) for init, faster (~4MHz) after
// 28MHz / 70 = 400kHz  (init)
// 28MHz /  7 = 4MHz    (data transfer - safe for all SD cards)
// ============================================================
reg [6:0] clk_div;
reg       fast_mode;
wire      spi_clk_en = fast_mode ? (clk_div == 7'd3) : (clk_div == 7'd34);

always @(posedge clk) begin
    if(reset) begin
        clk_div <= 0;
    end else begin
        clk_div <= clk_div + 1'd1;
        if(fast_mode && clk_div >= 7'd6)  clk_div <= 0;
        if(!fast_mode && clk_div >= 7'd69) clk_div <= 0;
    end
end

// ============================================================
// Main state machine
// ============================================================
localparam ST_RESET        = 5'd0;
localparam ST_INIT_CLK     = 5'd1;  // Send 80 dummy clocks with CS high
localparam ST_CMD0         = 5'd2;  // GO_IDLE_STATE
localparam ST_CMD0_WAIT    = 5'd3;
localparam ST_CMD8         = 5'd4;  // SEND_IF_COND (detect SDHC)
localparam ST_CMD8_WAIT    = 5'd5;
localparam ST_ACMD41       = 5'd6;  // APP_SEND_OP_COND
localparam ST_ACMD41_WAIT  = 5'd7;
localparam ST_CMD58        = 5'd8;  // READ_OCR (confirm SDHC)
localparam ST_CMD58_WAIT   = 5'd9;
localparam ST_READY        = 5'd10; // Card ready, idle
localparam ST_READ_START   = 5'd11; // CMD17 - read single block
localparam ST_READ_WAIT    = 5'd12; // Wait for data token 0xFE
localparam ST_READ_DATA    = 5'd13; // Clock in 512 bytes
localparam ST_READ_CRC     = 5'd14; // Clock in 2 CRC bytes (ignored)
localparam ST_WRITE_START  = 5'd15; // CMD24 - write single block
localparam ST_WRITE_TOKEN  = 5'd16; // Send data token 0xFE
localparam ST_WRITE_DATA   = 5'd17; // Clock out 512 bytes
localparam ST_WRITE_CRC    = 5'd18; // Send dummy CRC
localparam ST_WRITE_RESP   = 5'd19; // Wait for write response token
localparam ST_WRITE_BUSY   = 5'd20; // Wait while card is busy writing
localparam ST_ERROR        = 5'd21;

reg [4:0]  state;
reg [4:0]  return_state;   // Where to go after sending a command
reg [6:0]  init_clk_cnt;   // Counter for 80 init clocks
reg [8:0]  byte_cnt;       // Byte counter for 512-byte transfers
reg [3:0]  bit_cnt;        // Bit counter within a byte (0-7, up to 15 for CRC)
reg [7:0]  shift_out;      // Byte being shifted out (MOSI)
reg [7:0]  shift_in;       // Byte being shifted in  (MISO)
reg [47:0] cmd_out;        // 48-bit SPI command
reg [5:0]  cmd_bits;       // Bits remaining in command
reg        sdhc;           // 1 = SDHC/SDXC card (uses block addressing)
reg [7:0]  resp_byte;      // Response byte from card
reg [7:0]  timeout;        // Timeout counter
reg [7:0]  wr_byte;        // Latched write byte
reg [8:0]  wr_cnt;         // Write byte counter

// ============================================================
// Helper: build a standard 6-byte SPI command
// cmd[5:0], arg[31:0], crc[6:0] + stop bit
// ============================================================
function [47:0] make_cmd;
    input [5:0]  cmd;
    input [31:0] arg;
    input [6:0]  crc;
    begin
        make_cmd = {2'b01, cmd, arg, crc, 1'b1};
    end
endfunction

// ============================================================
// Main state machine - runs on every spi_clk_en tick
// ============================================================
always @(posedge clk) begin
    if(reset) begin
        state        <= ST_RESET;
        sd_cs        <= 1;
        sd_sck       <= 0;
        sd_mosi      <= 1;
        busy         <= 0;
        error        <= 0;
        card_ready   <= 0;
        rd_data_valid<= 0;
        wr_data_req  <= 0;
        fast_mode    <= 0;
        sdhc         <= 0;
    end
    else if(spi_clk_en) begin
        rd_data_valid <= 0;
        wr_data_req   <= 0;
        sd_sck        <= ~sd_sck; // Toggle SPI clock

        // ---- Actions on rising edge of SPI clock ----
        if(sd_sck) begin  // This is the falling edge of sd_sck (we just toggled)
            // Sample MISO on rising edge
            shift_in <= {shift_in[6:0], sd_miso};
        end
        else begin        // This is the rising edge - shift out MOSI
            case(state)

                // ---- Reset: deassert CS, prepare ----
                ST_RESET: begin
                    sd_cs       <= 1;
                    sd_mosi     <= 1;
                    fast_mode   <= 0;
                    busy        <= 1;
                    card_ready  <= 0;
                    error       <= 0;
                    init_clk_cnt<= 0;
                    state       <= ST_INIT_CLK;
                end

                // ---- Send 80 dummy clocks with CS=1 ----
                ST_INIT_CLK: begin
                    sd_mosi <= 1;
                    init_clk_cnt <= init_clk_cnt + 1'd1;
                    if(init_clk_cnt == 79) begin
                        sd_cs   <= 0; // Assert CS
                        cmd_out <= make_cmd(0, 32'h00000000, 7'h4A); // CMD0
                        cmd_bits<= 47;
                        state   <= ST_CMD0;
                    end
                end

                // ---- Send CMD0 (GO_IDLE_STATE) ----
                ST_CMD0: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        timeout <= 255;
                        state   <= ST_CMD0_WAIT;
                    end
                end

                // ---- Wait for R1 response = 0x01 (idle) ----
                ST_CMD0_WAIT: begin
                    sd_mosi <= 1;
                    if(!sd_miso) begin // Start bit received
                        // Collect remaining 7 bits
                        if(shift_in[6:0] == 7'b0000000) begin // R1 = 0x01
                            // Send CMD8
                            cmd_out  <= make_cmd(8, 32'h000001AA, 7'h43);
                            cmd_bits <= 47;
                            state    <= ST_CMD8;
                        end else begin
                            state <= ST_ERROR;
                        end
                    end else begin
                        timeout <= timeout - 1'd1;
                        if(timeout == 0) state <= ST_ERROR;
                    end
                end

                // ---- Send CMD8 (SEND_IF_COND) ----
                ST_CMD8: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        bit_cnt <= 0;
                        timeout <= 255;
                        state   <= ST_CMD8_WAIT;
                    end
                end

                // ---- Wait for CMD8 R7 response ----
                ST_CMD8_WAIT: begin
                    sd_mosi <= 1;
                    bit_cnt <= bit_cnt + 1'd1;
                    if(bit_cnt == 39) begin // R7 = 5 bytes
                        // If card responded with 0x01 and echoed 0xAA, it's v2
                        // If it didn't respond, it's v1 SD - still try ACMD41
                        cmd_out  <= make_cmd(55, 32'h00000000, 7'h00); // CMD55
                        cmd_bits <= 47;
                        state    <= ST_ACMD41;
                    end
                end

                // ---- Send CMD55 + ACMD41 ----
                ST_ACMD41: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        timeout <= 255;
                        state   <= ST_ACMD41_WAIT;
                    end
                end

                ST_ACMD41_WAIT: begin
                    sd_mosi <= 1;
                    if(!sd_miso) begin
                        if(shift_in[6:0] == 7'b0000000) begin
                            // Still idle (0x01), send CMD55+ACMD41 again
                            cmd_out  <= make_cmd(55, 32'h00000000, 7'h00);
                            cmd_bits <= 47;
                            state    <= ST_ACMD41;
                        end else begin
                            // Ready (0x00)! Check for SDHC via CMD58
                            cmd_out  <= make_cmd(58, 32'h00000000, 7'h00);
                            cmd_bits <= 47;
                            state    <= ST_CMD58;
                        end
                    end else begin
                        timeout <= timeout - 1'd1;
                        if(timeout == 0) state <= ST_ERROR;
                    end
                end

                // ---- CMD58: Read OCR to check SDHC bit ----
                ST_CMD58: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        bit_cnt <= 0;
                        state   <= ST_CMD58_WAIT;
                    end
                end

                ST_CMD58_WAIT: begin
                    sd_mosi <= 1;
                    bit_cnt <= bit_cnt + 1'd1;
                    if(bit_cnt == 8)  resp_byte <= shift_in; // OCR byte 1
                    if(bit_cnt == 39) begin
                        sdhc      <= resp_byte[6]; // CCS bit = SDHC/SDXC
                        fast_mode <= 1;            // Switch to fast SPI clock
                        busy      <= 0;
                        card_ready<= 1;
                        sd_cs     <= 1;
                        state     <= ST_READY;
                    end
                end

                // ---- READY: wait for rd_req or wr_req ----
                ST_READY: begin
                    sd_mosi <= 1;
                    error   <= 0;
                    if(rd_req) begin
                        busy    <= 1;
                        sd_cs   <= 0;
                        // CMD17: READ_SINGLE_BLOCK
                        // SDHC uses block address, SDSC uses byte address
                        cmd_out  <= make_cmd(17, sdhc ? block_addr : {block_addr[22:0], 9'b0}, 7'h00);
                        cmd_bits <= 47;
                        state    <= ST_READ_START;
                    end
                    else if(wr_req) begin
                        busy     <= 1;
                        sd_cs    <= 0;
                        cmd_out  <= make_cmd(24, sdhc ? block_addr : {block_addr[22:0], 9'b0}, 7'h00);
                        cmd_bits <= 47;
                        state    <= ST_WRITE_START;
                    end
                end

                // ---- READ: send CMD17 ----
                ST_READ_START: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        timeout <= 255;
                        state   <= ST_READ_WAIT;
                    end
                end

                // ---- READ: wait for data token 0xFE ----
                ST_READ_WAIT: begin
                    sd_mosi <= 1;
                    if(shift_in == 8'hFE) begin
                        byte_cnt <= 0;
                        bit_cnt  <= 7;
                        state    <= ST_READ_DATA;
                    end else begin
                        timeout <= timeout - 1'd1;
                        if(timeout == 0) state <= ST_ERROR;
                    end
                end

                // ---- READ: clock in 512 data bytes ----
                ST_READ_DATA: begin
                    sd_mosi <= 1;
                    bit_cnt <= bit_cnt - 1'd1;
                    if(bit_cnt == 0) begin
                        rd_data       <= shift_in;
                        rd_data_valid <= 1;
                        byte_cnt      <= byte_cnt + 1'd1;
                        bit_cnt       <= 7;
                        if(byte_cnt == 511) begin
                            state <= ST_READ_CRC;
                            bit_cnt <= 15; // 2 CRC bytes = 16 bits
                        end
                    end
                end

                // ---- READ: skip 2 CRC bytes ----
                ST_READ_CRC: begin
                    sd_mosi <= 1;
                    bit_cnt <= bit_cnt - 1'd1;
                    if(bit_cnt == 0) begin
                        sd_cs  <= 1;
                        busy   <= 0;
                        state  <= ST_READY;
                    end
                end

                // ---- WRITE: send CMD24 ----
                ST_WRITE_START: begin
                    sd_mosi  <= cmd_out[47];
                    cmd_out  <= {cmd_out[46:0], 1'b1};
                    cmd_bits <= cmd_bits - 1'd1;
                    if(cmd_bits == 0) begin
                        timeout <= 255;
                        state   <= ST_WRITE_TOKEN;
                    end
                end

                // ---- WRITE: send 1 byte gap + data token 0xFE ----
                ST_WRITE_TOKEN: begin
                    // Wait for R1 response then send token
                    if(!sd_miso) begin
                        shift_out <= 8'hFE; // Data token
                        bit_cnt   <= 7;
                        wr_cnt    <= 0;
                        wr_data_req <= 1; // Request first byte
                        state     <= ST_WRITE_DATA;
                    end else begin
                        sd_mosi <= 1;
                        timeout <= timeout - 1'd1;
                        if(timeout == 0) state <= ST_ERROR;
                    end
                end

                // ---- WRITE: send 512 data bytes ----
                ST_WRITE_DATA: begin
                    sd_mosi   <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b1};
                    bit_cnt   <= bit_cnt - 1'd1;
                    if(bit_cnt == 0) begin
                        wr_cnt <= wr_cnt + 1'd1;
                        if(wr_cnt == 511) begin
                            // All data sent, send dummy CRC
                            shift_out <= 8'hFF;
                            bit_cnt   <= 7;
                            state     <= ST_WRITE_CRC;
                        end else begin
                            shift_out   <= wr_data; // Latch next byte
                            wr_data_req <= 1;       // Request next byte
                            bit_cnt     <= 7;
                        end
                    end
                end

                // ---- WRITE: send 2 dummy CRC bytes ----
                ST_WRITE_CRC: begin
                    sd_mosi   <= 1;
                    bit_cnt   <= bit_cnt - 1'd1;
                    if(bit_cnt == 0) begin
                        timeout <= 255;
                        state   <= ST_WRITE_RESP;
                    end
                end

                // ---- WRITE: read data response token ----
                ST_WRITE_RESP: begin
                    sd_mosi <= 1;
                    if(!sd_miso) begin
                        // Response token: xxx0sss1
                        // sss = 010 means accepted
                        if(shift_in[3:1] == 3'b010) begin
                            timeout <= 255;
                            state   <= ST_WRITE_BUSY;
                        end else begin
                            state <= ST_ERROR;
                        end
                    end else begin
                        timeout <= timeout - 1'd1;
                        if(timeout == 0) state <= ST_ERROR;
                    end
                end

                // ---- WRITE: wait while card is busy (MISO=0) ----
                ST_WRITE_BUSY: begin
                    sd_mosi <= 1;
                    if(sd_miso) begin // Card releases MISO when done
                        sd_cs  <= 1;
                        busy   <= 0;
                        state  <= ST_READY;
                    end
                end

                // ---- ERROR state ----
                ST_ERROR: begin
                    sd_cs  <= 1;
                    sd_mosi<= 1;
                    busy   <= 0;
                    error  <= 1;
                    state  <= ST_READY; // Allow retry
                end

            endcase
        end
    end
end

endmodule
