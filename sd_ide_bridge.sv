/********************************************/
/* sd_ide_bridge.sv                         */
/* SD card to IDE/ATA bridge                */
/* For MiSTer Minimig-AGA secondary SD slot */
/*                                          */
/* Makes the secondary SD card appear as a  */
/* standard ATA hard disk to AmigaOS.       */
/* Plugs into the existing Minimig IDE bus. */
/********************************************/

module sd_ide_bridge
(
    input  wire        clk,         // System clock (28MHz)
    input  wire        reset,       // Active high reset

    // ---- IDE/ATA register interface (from Minimig/fastchip) ----
    input  wire  [4:0] ide_addr,    // ATA register address
    input  wire [15:0] ide_writedata, // Data from Amiga to drive
    output reg  [15:0] ide_readdata,  // Data from drive to Amiga
    input  wire        ide_rd,      // Read strobe
    input  wire        ide_wr,      // Write strobe
    output reg         ide_irq,     // Interrupt to Amiga (command complete)
    output reg   [5:0] ide_req,     // Request to HPS (we steal this for SD)

    // ---- Interface to sd_spi_controller ----
    output reg  [31:0] sd_block_addr,  // Block address to read/write
    output reg         sd_rd_req,      // Pulse to start a read
    output reg         sd_wr_req,      // Pulse to start a write
    output reg   [7:0] sd_wr_data,     // Write data byte to SPI controller
    input  wire  [7:0] sd_rd_data,     // Read data byte from SPI controller
    input  wire        sd_rd_valid,    // Read data byte is valid
    input  wire        sd_wr_req_next, // SPI controller wants next write byte
    input  wire        sd_busy,        // SPI controller is busy
    input  wire        sd_error,       // SPI controller error
    input  wire        sd_card_ready,  // SD card initialised OK

    // ---- Drive identity ----
    input  wire        drive_num       // 0=master, 1=slave
);

// ============================================================
// ATA Register addresses (ide_addr[4:0])
// ============================================================
localparam ATA_DATA       = 5'h00; // Data register (16-bit)
localparam ATA_ERROR      = 5'h01; // Error / Features
localparam ATA_SEC_COUNT  = 5'h02; // Sector count
localparam ATA_LBA_LO     = 5'h03; // LBA bits 0-7
localparam ATA_LBA_MID    = 5'h04; // LBA bits 8-15
localparam ATA_LBA_HI     = 5'h05; // LBA bits 16-23
localparam ATA_DRV_HEAD   = 5'h06; // Drive / Head / LBA bits 24-27
localparam ATA_STATUS     = 5'h07; // Status (read) / Command (write)

// ============================================================
// ATA Status register bits
// ============================================================
localparam ATA_ST_BSY  = 8'h80; // Busy
localparam ATA_ST_DRDY = 8'h40; // Drive ready
localparam ATA_ST_DRQ  = 8'h08; // Data request
localparam ATA_ST_ERR  = 8'h01; // Error

// ============================================================
// ATA Commands
// ============================================================
localparam CMD_IDENTIFY    = 8'hEC; // Identify drive
localparam CMD_READ_MULTI  = 8'hC4; // Read multiple
localparam CMD_WRITE_MULTI = 8'hC5; // Write multiple  
localparam CMD_READ_SECTOR = 8'h20; // Read sectors
localparam CMD_WRITE_SECTOR= 8'h30; // Write sectors
localparam CMD_SET_MULTI   = 8'hC6; // Set multiple mode
localparam CMD_INIT_PARAMS = 8'h91; // Initialize drive parameters
localparam CMD_STANDBY_IMM = 8'hE0; // Standby immediate

// ============================================================
// Drive geometry for Amiga compatibility
// We present the SD card as a CHS drive to AmigaOS
// but use LBA28 internally.
// 
// Max LBA28 = 268,435,455 sectors = ~128GB
// For a 32GB card: 32*1024*1024*1024/512 = 67,108,864 sectors
//
// CHS presented to Amiga: 
//   Cylinders = 65535, Heads = 16, Sectors = 63
//   (standard translation geometry - HDToolBox will handle it)
// ============================================================
localparam DRIVE_CYLS  = 16'd65535;
localparam DRIVE_HEADS = 8'd16;
localparam DRIVE_SECS  = 8'd63;

// Total sectors for 32GB = 67,108,864
// We let AmigaOS figure out the real size via IDENTIFY
localparam TOTAL_SECTORS = 32'd67108864; // 32GB default

// ============================================================
// Internal registers
// ============================================================
reg  [7:0] ata_error;
reg  [7:0] ata_sec_count;
reg  [7:0] ata_lba_lo;
reg  [7:0] ata_lba_mid;
reg  [7:0] ata_lba_hi;
reg  [7:0] ata_drv_head;
reg  [7:0] ata_status;
reg  [7:0] ata_command;

// LBA28 address assembled from registers
wire [27:0] lba28 = {ata_drv_head[3:0], ata_lba_hi, ata_lba_mid, ata_lba_lo};

// ============================================================
// 512-byte sector buffer (for IDENTIFY and data transfers)
// We use a simple register array
// ============================================================
reg  [15:0] sector_buf [0:255]; // 256 words = 512 bytes
reg  [8:0]  buf_ptr;            // Byte pointer into buffer
reg  [8:0]  buf_cnt;            // Bytes remaining in transfer

// ============================================================
// State machine
// ============================================================
localparam ST_IDLE          = 4'd0;
localparam ST_IDENTIFY      = 4'd1;  // Build IDENTIFY response
localparam ST_PIO_READ_SD   = 4'd2;  // Trigger SD read
localparam ST_PIO_READ_WAIT = 4'd3;  // Wait for SD data
localparam ST_PIO_READ_RDY  = 4'd4;  // Data in buffer, waiting for Amiga to read
localparam ST_PIO_WRITE_RDY = 4'd5;  // Waiting for Amiga to write sector
localparam ST_PIO_WRITE_SD  = 4'd6;  // Trigger SD write
localparam ST_PIO_WRITE_WAIT= 4'd7;  // Wait for SD write complete
localparam ST_COMPLETE      = 4'd8;  // Command complete, raise IRQ
localparam ST_ERROR         = 4'd9;

reg [3:0] state;
reg [8:0] id_word;     // Word counter for IDENTIFY data
reg [7:0] sec_remain;  // Sectors remaining in multi-sector transfer

// ============================================================
// IDENTIFY DEVICE response buffer builder
// We fill sector_buf with the standard ATA IDENTIFY response
// AmigaOS/HDToolBox reads this to understand the drive
// ============================================================
task build_identify;
    integer i;
    begin
        // Clear buffer first
        for(i = 0; i < 256; i = i+1) sector_buf[i] <= 16'h0000;

        // Word 0: General config - fixed drive, not removable
        sector_buf[0]   <= 16'h0040;
        // Word 1: Cylinders
        sector_buf[1]   <= DRIVE_CYLS;
        // Word 3: Heads
        sector_buf[3]   <= {8'h00, DRIVE_HEADS};
        // Word 6: Sectors per track
        sector_buf[6]   <= {8'h00, DRIVE_SECS};
        // Word 10-19: Serial number (ASCII, space padded)
        sector_buf[10]  <= 16'h4D49; // "MI"
        sector_buf[11]  <= 16'h5354; // "ST"
        sector_buf[12]  <= 16'h4552; // "ER"
        sector_buf[13]  <= 16'h5344; // "SD"
        sector_buf[14]  <= 16'h3220; // "2 "
        sector_buf[15]  <= 16'h2020; // "  "
        sector_buf[16]  <= 16'h2020;
        sector_buf[17]  <= 16'h2020;
        sector_buf[18]  <= 16'h2020;
        sector_buf[19]  <= 16'h2020;
        // Word 23-26: Firmware revision
        sector_buf[23]  <= 16'h312E; // "1."
        sector_buf[24]  <= 16'h3030; // "00"
        sector_buf[25]  <= 16'h2020;
        sector_buf[26]  <= 16'h2020;
        // Word 27-46: Model number
        sector_buf[27]  <= 16'h4D69; // "Mi"
        sector_buf[28]  <= 16'h6E69; // "ni"
        sector_buf[29]  <= 16'h6D69; // "mi"
        sector_buf[30]  <= 16'h6720; // "g "
        sector_buf[31]  <= 16'h5344; // "SD"
        sector_buf[32]  <= 16'h3220; // "2 "
        sector_buf[33]  <= 16'h4472; // "Dr"
        sector_buf[34]  <= 16'h6976; // "iv"
        sector_buf[35]  <= 16'h6520; // "e "
        sector_buf[36]  <= 16'h2020;
        sector_buf[37]  <= 16'h2020;
        sector_buf[38]  <= 16'h2020;
        sector_buf[39]  <= 16'h2020;
        sector_buf[40]  <= 16'h2020;
        sector_buf[41]  <= 16'h2020;
        sector_buf[42]  <= 16'h2020;
        sector_buf[43]  <= 16'h2020;
        sector_buf[44]  <= 16'h2020;
        sector_buf[45]  <= 16'h2020;
        sector_buf[46]  <= 16'h2020;
        // Word 47: Max sectors per interrupt (multi-sector)
        sector_buf[47]  <= 16'h8010; // 16 sectors max
        // Word 49: Capabilities - LBA supported
        sector_buf[49]  <= 16'h0200;
        // Word 51: PIO timing mode
        sector_buf[51]  <= 16'h0200;
        // Word 53: Fields valid
        sector_buf[53]  <= 16'h0001;
        // Word 54: Current cylinders
        sector_buf[54]  <= DRIVE_CYLS;
        // Word 55: Current heads
        sector_buf[55]  <= {8'h00, DRIVE_HEADS};
        // Word 56: Current sectors per track
        sector_buf[56]  <= {8'h00, DRIVE_SECS};
        // Word 57-58: Current capacity in sectors
        sector_buf[57]  <= TOTAL_SECTORS[15:0];
        sector_buf[58]  <= TOTAL_SECTORS[31:16];
        // Word 60-61: Total LBA sectors
        sector_buf[60]  <= TOTAL_SECTORS[15:0];
        sector_buf[61]  <= TOTAL_SECTORS[31:16];
    end
endtask

// ============================================================
// Write data - stream sector_buf to SPI controller
// ============================================================
reg [8:0] rd_byte_cnt;
reg       rd_hi_lo;
reg [8:0] wr_byte_cnt;
reg       wr_hi_lo;

always @(posedge clk) begin
    if(sd_wr_req_next && state == ST_PIO_WRITE_WAIT) begin
        if(!wr_hi_lo) begin
            sd_wr_data <= sector_buf[wr_byte_cnt[8:1]][15:8];
        end else begin
            sd_wr_data <= sector_buf[wr_byte_cnt[8:1]][7:0];
        end
        wr_byte_cnt <= wr_byte_cnt + 1'd1;
        wr_hi_lo    <= ~wr_hi_lo;
    end
    if(state == ST_PIO_WRITE_SD) begin
        wr_byte_cnt <= 0;
        wr_hi_lo    <= 0;
    end
end

// ============================================================
// Main ATA state machine
// ============================================================
always @(posedge clk) begin
    if(reset) begin
        state        <= ST_IDLE;
        ata_status   <= ATA_ST_DRDY; // Ready, not busy
        ata_error    <= 8'h00;
        ide_irq      <= 0;
        ide_req      <= 0;
        sd_rd_req    <= 0;
        sd_wr_req    <= 0;
        buf_ptr      <= 0;
        sec_remain   <= 0;
        rd_byte_cnt  <= 0;
        rd_hi_lo     <= 0;
    end
    else begin
        sd_rd_req <= 0;
        sd_wr_req <= 0;
        ide_irq   <= 0;

        // SD read data capture - fills sector_buf from SPI stream
        if(sd_rd_valid) begin
            if(!rd_hi_lo)
                sector_buf[rd_byte_cnt[8:1]][15:8] <= sd_rd_data;
            else
                sector_buf[rd_byte_cnt[8:1]][7:0]  <= sd_rd_data;
            rd_byte_cnt <= rd_byte_cnt + 1'd1;
            rd_hi_lo    <= ~rd_hi_lo;
        end

        case(state)

            // ---- IDLE: wait for ATA command ----
            ST_IDLE: begin
                if(sd_card_ready) begin
                    ata_status <= ATA_ST_DRDY;
                end else begin
                    ata_status <= ATA_ST_BSY;
                end
            end

            // ---- Build IDENTIFY response in buffer ----
            ST_IDENTIFY: begin
                build_identify();
                buf_ptr    <= 0;
                ata_status <= ATA_ST_DRDY | ATA_ST_DRQ;
                state      <= ST_PIO_READ_RDY;
            end

            // ---- Trigger SD block read ----
            ST_PIO_READ_SD: begin
                ata_status   <= ATA_ST_BSY;
                sd_block_addr<= {4'b0000, lba28} + (ata_sec_count - sec_remain);
                sd_rd_req    <= 1;
                rd_byte_cnt  <= 0;
                rd_hi_lo     <= 0;
                state        <= ST_PIO_READ_WAIT;
            end

            // ---- Wait for SD read to complete ----
            ST_PIO_READ_WAIT: begin
                if(!sd_busy && !sd_rd_req) begin
                    if(sd_error) begin
                        state <= ST_ERROR;
                    end else begin
                        buf_ptr    <= 0;
                        ata_status <= ATA_ST_DRDY | ATA_ST_DRQ;
                        state      <= ST_PIO_READ_RDY;
                    end
                end
            end

            // ---- Data ready in buffer, Amiga reading words ----
            ST_PIO_READ_RDY: begin
                // Nothing to do here, reads handled in register read section
            end

            // ---- Waiting for Amiga to write a full sector ----
            ST_PIO_WRITE_RDY: begin
                // Nothing to do here, writes handled in register write section
            end

            // ---- Trigger SD block write ----
            ST_PIO_WRITE_SD: begin
                ata_status    <= ATA_ST_BSY;
                sd_block_addr <= {4'b0000, lba28} + (ata_sec_count - sec_remain);
                sd_wr_req     <= 1;
                wr_byte_cnt   <= 0;
                wr_hi_lo      <= 0;
                state         <= ST_PIO_WRITE_WAIT;
            end

            // ---- Wait for SD write to complete ----
            ST_PIO_WRITE_WAIT: begin
                if(!sd_busy && !sd_wr_req) begin
                    if(sd_error) begin
                        state <= ST_ERROR;
                    end else begin
                        sec_remain <= sec_remain - 1'd1;
                        if(sec_remain == 1) begin
                            state <= ST_COMPLETE;
                        end else begin
                            buf_ptr    <= 0;
                            ata_status <= ATA_ST_DRDY | ATA_ST_DRQ;
                            state      <= ST_PIO_WRITE_RDY;
                        end
                    end
                end
            end

            // ---- Command complete ----
            ST_COMPLETE: begin
                ata_status <= ATA_ST_DRDY;
                ide_irq    <= 1;
                state      <= ST_IDLE;
            end

            // ---- Error ----
            ST_ERROR: begin
                ata_error  <= 8'h04; // Abort
                ata_status <= ATA_ST_DRDY | ATA_ST_ERR;
                ide_irq    <= 1;
                state      <= ST_IDLE;
            end

        endcase

        // ============================================================
        // ATA Register WRITES from Amiga
        // ============================================================
        if(ide_wr) begin
            case(ide_addr)
                ATA_DATA: begin
                    // Amiga writing a word into our sector buffer
                    if(state == ST_PIO_WRITE_RDY) begin
                        sector_buf[buf_ptr[8:1]] <= ide_writedata;
                        buf_ptr <= buf_ptr + 2'd2;
                        if(buf_ptr == 510) begin
                            // Full sector received, write to SD
                            sec_remain <= sec_remain - 1'd1;
                            state      <= ST_PIO_WRITE_SD;
                        end
                    end
                end
                ATA_ERROR:     ; // Features register write - ignore for now
                ATA_SEC_COUNT: ata_sec_count <= ide_writedata[7:0];
                ATA_LBA_LO:    ata_lba_lo    <= ide_writedata[7:0];
                ATA_LBA_MID:   ata_lba_mid   <= ide_writedata[7:0];
                ATA_LBA_HI:    ata_lba_hi    <= ide_writedata[7:0];
                ATA_DRV_HEAD:  ata_drv_head  <= ide_writedata[7:0];

                ATA_STATUS: begin
                    // Command register write - execute command
                    ata_command <= ide_writedata[7:0];
                    case(ide_writedata[7:0])

                        CMD_IDENTIFY: begin
                            state <= ST_IDENTIFY;
                        end

                        CMD_READ_SECTOR,
                        CMD_READ_MULTI: begin
                            sec_remain <= ata_sec_count ? ata_sec_count : 8'd1;
                            state      <= ST_PIO_READ_SD;
                        end

                        CMD_WRITE_SECTOR,
                        CMD_WRITE_MULTI: begin
                            sec_remain <= ata_sec_count ? ata_sec_count : 8'd1;
                            buf_ptr    <= 0;
                            ata_status <= ATA_ST_DRDY | ATA_ST_DRQ;
                            state      <= ST_PIO_WRITE_RDY;
                        end

                        CMD_INIT_PARAMS,
                        CMD_SET_MULTI,
                        CMD_STANDBY_IMM: begin
                            // Accept but do nothing - just acknowledge
                            state <= ST_COMPLETE;
                        end

                        default: begin
                            // Unknown command - return error
                            state <= ST_ERROR;
                        end
                    endcase
                end
            endcase
        end

        // ============================================================
        // ATA Register READS from Amiga
        // ============================================================
        if(ide_rd) begin
            case(ide_addr)
                ATA_DATA: begin
                    if(state == ST_PIO_READ_RDY) begin
                        ide_readdata <= sector_buf[buf_ptr[8:1]];
                        buf_ptr <= buf_ptr + 2'd2;
                        if(buf_ptr == 510) begin
                            // Full sector sent to Amiga
                            sec_remain <= sec_remain - 1'd1;
                            if(sec_remain == 1) begin
                                state <= ST_COMPLETE;
                            end else begin
                                // More sectors to read
                                state <= ST_PIO_READ_SD;
                            end
                        end
                    end else begin
                        ide_readdata <= 16'hFFFF;
                    end
                end
                ATA_ERROR:    ide_readdata <= {8'h00, ata_error};
                ATA_SEC_COUNT:ide_readdata <= {8'h00, ata_sec_count};
                ATA_LBA_LO:   ide_readdata <= {8'h00, ata_lba_lo};
                ATA_LBA_MID:  ide_readdata <= {8'h00, ata_lba_mid};
                ATA_LBA_HI:   ide_readdata <= {8'h00, ata_lba_hi};
                ATA_DRV_HEAD: ide_readdata <= {8'h00, ata_drv_head};
                ATA_STATUS:   ide_readdata <= {8'h00, ata_status};
                default:      ide_readdata <= 16'hFFFF;
            endcase
        end

    end
end

endmodule
