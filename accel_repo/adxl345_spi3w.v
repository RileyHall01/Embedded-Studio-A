// ============================================================================
// adxl345_spi3w.v
// 3-wire SPI master (mode 3: CPOL=1, CPHA=1) for the ADXL345 on the DE0-Nano.
// The DE0-Nano shares one bidirectional data line (I2C_SDAT) between MOSI and
// MISO, so the ADXL345 must be configured for 3-wire SPI (DATA_FORMAT bit 6).
//
// One "transaction" = 1 command byte + up to 6 data bytes (write or read).
// SCLK is derived from clk by CLK_DIV (50 MHz / 25 = 2 MHz, ADXL345 max 5 MHz).
// ============================================================================
module adxl345_spi3w #(
    parameter CLK_DIV = 25          // clk cycles per SCLK half-period
)(
    input  wire        clk,
    input  wire        rst_n,

    // request interface
    input  wire        start,        // pulse to begin a transaction
    input  wire        wr,           // 1 = write, 0 = read
    input  wire [5:0]  addr,         // register address
    input  wire [7:0]  wdata,        // data byte for writes
    input  wire [2:0]  nbytes,       // data bytes to read (1..6), ignored for write
    output reg  [47:0] rdata,        // read bytes, first byte in [47:40]
    output reg         busy,
    output reg         done,         // 1-cycle pulse when transaction completes

    // ADXL345 pins
    output reg         spi_csn,
    output reg         spi_sclk,
    inout  wire        spi_sdio
);

    // SDIO tri-state: drive during command byte and write data, release for reads
    reg sdio_oe;
    reg sdio_out;
    assign spi_sdio = sdio_oe ? sdio_out : 1'bz;

    localparam ST_IDLE = 3'd0,
               ST_CMD  = 3'd1,
               ST_DATA = 3'd2,
               ST_END  = 3'd3;

    reg [2:0]  state;
    reg [7:0]  shreg;          // shift register (command byte / write data)
    reg [2:0]  bitcnt;         // bit within current byte
    reg [2:0]  bytecnt;        // data bytes remaining
    reg [7:0]  clkcnt;         // SCLK divider
    reg        rd_mode;
    reg        phase;          // 0 = about to fall (shift), 1 = about to rise (sample)

    wire [7:0] cmd = {~wr, (nbytes > 3'd1), addr};   // R/W, MB, A5..A0

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            spi_csn  <= 1'b1;
            spi_sclk <= 1'b1;
            sdio_oe  <= 1'b0;
            sdio_out <= 1'b0;
            busy     <= 1'b0;
            done     <= 1'b0;
            rdata    <= 48'd0;
            clkcnt   <= 8'd0;
            bitcnt   <= 3'd0;
            bytecnt  <= 3'd0;
            rd_mode  <= 1'b0;
            phase    <= 1'b0;
            shreg    <= 8'd0;
        end else begin
            done <= 1'b0;

            case (state)
            // ----------------------------------------------------------------
            ST_IDLE: begin
                spi_csn  <= 1'b1;
                spi_sclk <= 1'b1;
                sdio_oe  <= 1'b0;
                busy     <= 1'b0;
                if (start) begin
                    busy    <= 1'b1;
                    spi_csn <= 1'b0;
                    shreg   <= cmd;
                    rd_mode <= ~wr;
                    bytecnt <= wr ? 3'd1 : nbytes;
                    bitcnt  <= 3'd7;
                    clkcnt  <= 8'd0;
                    phase   <= 1'b0;
                    state   <= ST_CMD;
                end
            end
            // ----------------------------------------------------------------
            // Shift the command byte out (always driven by master)
            ST_CMD: begin
                sdio_oe <= 1'b1;
                if (clkcnt == CLK_DIV-1) begin
                    clkcnt <= 8'd0;
                    if (phase == 1'b0) begin
                        // falling edge: present next bit
                        spi_sclk <= 1'b0;
                        sdio_out <= shreg[7];
                        phase    <= 1'b1;
                    end else begin
                        // rising edge: slave samples
                        spi_sclk <= 1'b1;
                        shreg    <= {shreg[6:0], 1'b0};
                        phase    <= 1'b0;
                        if (bitcnt == 3'd0) begin
                            bitcnt <= 3'd7;
                            if (rd_mode) sdio_oe <= 1'b0;   // release bus for slave
                            else         shreg   <= wdata;  // load write data
                            state <= ST_DATA;
                        end else
                            bitcnt <= bitcnt - 3'd1;
                    end
                end else
                    clkcnt <= clkcnt + 8'd1;
            end
            // ----------------------------------------------------------------
            // Data bytes: write -> keep driving; read -> sample on rising edge
            ST_DATA: begin
                if (clkcnt == CLK_DIV-1) begin
                    clkcnt <= 8'd0;
                    if (phase == 1'b0) begin
                        spi_sclk <= 1'b0;
                        if (!rd_mode) sdio_out <= shreg[7];
                        phase <= 1'b1;
                    end else begin
                        spi_sclk <= 1'b1;
                        if (rd_mode) rdata <= {rdata[46:0], spi_sdio};
                        else         shreg <= {shreg[6:0], 1'b0};
                        phase <= 1'b0;
                        if (bitcnt == 3'd0) begin
                            bitcnt <= 3'd7;
                            if (bytecnt == 3'd1)
                                state <= ST_END;
                            else
                                bytecnt <= bytecnt - 3'd1;
                        end else
                            bitcnt <= bitcnt - 3'd1;
                    end
                end else
                    clkcnt <= clkcnt + 8'd1;
            end
            // ----------------------------------------------------------------
            ST_END: begin
                if (clkcnt == CLK_DIV-1) begin
                    spi_csn <= 1'b1;
                    sdio_oe <= 1'b0;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state   <= ST_IDLE;
                    clkcnt  <= 8'd0;
                end else
                    clkcnt <= clkcnt + 8'd1;
            end
            default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
