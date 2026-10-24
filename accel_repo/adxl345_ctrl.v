// ============================================================================
// adxl345_ctrl.v
// Configures the ADXL345 after power-up, then reads DATAX0..DATAZ1 (6 bytes,
// one multi-byte burst) at SAMPLE_HZ. Outputs signed 16-bit X/Y/Z samples
// with a data_valid strobe.
//
// Init sequence:
//   DATA_FORMAT (0x31) = 0x48  -> 3-wire SPI, full-res, +/-2g
//   BW_RATE     (0x2C) = 0x0A  -> 100 Hz output data rate
//   POWER_CTL   (0x2D) = 0x08  -> measurement mode
// ============================================================================
module adxl345_ctrl #(
    parameter CLK_HZ    = 50_000_000,
    parameter SAMPLE_HZ = 100
)(
    input  wire        clk,
    input  wire        rst_n,

    output reg  signed [15:0] accel_x,
    output reg  signed [15:0] accel_y,
    output reg  signed [15:0] accel_z,
    output reg         data_valid,      // 1-cycle pulse per new sample

    // ADXL345 pins
    output wire        spi_csn,
    output wire        spi_sclk,
    inout  wire        spi_sdio
);

    // ------------------------------------------------------------------ SPI
    reg         start, wr;
    reg  [5:0]  addr;
    reg  [7:0]  wdata;
    reg  [2:0]  nbytes;
    wire [47:0] rdata;
    wire        busy, done;

    adxl345_spi3w #(.CLK_DIV(25)) u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(start), .wr(wr), .addr(addr), .wdata(wdata),
        .nbytes(nbytes), .rdata(rdata), .busy(busy), .done(done),
        .spi_csn(spi_csn), .spi_sclk(spi_sclk), .spi_sdio(spi_sdio)
    );

    // ------------------------------------------------------- sample timer
    localparam integer SAMPLE_DIV = CLK_HZ / SAMPLE_HZ;
    reg [$clog2(SAMPLE_DIV)-1:0] tick_cnt;
    reg sample_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt    <= 0;
            sample_tick <= 1'b0;
        end else if (tick_cnt == SAMPLE_DIV-1) begin
            tick_cnt    <= 0;
            sample_tick <= 1'b1;
        end else begin
            tick_cnt    <= tick_cnt + 1'b1;
            sample_tick <= 1'b0;
        end
    end

    // ---------------------------------------------------------------- FSM
    localparam integer PWRUP_CYC = CLK_HZ / 100;      // ~10 ms power-up wait

    localparam ST_PWRUP   = 3'd0,
               ST_FORMAT  = 3'd1,
               ST_RATE    = 3'd2,
               ST_MEASURE = 3'd3,
               ST_WAIT    = 3'd4,
               ST_READ    = 3'd5;

    reg [2:0]  state;
    reg [$clog2(PWRUP_CYC)-1:0] pwr_cnt;

    // helper: issue one write and advance when done
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_PWRUP;
            pwr_cnt    <= 0;
            start      <= 1'b0;
            wr         <= 1'b0;
            addr       <= 6'd0;
            wdata      <= 8'd0;
            nbytes     <= 3'd1;
            accel_x    <= 16'sd0;
            accel_y    <= 16'sd0;
            accel_z    <= 16'sd0;
            data_valid <= 1'b0;
        end else begin
            start      <= 1'b0;
            data_valid <= 1'b0;

            case (state)
            ST_PWRUP: begin
                if (pwr_cnt == PWRUP_CYC-1) begin
                    // DATA_FORMAT = 3-wire SPI | FULL_RES | +/-2g
                    wr <= 1'b1; addr <= 6'h31; wdata <= 8'h48;
                    start <= 1'b1; state <= ST_FORMAT;
                end else
                    pwr_cnt <= pwr_cnt + 1'b1;
            end

            ST_FORMAT: if (done) begin
                wr <= 1'b1; addr <= 6'h2C; wdata <= 8'h0A;   // 100 Hz
                start <= 1'b1; state <= ST_RATE;
            end

            ST_RATE: if (done) begin
                wr <= 1'b1; addr <= 6'h2D; wdata <= 8'h08;   // measure
                start <= 1'b1; state <= ST_MEASURE;
            end

            ST_MEASURE: if (done)
                state <= ST_WAIT;

            ST_WAIT: if (sample_tick && !busy) begin
                wr <= 1'b0; addr <= 6'h32; nbytes <= 3'd6;   // burst DATAX0..DATAZ1
                start <= 1'b1; state <= ST_READ;
            end

            ST_READ: if (done) begin
                // bytes arrive in register order: X0 X1 Y0 Y1 Z0 Z1
                // first byte read sits in rdata[47:40]
                accel_x    <= {rdata[39:32], rdata[47:40]};  // {X1, X0}
                accel_y    <= {rdata[23:16], rdata[31:24]};  // {Y1, Y0}
                accel_z    <= {rdata[ 7: 0], rdata[15: 8]};  // {Z1, Z0}
                data_valid <= 1'b1;
                state      <= ST_WAIT;
            end

            default: state <= ST_WAIT;
            endcase
        end
    end
endmodule
