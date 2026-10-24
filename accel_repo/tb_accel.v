// ============================================================================
// tb_accel.v
// Simulation: behavioural 3-wire-SPI ADXL345 model + activity detector check.
// Phase 1 feeds constant readings (stationary), phase 2 injects vibration.
// Run with ModelSim/Questa (bundled with Quartus) or iverilog:
//   iverilog -g2005 -o tb adxl345_spi3w.v adxl345_ctrl.v activity_detect.v tb_accel.v && vvp tb
// ============================================================================
`timescale 1ns/1ps

module tb_accel;
    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;                 // 50 MHz

    wire csn, sclk;
    wire sdio;

    wire signed [15:0] ax, ay, az;
    wire dv, moving;
    wire [15:0] level;

    // DUT: controller with a fast sample rate so simulation stays short
    adxl345_ctrl #(.CLK_HZ(50_000_000), .SAMPLE_HZ(10_000)) u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .accel_x(ax), .accel_y(ay), .accel_z(az), .data_valid(dv),
        .spi_csn(csn), .spi_sclk(sclk), .spi_sdio(sdio)
    );

    activity_detect #(.THRESHOLD(16'd32), .ON_COUNT(4'd3), .HOLD_SAMPLES(8'd20)) u_act (
        .clk(clk), .rst_n(rst_n), .data_valid(dv),
        .accel_x(ax), .accel_y(ay), .accel_z(az),
        .moving(moving), .activity_level(level)
    );

    // ------------------------------------------------ behavioural ADXL345
    reg signed [15:0] mdl_x = 16'sd0,
                      mdl_y = 16'sd0,
                      mdl_z = 16'sd256;    // ~1 g on Z
    reg vibrate = 0;
    integer seed = 1;

    always @(negedge csn)
        if (vibrate) begin
            mdl_x = 16'sd0   + ($random(seed) % 60);
            mdl_y = 16'sd0   + ($random(seed) % 60);
            mdl_z = 16'sd256 + ($random(seed) % 60);
        end

    function [7:0] regread(input [5:0] ra);
        case (ra)
            6'h32: regread = mdl_x[7:0];
            6'h33: regread = mdl_x[15:8];
            6'h34: regread = mdl_y[7:0];
            6'h35: regread = mdl_y[15:8];
            6'h36: regread = mdl_z[7:0];
            6'h37: regread = mdl_z[15:8];
            default: regread = 8'h00;
        endcase
    endfunction

    // Edge-driven 3-wire SPI slave (mode 3)
    reg [7:0] shin;
    reg [3:0] cbits;      // command bits received
    reg       in_cmd, rd, mb;
    reg [5:0] a;
    reg [7:0] tx;
    reg [3:0] bidx;
    reg       drive = 0;
    reg       txbit;
    assign sdio = drive ? txbit : 1'bz;

    always @(negedge csn) begin
        in_cmd <= 1'b1;
        cbits  <= 4'd0;
        drive  <= 1'b0;
    end

    always @(posedge csn) begin
        drive  <= 1'b0;
        in_cmd <= 1'b0;
    end

    // sample MOSI / advance state on rising edges
    always @(posedge sclk) if (!csn) begin
        if (in_cmd) begin
            shin  <= {shin[6:0], sdio};
            cbits <= cbits + 4'd1;
            if (cbits == 4'd7) begin
                in_cmd <= 1'b0;
                rd     <= shin[6];               // bit7 of command
                mb     <= shin[5];               // bit6
                a      <= {shin[4:0], sdio};     // A5..A0
                tx     <= regread({shin[4:0], sdio});
                bidx   <= 4'd7;
            end
        end else if (rd) begin
            if (bidx == 4'd0) begin
                bidx <= 4'd7;
                if (mb) begin
                    a  <= a + 6'd1;
                    tx <= regread(a + 6'd1);
                end
            end else
                bidx <= bidx - 4'd1;
        end
        // writes: data byte ignored (config regs not modelled)
    end

    // drive MISO on falling edges during read data bytes
    always @(negedge sclk) if (!csn && !in_cmd && rd) begin
        drive <= 1'b1;
        txbit <= tx[bidx];
    end

    // ------------------------------------------------------------- stimulus
    integer i;
    integer fails = 0;
    initial begin
        #100 rst_n = 1;

        // stationary phase
        for (i = 0; i < 40; i = i + 1) @(posedge dv);
        if (moving !== 1'b0) begin $display("FAIL: moving asserted while stationary (x=%0d y=%0d z=%0d lvl=%0d)", ax, ay, az, level); fails = fails + 1; end
        else                 $display("PASS: stationary detected");

        // vibration phase
        vibrate = 1;
        for (i = 0; i < 40; i = i + 1) @(posedge dv);
        if (moving !== 1'b1) begin $display("FAIL: movement not detected"); fails = fails + 1; end
        else                 $display("PASS: movement detected (level=%0d)", level);

        // settle phase
        vibrate = 0;
        for (i = 0; i < 60; i = i + 1) @(posedge dv);
        if (moving !== 1'b0) begin $display("FAIL: did not return to stationary"); fails = fails + 1; end
        else                 $display("PASS: returned to stationary");

        if (fails == 0) $display("ALL TESTS PASSED");
        $finish;
    end
endmodule
