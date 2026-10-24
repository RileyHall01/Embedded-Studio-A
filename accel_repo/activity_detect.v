// ============================================================================
// activity_detect.v
// Stationary / moving classification (learning-contract spec #8).
//
// Method:
//   1. Track a slow baseline of each axis with a first-order IIR low-pass
//      (baseline += (sample - baseline) >>> BASE_SHIFT). The baseline absorbs
//      gravity and board orientation, so no calibration is needed.
//   2. Activity metric = |x - bx| + |y - by| + |z - bz| (Manhattan magnitude
//      of the high-pass residual -- cheap, no multipliers).
//   3. Compare against THRESHOLD, then debounce:
//        - ON_COUNT consecutive over-threshold samples  -> moving = 1
//        - metric below threshold for HOLD_SAMPLES      -> moving = 0
//      The hold time stops the flag chattering between steps.
//
// At full-res +/-2g the ADXL345 gives ~256 LSB/g, so THRESHOLD = 32
// corresponds to ~0.125 g of combined deviation. Tune on real hardware.
// ============================================================================
module activity_detect #(
    parameter THRESHOLD    = 16'd32,   // LSBs of combined deviation
    parameter ON_COUNT     = 4'd3,     // consecutive hits to declare movement
    parameter HOLD_SAMPLES = 8'd100,   // quiet samples to declare stationary (1 s @ 100 Hz)
    parameter BASE_SHIFT   = 5         // baseline IIR: alpha = 2^-BASE_SHIFT
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               data_valid,
    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,

    output reg                moving,        // 1 = movement, 0 = stationary
    output reg  [15:0]        activity_level // current metric, handy for debug/GUI
);

    // Baselines kept with BASE_SHIFT fractional bits for IIR precision
    reg signed [15+BASE_SHIFT:0] base_x, base_y, base_z;
    reg init_done;

    wire signed [15:0] bx = base_x >>> BASE_SHIFT;
    wire signed [15:0] by = base_y >>> BASE_SHIFT;
    wire signed [15:0] bz = base_z >>> BASE_SHIFT;

    wire signed [16:0] dx = accel_x - bx;
    wire signed [16:0] dy = accel_y - by;
    wire signed [16:0] dz = accel_z - bz;

    wire [16:0] adx = dx[16] ? -dx : dx;
    wire [16:0] ady = dy[16] ? -dy : dy;
    wire [16:0] adz = dz[16] ? -dz : dz;

    wire [18:0] metric = adx + ady + adz;

    reg [3:0] on_cnt;
    reg [7:0] hold_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_x <= 0; base_y <= 0; base_z <= 0;
            init_done      <= 1'b0;
            moving         <= 1'b0;
            activity_level <= 16'd0;
            on_cnt         <= 4'd0;
            hold_cnt       <= 8'd0;
        end else if (data_valid) begin
            if (!init_done) begin
                // seed baselines with the first sample
                base_x <= accel_x <<< BASE_SHIFT;
                base_y <= accel_y <<< BASE_SHIFT;
                base_z <= accel_z <<< BASE_SHIFT;
                init_done <= 1'b1;
            end else begin
                // slow baseline tracking
                base_x <= base_x + dx;
                base_y <= base_y + dy;
                base_z <= base_z + dz;

                activity_level <= (metric > 19'd65535) ? 16'hFFFF : metric[15:0];

                if (metric > {3'b000, THRESHOLD}) begin
                    hold_cnt <= HOLD_SAMPLES;
                    if (on_cnt == ON_COUNT) moving <= 1'b1;
                    else                    on_cnt <= on_cnt + 4'd1;
                end else begin
                    on_cnt <= 4'd0;
                    if (hold_cnt != 8'd0) hold_cnt <= hold_cnt - 8'd1;
                    else                  moving   <= 1'b0;
                end
            end
        end
    end
endmodule
