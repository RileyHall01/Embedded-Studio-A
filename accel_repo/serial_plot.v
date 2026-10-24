// ============================================================================
// serial_plot.v
// Streams "x,y,z,m\n" as signed ASCII decimals over UART every data_valid.
// m = 0 or MOVING_LEVEL (scaled so the flag is visible on the same axes).
// Compatible with the Arduino IDE Serial Plotter and the included plot.py.
//
// Max line: "-32768,-32768,-32768,200\n" = 25 bytes, 26 ms at 9600 baud.
// The top level sends every 10th sample (10 lines/s) so this keeps up.
// ============================================================================
module serial_plot #(
    parameter CLK_HZ       = 50_000_000,
    parameter BAUD         = 9600,
    parameter MOVING_LEVEL = 16'd1000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        data_valid,
    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,
    input  wire        moving,
    output wire        txd
);
    // ---------------------------------------------------------------- UART
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .busy(tx_busy), .txd(txd)
    );

    // ---------------------------------------------------------- formatter
    reg signed [15:0] vals [0:3];
    reg [1:0]  vidx;          // which of the 4 values
    reg [15:0] mag;           // magnitude being converted
    reg [2:0]  didx;          // digit position 0..4 (10000..1)
    reg [3:0]  digit;
    reg        nonzero;       // a non-zero digit has been sent (leading-zero suppression)
    reg        pending;       // new sample arrived while busy

    function [15:0] pow10(input [2:0] i);
        case (i)
            3'd0: pow10 = 16'd10000;
            3'd1: pow10 = 16'd1000;
            3'd2: pow10 = 16'd100;
            3'd3: pow10 = 16'd10;
            default: pow10 = 16'd1;
        endcase
    endfunction

    localparam S_IDLE  = 3'd0,
               S_SIGN  = 3'd1,
               S_DIV   = 3'd2,   // subtract pow10 repeatedly
               S_EMIT  = 3'd3,   // send digit (or skip leading zero)
               S_SEP   = 3'd4,   // ',' or '\n'
               S_WAIT  = 3'd5;   // wait for UART, then continue to `next`

    reg [2:0] state, next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; next <= S_IDLE; tx_start <= 1'b0; tx_data <= 8'd0;
            vidx <= 0; mag <= 0; didx <= 0; digit <= 0; nonzero <= 0; pending <= 0;
            vals[0] <= 0; vals[1] <= 0; vals[2] <= 0; vals[3] <= 0;
        end else begin
            tx_start <= 1'b0;
            if (data_valid) pending <= 1'b1;

            case (state)
            S_IDLE: if (pending || data_valid) begin
                pending <= 1'b0;
                vals[0] <= accel_x;
                vals[1] <= accel_y;
                vals[2] <= accel_z;
                vals[3] <= moving ? $signed(MOVING_LEVEL) : 16'sd0;
                vidx  <= 2'd0;
                state <= S_SIGN;
            end

            // send '-' if negative, take magnitude
            S_SIGN: begin
                didx    <= 3'd0;
                nonzero <= 1'b0;
                digit   <= 4'd0;
                if (vals[vidx] < 0) begin
                    mag     <= -vals[vidx];
                    tx_data <= "-"; tx_start <= 1'b1;
                    next <= S_DIV; state <= S_WAIT;
                end else begin
                    mag   <= vals[vidx];
                    state <= S_DIV;
                end
            end

            // count how many times pow10(didx) fits
            S_DIV: begin
                if (mag >= pow10(didx)) begin
                    mag   <= mag - pow10(didx);
                    digit <= digit + 4'd1;
                end else
                    state <= S_EMIT;
            end

            S_EMIT: begin
                if (digit != 0 || nonzero || didx == 3'd4) begin
                    tx_data <= "0" + {4'd0, digit};
                    tx_start <= 1'b1;
                    nonzero <= 1'b1;
                    next  <= (didx == 3'd4) ? S_SEP : S_DIV;
                    state <= S_WAIT;
                end else
                    state <= S_DIV;         // skip leading zero
                digit <= 4'd0;
                didx  <= didx + 3'd1;
            end

            S_SEP: begin
                tx_data  <= (vidx == 2'd3) ? "\n" : ",";
                tx_start <= 1'b1;
                if (vidx == 2'd3) next <= S_IDLE;
                else begin vidx <= vidx + 2'd1; next <= S_SIGN; end
                state <= S_WAIT;
            end

            // one cycle for start pulse to register, then wait for busy to drop
            S_WAIT: if (!tx_start && !tx_busy) state <= next;

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
