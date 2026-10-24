// ============================================================================
// uart_tx.v  --  8N1 UART transmitter
// ============================================================================
module uart_tx #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,      // pulse with data to send one byte
    input  wire [7:0] data,
    output reg        busy,
    output reg        txd
);
    localparam integer DIV = CLK_HZ / BAUD;
    reg [$clog2(DIV)-1:0] cnt;
    reg [3:0] bitn;
    reg [9:0] sh;                 // {stop, data[7:0], start}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txd <= 1'b1; busy <= 1'b0; cnt <= 0; bitn <= 0; sh <= 10'h3FF;
        end else if (!busy) begin
            txd <= 1'b1;
            if (start) begin
                sh   <= {1'b1, data, 1'b0};
                busy <= 1'b1; cnt <= 0; bitn <= 0;
            end
        end else begin
            txd <= sh[0];
            if (cnt == DIV-1) begin
                cnt <= 0;
                sh  <= {1'b1, sh[9:1]};
                if (bitn == 4'd9) busy <= 1'b0;
                else bitn <= bitn + 4'd1;
            end else
                cnt <= cnt + 1'b1;
        end
    end
endmodule
