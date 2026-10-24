// ============================================================================
// dht11_temp.v
// XC4520 (DHT11) temperature subsystem for the DE0-Nano health monitor.
//
//   dht11_reader  - drives / decodes the sensor's single-wire protocol.
//   temp_uart_tx  - sends one line per reading to the Arduino:
//                       $T=36.0*72<CR><LF>
//                   (checksum = XOR of the characters between '$' and '*',
//                    as two hex digits; 36.0 C -> 0x72)
//
// The Arduino sketch already forwards every byte it receives from the FPGA
// to the PC, so no Arduino change is needed.
//
// DHT11 limits: whole degrees only (tenths byte is always 0), +/-2 C,
// 0-50 C range, at most one reading per second.
// ============================================================================


// ----------------------------------------------------------------------------
// dht11_reader
// ----------------------------------------------------------------------------
// Protocol (all times from the DHT11 datasheet):
//   FPGA pulls the line low >= 18 ms, releases it.
//   Sensor answers: low ~80 us, high ~80 us.
//   Then 40 bits, each: low ~50 us, then high 26-28 us (= 0) or ~70 us (= 1).
//   Bytes: humidity int, humidity dec, temp int, temp dec, checksum.
// ----------------------------------------------------------------------------
module dht11_reader #(
    parameter CLK_HZ   = 50_000_000,
    parameter PWRUP_MS = 1500,   // settle time after reset (sensor needs > 1 s)
    parameter POLL_MS  = 2000    // time between reads (datasheet minimum 1 s)
)(
    input  wire       clk,
    input  wire       rst_n,        // active-low async reset

    inout  wire       dht_data,     // sensor data pin (external pull-up needed)

    output reg  [7:0] temp_c,       // temperature, whole degrees C
    output reg  [3:0] temp_tenths,  // tenths digit (0 on a DHT11)
    output reg  [7:0] humidity,     // relative humidity, %
    output reg        data_valid,   // 1-clock pulse: new good reading
    output reg        read_ok       // level: 1 = last read succeeded
);

    localparam integer TICKS_PER_US  = CLK_HZ / 1_000_000;
    localparam integer PWRUP_US      = PWRUP_MS * 1000;
    localparam integer POLL_US       = POLL_MS  * 1000;
    localparam integer START_LOW_US  = 20_000;  // host start pulse (>= 18 ms)
    localparam integer TIMEOUT_US    = 150;     // max wait for any sensor edge
    localparam integer BIT_THRESH_US = 45;      // high longer than this = 1

    localparam [3:0]
        ST_PWRUP     = 4'd0,
        ST_START_LOW = 4'd1,
        ST_REL_HIGH  = 4'd2,   // line released, wait for pull-up to bring it high
        ST_RESP_WAIT = 4'd3,   // wait for sensor to pull low
        ST_RESP_LOW  = 4'd4,   // sensor low  (~80 us)
        ST_RESP_HIGH = 4'd5,   // sensor high (~80 us)
        ST_BIT_LOW   = 4'd6,   // start of each bit (~50 us low)
        ST_BIT_HIGH  = 4'd7,   // bit value: high pulse width
        ST_CHECK     = 4'd8,
        ST_WAIT      = 4'd9;

    reg [3:0]  state;
    reg [21:0] us_cnt;         // microseconds spent in the current state
    reg [7:0]  tick_cnt;
    reg        tick;           // 1 us tick
    reg [39:0] shift;
    reg [5:0]  bit_cnt;
    reg        dht_oe;         // 1 = drive the line low, 0 = release (Hi-Z)

    // Open-drain style line: only ever drive low, never high.
    assign dht_data = dht_oe ? 1'b0 : 1'bz;

    // Two-flop synchroniser for the asynchronous sensor line
    reg din_m, din_s;
    always @(posedge clk) begin
        din_m <= dht_data;
        din_s <= din_m;
    end

    // Frame fields and checks
    wire [7:0] hum_i = shift[39:32];
    wire [7:0] hum_d = shift[31:24];
    wire [7:0] tmp_i = shift[23:16];
    wire [7:0] tmp_d = shift[15:8];
    wire [7:0] csum  = shift[7:0];
    wire [7:0] sum   = hum_i + hum_d + tmp_i + tmp_d;
    wire       frame_ok = (sum == csum) && (hum_i <= 8'd100) && (tmp_i <= 8'd60);

    wire timeout = (us_cnt >= TIMEOUT_US);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_PWRUP;
            us_cnt      <= 22'd0;
            tick_cnt    <= 8'd0;
            tick        <= 1'b0;
            shift       <= 40'd0;
            bit_cnt     <= 6'd0;
            dht_oe      <= 1'b0;
            temp_c      <= 8'd0;
            temp_tenths <= 4'd0;
            humidity    <= 8'd0;
            data_valid  <= 1'b0;
            read_ok     <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            // 1 us tick and per-state microsecond counter
            if (tick_cnt == TICKS_PER_US - 1) begin
                tick_cnt <= 8'd0;
                tick     <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 1'b1;
                tick     <= 1'b0;
            end
            if (tick) us_cnt <= us_cnt + 1'b1;

            case (state)
                ST_PWRUP: begin
                    if (us_cnt >= PWRUP_US) begin
                        us_cnt <= 22'd0;
                        state  <= ST_START_LOW;
                    end
                end

                ST_START_LOW: begin
                    dht_oe <= 1'b1;                    // pull line low
                    if (us_cnt >= START_LOW_US) begin
                        dht_oe <= 1'b0;                // release
                        us_cnt <= 22'd0;
                        state  <= ST_REL_HIGH;
                    end
                end

                ST_REL_HIGH: begin
                    if (din_s) begin us_cnt <= 22'd0; state <= ST_RESP_WAIT; end
                end

                ST_RESP_WAIT: begin
                    if (!din_s) begin us_cnt <= 22'd0; state <= ST_RESP_LOW; end
                end

                ST_RESP_LOW: begin
                    if (din_s) begin us_cnt <= 22'd0; state <= ST_RESP_HIGH; end
                end

                ST_RESP_HIGH: begin
                    if (!din_s) begin
                        bit_cnt <= 6'd0;
                        us_cnt  <= 22'd0;
                        state   <= ST_BIT_LOW;
                    end
                end

                ST_BIT_LOW: begin
                    if (din_s) begin us_cnt <= 22'd0; state <= ST_BIT_HIGH; end
                end

                ST_BIT_HIGH: begin
                    if (!din_s) begin
                        // high pulse ended: long = 1, short = 0
                        shift  <= {shift[38:0], (us_cnt >= BIT_THRESH_US)};
                        us_cnt <= 22'd0;
                        if (bit_cnt == 6'd39) begin
                            state <= ST_CHECK;
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                            state   <= ST_BIT_LOW;
                        end
                    end
                end

                ST_CHECK: begin
                    if (frame_ok) begin
                        humidity    <= hum_i;
                        temp_c      <= tmp_i;
                        temp_tenths <= (tmp_d <= 8'd9) ? tmp_d[3:0] : 4'd0;
                        data_valid  <= 1'b1;
                        read_ok     <= 1'b1;
                    end else begin
                        read_ok     <= 1'b0;
                    end
                    us_cnt <= 22'd0;
                    state  <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (us_cnt >= POLL_US) begin
                        us_cnt <= 22'd0;
                        state  <= ST_START_LOW;
                    end
                end

                default: begin
                    us_cnt <= 22'd0;
                    state  <= ST_WAIT;
                end
            endcase

            // Any handshake state that waits too long (sensor missing, wire
            // fault) aborts this read and retries after POLL_MS.
            // Placed after the case so it overrides the assignments above.
            if (timeout && state >= ST_REL_HIGH && state <= ST_BIT_HIGH) begin
                us_cnt  <= 22'd0;
                state   <= ST_WAIT;
                read_ok <= 1'b0;
            end
        end
    end

endmodule


// ----------------------------------------------------------------------------
// temp_uart_tx
// ----------------------------------------------------------------------------
// Sends "$T=tt.t*CS\r\n" (12 bytes, ~12.5 ms at 9600 baud) once per good
// reading. The output shares the Arduino-facing UART line with serial_plot,
// so transmission is scheduled in a quiet window of the accelerometer stream:
//
//   slot_sync = the pulse that starts each serial_plot line (uart_data_valid,
//               every 100 ms). The temperature line starts GUARD_MS later,
//               after the accelerometer line has finished (it is < 25 ms
//               long) and ends well before the next one begins.
//
// Both transmitters idle high, so the two lines are simply AND-ed together
// at the top level.
//
// If slot_sync stops arriving (e.g. accelerometer fault) for STALL_MS, the
// line is free, so the temperature is sent anyway.
// ----------------------------------------------------------------------------
module temp_uart_tx #(
    parameter CLK_HZ   = 50_000_000,
    parameter BAUD     = 9600,
    parameter GUARD_MS = 40,
    parameter STALL_MS = 200
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] temp_c,        // whole degrees C (clamped to 99)
    input  wire [3:0] temp_tenths,   // 0-9
    input  wire       temp_valid,    // pulse: new reading available
    input  wire       slot_sync,     // pulse: start of an accelerometer line

    output reg        txd            // idles high
);

    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam integer GUARD_CLKS   = (CLK_HZ / 1000) * GUARD_MS;
    localparam integer STALL_CLKS   = (CLK_HZ / 1000) * STALL_MS;

    // ---------------- message formatting ----------------
    wire [7:0] t_clamped = (temp_c > 8'd99) ? 8'd99 : temp_c;
    wire [7:0] tens_w    = t_clamped / 8'd10;
    wire [7:0] ones_w    = t_clamped % 8'd10;

    reg  [7:0] c_tens, c_ones, c_dec;   // latched ASCII digits for this message
    reg  [3:0] idx;                     // byte index within the message
    reg  [7:0] msg_char;

    function [7:0] hex_ascii;
        input [3:0] n;
        begin
            hex_ascii = (n < 4'd10) ? (8'h30 + n) : (8'h37 + n);
        end
    endfunction

    // XOR of everything between '$' and '*':  T = <tens><ones> . <tenths>
    wire [7:0] cs = 8'h54 ^ 8'h3D ^ c_tens ^ c_ones ^ 8'h2E ^ c_dec;

    always @(*) begin
        case (idx)
            4'd0:    msg_char = "$";
            4'd1:    msg_char = "T";
            4'd2:    msg_char = "=";
            4'd3:    msg_char = c_tens;
            4'd4:    msg_char = c_ones;
            4'd5:    msg_char = ".";
            4'd6:    msg_char = c_dec;
            4'd7:    msg_char = "*";
            4'd8:    msg_char = hex_ascii(cs[7:4]);
            4'd9:    msg_char = hex_ascii(cs[3:0]);
            4'd10:   msg_char = 8'h0D;   // CR
            default: msg_char = 8'h0A;   // LF
        endcase
    end

    // ---------------- byte transmitter (8N1) ----------------
    reg        send_byte;       // pulse: start sending tx_byte
    reg  [7:0] tx_byte;
    reg        busy;
    reg        byte_done;       // pulse: stop bit finished
    reg [15:0] baud_cnt;
    reg  [3:0] bit_n;
    reg  [9:0] shreg;           // {stop, data[7:0], start}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txd       <= 1'b1;
            busy      <= 1'b0;
            byte_done <= 1'b0;
            baud_cnt  <= 16'd0;
            bit_n     <= 4'd0;
            shreg     <= 10'h3FF;
        end else begin
            byte_done <= 1'b0;
            if (!busy) begin
                if (send_byte) begin
                    shreg    <= {1'b1, tx_byte, 1'b0};
                    txd      <= 1'b0;                 // start bit
                    busy     <= 1'b1;
                    baud_cnt <= 16'd0;
                    bit_n    <= 4'd0;
                end
            end else if (baud_cnt == CLKS_PER_BIT - 1) begin
                baud_cnt <= 16'd0;
                if (bit_n == 4'd9) begin              // stop bit complete
                    busy      <= 1'b0;
                    byte_done <= 1'b1;
                end else begin
                    bit_n <= bit_n + 1'b1;
                    shreg <= {1'b1, shreg[9:1]};
                    txd   <= shreg[1];                // d0..d7, then stop
                end
            end else begin
                baud_cnt <= baud_cnt + 1'b1;
            end
        end
    end

    // ---------------- message scheduler ----------------
    localparam [1:0] M_IDLE = 2'd0, M_GUARD = 2'd1, M_LOAD = 2'd2, M_WAIT = 2'd3;

    reg [1:0]  mstate;
    reg        pending;
    reg [23:0] slot_age;
    reg [23:0] guard_cnt;
    wire       stalled = (slot_age == STALL_CLKS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstate    <= M_IDLE;
            pending   <= 1'b0;
            slot_age  <= 24'd0;
            guard_cnt <= 24'd0;
            idx       <= 4'd0;
            c_tens    <= "0";
            c_ones    <= "0";
            c_dec     <= "0";
            send_byte <= 1'b0;
            tx_byte   <= 8'hFF;
        end else begin
            send_byte <= 1'b0;

            if (temp_valid) pending <= 1'b1;

            // time since the last accelerometer line started
            if (slot_sync)                    slot_age <= 24'd0;
            else if (slot_age != STALL_CLKS)  slot_age <= slot_age + 1'b1;

            case (mstate)
                M_IDLE: begin
                    if (pending && (slot_sync || stalled)) begin
                        guard_cnt <= 24'd0;
                        mstate    <= M_GUARD;
                    end
                end

                M_GUARD: begin
                    if (guard_cnt == GUARD_CLKS - 1) begin
                        c_tens <= 8'h30 + tens_w[3:0];
                        c_ones <= 8'h30 + ones_w[3:0];
                        c_dec  <= 8'h30 + temp_tenths;
                        idx    <= 4'd0;
                        mstate <= M_LOAD;
                    end else begin
                        guard_cnt <= guard_cnt + 1'b1;
                    end
                end

                M_LOAD: begin
                    if (!busy) begin
                        tx_byte   <= msg_char;
                        send_byte <= 1'b1;
                        mstate    <= M_WAIT;
                    end
                end

                M_WAIT: begin
                    if (byte_done) begin
                        if (idx == 4'd11) begin
                            pending <= 1'b0;
                            mstate  <= M_IDLE;
                        end else begin
                            idx    <= idx + 1'b1;
                            mstate <= M_LOAD;
                        end
                    end
                end
            endcase
        end
    end

endmodule
