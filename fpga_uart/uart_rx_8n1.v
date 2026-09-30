// uart_rx_8n1.v — 8N1 UART receiver, oversampled at the full 12MHz system
// clock (not the divided 9600Hz clock) so it can reliably find the middle
// of each bit rather than bit-banging blindly like a free-running TX can.
// CLKS_PER_BIT = 1250 for 12MHz / 9600 baud, matching VSD's own verified
// divisor (625 half-period toggle -> 9600Hz, i.e. 1250 clocks per full bit).

module uart_rx_8n1 #(
    parameter CLKS_PER_BIT = 1250
)(
    input  wire       clk,       // full-speed system clock (12MHz)
    input  wire       rx,        // serial input pin
    output reg        rx_dv,     // one-clock pulse when a byte is ready
    output reg [7:0]  rx_byte
);

    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3, CLEANUP = 4;

    reg [2:0]  state = IDLE;
    reg [15:0] clk_count = 0;
    reg [2:0]  bit_index = 0;
    reg [7:0]  rx_shift = 0;
    reg        rx_d1 = 1, rx_d2 = 1; // 2-FF synchronizer for the async input

    always @(posedge clk) begin
        rx_d1 <= rx;
        rx_d2 <= rx_d1;
    end

    always @(posedge clk) begin
        rx_dv <= 1'b0;
        case (state)
            IDLE: begin
                clk_count <= 0;
                bit_index <= 0;
                if (rx_d2 == 1'b0) state <= START; // falling edge = start bit
            end
            START: begin
                // sample at the middle of the start bit to confirm it's real
                if (clk_count == (CLKS_PER_BIT-1)/2) begin
                    if (rx_d2 == 1'b0) begin
                        clk_count <= 0;
                        state <= DATA;
                    end else begin
                        state <= IDLE; // false start (glitch)
                    end
                end else begin
                    clk_count <= clk_count + 1;
                end
            end
            DATA: begin
                if (clk_count < CLKS_PER_BIT-1) begin
                    clk_count <= clk_count + 1;
                end else begin
                    clk_count <= 0;
                    rx_shift[bit_index] <= rx_d2;
                    if (bit_index < 7) begin
                        bit_index <= bit_index + 1;
                    end else begin
                        bit_index <= 0;
                        state <= STOP;
                    end
                end
            end
            STOP: begin
                if (clk_count < CLKS_PER_BIT-1) begin
                    clk_count <= clk_count + 1;
                end else begin
                    rx_dv   <= 1'b1;
                    rx_byte <= rx_shift;
                    clk_count <= 0;
                    state <= CLEANUP;
                end
            end
            CLEANUP: state <= IDLE;
            default: state <= IDLE;
        endcase
    end

endmodule
