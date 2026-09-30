// top.v — VSDSquadron FM (iCE40UP5K) UART-driven wrapper for bfp_simd_core
//
// Command protocol, 14 bytes, sent LSB-first per field:
//   byte0     op          (0=LOAD,1=ADD,2=SUB,3=MUL,4=PASS)
//   byte1     waddr       (0-7)
//   byte2     raddr1      (0-7)
//   byte3     raddr2      (0-7)
//   byte4     lane_enable (bit0..3 = lane 0..3)
//   byte5     imm_exp     (signed 8-bit)
//   byte6-7   imm_mant lane0 (low byte, high byte)
//   byte8-9   imm_mant lane1
//   byte10-11 imm_mant lane2
//   byte12-13 imm_mant lane3
//
// Response, 9 bytes, sent after the operation completes:
//   byte0-1  result_mant lane0
//   byte2-3  result_mant lane1
//   byte4-5  result_mant lane2
//   byte6-7  result_mant lane3
//   byte8    result_exp

`include "uart_rx_8n1.v"

module top (
    output wire led_red,
    output wire led_blue,
    output wire led_green,
    output wire uarttx,
    input  wire uartrx,
    input  wire hw_clk
);

    wire int_osc;
    SB_HFOSC #(.CLKHF_DIV("0b11")) u_SB_HFOSC (
        .CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc)
    );
    // int_osc = 12MHz, matching VSD's own verified reference designs

    // 9600 baud bit-clock, same divisor VSD's own tx example uses
    reg clk_9600 = 0;
    reg [31:0] cntr_9600 = 0;
    localparam period_9600 = 312;
    always @(posedge int_osc) begin
        cntr_9600 <= cntr_9600 + 1;
        if (cntr_9600 == period_9600) begin
            clk_9600 <= ~clk_9600;
            cntr_9600 <= 0;
        end
    end

    // ---- UART RX: oversampled directly off the 12MHz clock ----
    wire       rx_dv;
    wire [7:0] rx_byte;
    uart_rx_8n1 #(.CLKS_PER_BIT(625)) u_rx (
        .clk(int_osc), .rx(uartrx), .rx_dv(rx_dv), .rx_byte(rx_byte)
    );

    // ---- UART TX: bit-banged at the 9600Hz bit clock, VSD's own pattern ----
    reg  [7:0] tx_byte = 0;
    reg        tx_send = 0;
    reg        txbit = 1;
    assign uarttx = txbit;
    reg  [3:0] tx_state = 0;
    reg  [7:0] tx_shift = 0;
    reg  [3:0] tx_bitcnt = 0;
    localparam TX_IDLE=0, TX_START=1, TX_DATA=2, TX_STOP=3;
    reg tx_done_r = 0;
    always @(posedge clk_9600) begin
        tx_done_r <= 1'b0;
        case (tx_state)
            TX_IDLE: begin
                txbit <= 1'b1;
                if (tx_send) begin
                    tx_shift <= tx_byte;
                    tx_state <= TX_START;
                end
            end
            TX_START: begin
                txbit <= 1'b0;
                tx_bitcnt <= 0;
                tx_state <= TX_DATA;
            end
            TX_DATA: begin
                txbit <= tx_shift[0];
                tx_shift <= tx_shift >> 1;
                if (tx_bitcnt == 7) tx_state <= TX_STOP;
                else tx_bitcnt <= tx_bitcnt + 1;
            end
            TX_STOP: begin
                txbit <= 1'b1;
                tx_done_r <= 1'b1;
                tx_state <= TX_IDLE;
            end
        endcase
    end

    // ---- synchronize tx_done_r (slow 9600Hz domain) into int_osc domain
    // as a clean single-cycle pulse, rather than sampling the raw level
    // (which stays high for a long stretch and would otherwise race the
    // fast control FSM through multiple false "done" detections) ----
    reg tx_done_s0 = 0, tx_done_s1 = 0, tx_done_s2 = 0;
    always @(posedge int_osc) begin
        tx_done_s0 <= tx_done_r;
        tx_done_s1 <= tx_done_s0;
        tx_done_s2 <= tx_done_s1;
    end
    wire tx_done_pulse = tx_done_s1 & ~tx_done_s2;

    // ---- command byte assembly ----
    reg [7:0] cmd_bytes [0:13];
    reg [3:0] cmd_idx = 0;
    reg       cmd_ready = 0;

    always @(posedge int_osc) begin
        cmd_ready <= 1'b0;
        if (rx_dv) begin
            cmd_bytes[cmd_idx] <= rx_byte;
            if (cmd_idx == 13) begin
                cmd_idx <= 0;
                cmd_ready <= 1'b1;
            end else begin
                cmd_idx <= cmd_idx + 1;
            end
        end
    end

    // ---- bfp_simd_core instance ----
    localparam MANT_WIDTH = 16;
    localparam EXP_WIDTH  = 8;
    localparam LANES      = 4;
    localparam RADDR      = 3;

    reg                          rst_n = 0;
    reg                          reg_we = 0;
    reg  [RADDR-1:0]             waddr = 0, raddr1 = 0, raddr2 = 0;
    reg  [2:0]                   op = 0;
    reg  [LANES-1:0]             lane_enable = {LANES{1'b1}};
    reg  [LANES*MANT_WIDTH-1:0]  imm_mant = 0;
    reg  [EXP_WIDTH-1:0]         imm_exp = 0;
    wire [LANES*MANT_WIDTH-1:0]  result_mant;
    wire [EXP_WIDTH-1:0]         result_exp;
    wire [EXP_WIDTH-1:0]         waddr_exp_o;

    bfp_simd_core dut (
        .clk(int_osc), .rst_n(rst_n), .reg_we(reg_we),
        .waddr(waddr), .raddr1(raddr1), .raddr2(raddr2),
        .op(op), .lane_enable(lane_enable),
        .imm_mant(imm_mant), .imm_exp(imm_exp),
        .result_mant(result_mant), .result_exp(result_exp), .waddr_exp_o(waddr_exp_o)
    );

    // ---- response FSM: execute op, then stream out 9 response bytes ----
    localparam S_IDLE=0, S_EXEC=1, S_SEND=2, S_WAIT=3, S_NEXT=4;
    reg [3:0] rstate = S_IDLE;
    reg [3:0] resp_idx = 0;
    reg [7:0] resp_bytes [0:8];

    always @(posedge int_osc) begin
        rst_n <= 1'b1;
        reg_we <= 1'b0;
        case (rstate)
            S_IDLE: begin
                if (cmd_ready) begin
                    op          <= cmd_bytes[0][2:0];
                    waddr       <= cmd_bytes[1][2:0];
                    raddr1      <= cmd_bytes[2][2:0];
                    raddr2      <= cmd_bytes[3][2:0];
                    lane_enable <= cmd_bytes[4][3:0];
                    imm_exp     <= cmd_bytes[5];
                    imm_mant    <= { cmd_bytes[13], cmd_bytes[12],
                                      cmd_bytes[11], cmd_bytes[10],
                                      cmd_bytes[9],  cmd_bytes[8],
                                      cmd_bytes[7],  cmd_bytes[6] };
                    reg_we <= 1'b1;
                    rstate <= S_EXEC;
                end
            end
            S_EXEC: begin
                resp_bytes[0] <= result_mant[7:0];
                resp_bytes[1] <= result_mant[15:8];
                resp_bytes[2] <= result_mant[23:16];
                resp_bytes[3] <= result_mant[31:24];
                resp_bytes[4] <= result_mant[39:32];
                resp_bytes[5] <= result_mant[47:40];
                resp_bytes[6] <= result_mant[55:48];
                resp_bytes[7] <= result_mant[63:56];
                resp_bytes[8] <= result_exp;
                resp_idx <= 0;
                rstate <= S_SEND;
            end
            S_SEND: begin
                if (!tx_send) begin
                    tx_byte <= resp_bytes[resp_idx];
                    tx_send <= 1'b1;
                    rstate <= S_WAIT;
                end
            end
            S_WAIT: begin
                if (tx_done_pulse) begin
                    tx_send <= 1'b0;
                    rstate <= S_NEXT;
                end
            end
            S_NEXT: begin
                if (resp_idx == 8) rstate <= S_IDLE;
                else begin
                    resp_idx <= resp_idx + 1;
                    rstate <= S_SEND;
                end
            end
        endcase
    end

    // status LED: red while idle waiting, green while processing a command
    assign led_red   = (rstate == S_IDLE);
    assign led_green = (rstate != S_IDLE);
    assign led_blue  = 1'b0;

endmodule
