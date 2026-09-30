// bfp_simd_core.v v3 - flattened regfile, Yosys-safe

module bfp_simd_core #(
    parameter MANT_WIDTH = 16,
    parameter EXP_WIDTH  = 8,
    parameter LANES      = 4,
    parameter REGS       = 8,
    parameter RADDR      = 3
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          reg_we,
    input  wire [RADDR-1:0]              waddr,
    input  wire [RADDR-1:0]              raddr1,
    input  wire [RADDR-1:0]              raddr2,
    input  wire [2:0]                    op,
    input  wire [LANES-1:0]              lane_enable,
    input  wire [LANES*MANT_WIDTH-1:0]   imm_mant,
    input  wire [EXP_WIDTH-1:0]          imm_exp,
    output wire [LANES*MANT_WIDTH-1:0]   result_mant,
    output wire [EXP_WIDTH-1:0]          result_exp,
    output wire [EXP_WIDTH-1:0]          waddr_exp_o
);

    localparam OP_LOAD = 3'b000;
    localparam OP_ADD  = 3'b001;
    localparam OP_SUB  = 3'b010;
    localparam OP_MUL  = 3'b011;
    localparam OP_PASS = 3'b100;

    reg [MANT_WIDTH-1:0] regfile0 [0:REGS-1];
    reg [MANT_WIDTH-1:0] regfile1 [0:REGS-1];
    reg [MANT_WIDTH-1:0] regfile2 [0:REGS-1];
    reg [MANT_WIDTH-1:0] regfile3 [0:REGS-1];
    reg [EXP_WIDTH-1:0]  exp_tag [0:REGS-1];
    assign waddr_exp_o = exp_tag[waddr];

    reg [MANT_WIDTH-1:0] a_op [0:LANES-1];
    reg [MANT_WIDTH-1:0] b_op [0:LANES-1];
    always @(*) begin
        a_op[0] = regfile0[raddr1]; b_op[0] = regfile0[raddr2];
        a_op[1] = regfile1[raddr1]; b_op[1] = regfile1[raddr2];
        a_op[2] = regfile2[raddr1]; b_op[2] = regfile2[raddr2];
        a_op[3] = regfile3[raddr1]; b_op[3] = regfile3[raddr2];
    end
wire signed [EXP_WIDTH:0] exp1 = {exp_tag[raddr1][EXP_WIDTH-1], exp_tag[raddr1]};
    wire signed [EXP_WIDTH:0] exp2 = {exp_tag[raddr2][EXP_WIDTH-1], exp_tag[raddr2]};
    wire signed [EXP_WIDTH:0] exp_diff = exp1 - exp2;
    wire [5:0] align_shift_a = (exp_diff < 0) ? -exp_diff : 6'd0;
    wire [5:0] align_shift_b = (exp_diff > 0) ? exp_diff  : 6'd0;
    wire signed [EXP_WIDTH-1:0] common_exp = (exp_diff >= 0) ? exp1[EXP_WIDTH-1:0] : exp2[EXP_WIDTH-1:0];

    function [5:0] clz32;
        input [31:0] val;
        integer idx;
        begin
            clz32 = 32;
            for (idx = 31; idx >= 0; idx = idx - 1)
                if (val[idx] && clz32 == 32)
                    clz32 = 31 - idx;
        end
    endfunction

    function [5:0] req_shift_mul;
        input signed [31:0] prod;
        reg   [31:0] mag;
        integer lz;
        integer needed_bits;
        begin
            mag = prod[31] ? (~prod + 1'b1) : prod;
            if (mag == 0) begin
                req_shift_mul = 0;
            end else begin
                lz = clz32(mag);
                needed_bits = 32 - lz;
                if (needed_bits > (MANT_WIDTH - 1))
                    req_shift_mul = needed_bits - (MANT_WIDTH - 1);
                else
                    req_shift_mul = 0;
            end
        end
    endfunction

    reg signed [MANT_WIDTH-1:0] a_aligned [0:LANES-1];
    reg signed [MANT_WIDTH-1:0] b_aligned [0:LANES-1];
    reg signed [MANT_WIDTH:0]   raw_addsub [0:LANES-1];
    reg signed [2*MANT_WIDTH-1:0] raw_mul  [0:LANES-1];

    integer li;
    always @(*) begin
        for (li = 0; li < LANES; li = li + 1) begin
            a_aligned[li] = $signed(a_op[li]) >>> align_shift_a;
            b_aligned[li] = $signed(b_op[li]) >>> align_shift_b;
            raw_addsub[li] = (op == OP_SUB)
                              ? (a_aligned[li] - b_aligned[li])
                              : (a_aligned[li] + b_aligned[li]);
            raw_mul[li] = $signed(a_op[li]) * $signed(b_op[li]);
        end
    end

    reg overflow_any;
    reg [5:0] shift_addsub;
    reg [5:0] shift_mul_lane [0:LANES-1];
    reg [5:0] shift_mul_block;

    always @(*) begin
        overflow_any = 1'b0;
        for (li = 0; li < LANES; li = li + 1) begin
            if (lane_enable[li] &&
                (raw_addsub[li][MANT_WIDTH] != raw_addsub[li][MANT_WIDTH-1]))
                overflow_any = 1'b1;
        end
        shift_addsub = overflow_any ? 1 : 0;

        shift_mul_block = 0;
        for (li = 0; li < LANES; li = li + 1) begin
            shift_mul_lane[li] = lane_enable[li] ? req_shift_mul(raw_mul[li]) : 6'd0;
            if (shift_mul_lane[li] > shift_mul_block)
                shift_mul_block = shift_mul_lane[li];
        end
    end
reg [MANT_WIDTH-1:0] final_mant [0:LANES-1];
    reg [EXP_WIDTH-1:0]  final_exp;

    always @(*) begin
        case (op)
            OP_LOAD: begin
                for (li = 0; li < LANES; li = li + 1)
                    final_mant[li] = imm_mant[(li+1)*MANT_WIDTH-1 -: MANT_WIDTH];
                final_exp = imm_exp;
            end
            OP_ADD, OP_SUB: begin
                for (li = 0; li < LANES; li = li + 1)
                    final_mant[li] = (raw_addsub[li] >>> shift_addsub);
                final_exp = common_exp + shift_addsub;
            end
            OP_MUL: begin
                for (li = 0; li < LANES; li = li + 1)
                    final_mant[li] = (raw_mul[li] >>> shift_mul_block);
                final_exp = exp_tag[raddr1] + exp_tag[raddr2] + shift_mul_block;
            end
            default: begin
                for (li = 0; li < LANES; li = li + 1)
                    final_mant[li] = a_op[li];
                final_exp = exp_tag[raddr1];
            end
        endcase
    end

    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : pack
            assign result_mant[(g+1)*MANT_WIDTH-1 -: MANT_WIDTH] = final_mant[g];
        end
    endgenerate
    assign result_exp = final_exp;

    integer ri;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ri = 0; ri < REGS; ri = ri + 1) begin
                regfile0[ri] <= {MANT_WIDTH{1'b0}};
                regfile1[ri] <= {MANT_WIDTH{1'b0}};
                regfile2[ri] <= {MANT_WIDTH{1'b0}};
                regfile3[ri] <= {MANT_WIDTH{1'b0}};
            end
            for (ri = 0; ri < REGS; ri = ri + 1)
                exp_tag[ri] <= {EXP_WIDTH{1'b0}};
        end else if (reg_we) begin
            if (lane_enable[0]) regfile0[waddr] <= final_mant[0];
            if (lane_enable[1]) regfile1[waddr] <= final_mant[1];
            if (lane_enable[2]) regfile2[waddr] <= final_mant[2];
            if (lane_enable[3]) regfile3[waddr] <= final_mant[3];
            exp_tag[waddr] <= final_exp;
        end
    end

endmodule
