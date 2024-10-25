module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters
    // ======== choose your FSM style ==========
    // 1. FSM based on operation cycles
    parameter S_IDLE           = 3'd0;
    parameter S_ONE_CYCLE_OP   = 3'd1;
    parameter S_MULTI_CYCLE_OP = 3'd2;
    parameter S_BUFFER         = 3'd3;
    // 2. FSM based on operation modes
    // parameter S_IDLE = 4'd0;
    // parameter S_ADD  = 4'd1;
    // parameter S_SUB  = 4'd2;
    // parameter S_AND  = 4'd3;
    // parameter S_OR   = 4'd4;
    // parameter S_SLT  = 4'd5;
    // parameter S_SLL  = 4'd6;
    // parameter S_MUL  = 4'd7;
    // parameter S_DIV  = 4'd8;
    // parameter S_OUT  = 4'd9;

// Wires & Regs
    // Todo
    // state
    reg  [         2: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;
    

// Wire Assignments
    // Todo

    reg [4:0] Counter;
    reg [2*DATA_W - 1 : 0] output_data;
    reg [64:0] output_data2;
    assign o_data = (i_inst == 3'd6) ? output_data2[63:0] : output_data;
    reg output_valid;
    assign o_done = output_valid;

    wire [31:0] add_result;
    assign add_result = operand_a + operand_b;
    wire [31:0] sub_result;
    assign sub_result = operand_a - operand_b;
    wire [31:0] shift_ans = operand_a<<(operand_b);
    wire [63:0] div_temp;
    assign div_temp = (output_data + (operand_a << 1));
    wire [64:0] div_temp2;
    assign div_temp2 = (div_temp - (operand_b << 32));
    wire [64:0] div_temp3 = (output_data - (operand_b << 32)); 
    wire [64:0] div_temp4 = ((output_data - (operand_b << 32)) << 1) + 1; 
    wire [64:0] div_temp5 = (output_data << 1);

// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE           : state_nxt = (i_valid) ? S_BUFFER : S_IDLE;
            S_BUFFER         : state_nxt = ((i_inst[2] && i_inst[1]) ? S_MULTI_CYCLE_OP : S_ONE_CYCLE_OP);
            S_ONE_CYCLE_OP   : state_nxt = S_IDLE;
            S_MULTI_CYCLE_OP : state_nxt = (Counter == 5'd31)? S_IDLE : S_MULTI_CYCLE_OP;
            default : state_nxt = state;
        endcase
    end

    // Todo: Counter

    always @(negedge i_rst_n or posedge i_clk) begin
        if(!i_rst_n) Counter <= 5'd0;
        else if(state_nxt == S_BUFFER && Counter == 5'd31) Counter <= 5'd0;
        else if(state_nxt == S_MULTI_CYCLE_OP) Counter <= Counter + 5'd1;
        else Counter <= Counter;
    end

    // Todo: ALU output
    
    always @(posedge i_clk) begin
        if(i_valid) begin
            output_data <= 64'd0;
            output_data2 <= 65'd0;
        end
        else if(state_nxt == S_ONE_CYCLE_OP) begin
            case (i_inst)
                3'd0: begin // add
                    if(!operand_a[31] && !operand_b[31]) begin // pos + pos
                        output_data <= (add_result[31])? 64'h7FFF_FFFF : add_result;
                    end
                    else if (operand_a[31] && operand_b[31]) begin // neg + neg
                        output_data <= (!add_result[31])? 64'h8000_0000 : add_result;
                    end
                    else output_data <= add_result;
                end
                3'd1: begin // sub
                    if(!operand_a[31] && operand_b[31]) begin // pos - neg
                        output_data <= (sub_result[31])? 64'h7FFF_FFFF : sub_result;
                    end
                    else if (operand_a[31] && !operand_b[31]) begin // neg - pos
                        output_data <= (!sub_result[31])? 64'h8000_0000 : sub_result;
                    end
                    else output_data <= sub_result;
                end
                3'd2: begin // and
                    output_data <= operand_a & operand_b;
                end
                3'd3: begin // or
                    output_data <= operand_a | operand_b;
                end
                3'd4: begin // slt
                    if (!operand_a[31] && !operand_b[31]) begin // pos vs pos
                        output_data <= (operand_a < operand_b)? 64'd1 : 64'd0;
                    end
                    else if (operand_a[31] && operand_b[31]) begin // neg vs neg
                        output_data <= (operand_a < operand_b)? 64'd1 : 64'd0;
                    end
                    else if (!operand_a[31] && operand_b[31]) begin // pos vs neg
                        output_data <= 64'd0;
                    end // neg vs pos
                    else output_data <= 64'd1;
                end
                3'd5: begin // sll
                    output_data <= shift_ans;
                end
                default: output_data <= 64'd0;
            endcase
        end
        else if (state_nxt == S_MULTI_CYCLE_OP || state == S_MULTI_CYCLE_OP) begin
            case (i_inst)
                3'd6: begin // mul
                    if(operand_b[Counter]) begin
                        output_data2 <= (output_data2 + (operand_a << 32)) >> 1;
                    end
                    else begin
                        output_data2 <= output_data2 >> 1;
                    end
                end
                3'd7: begin // div
                    if(Counter == 0) begin
                        if (!div_temp2[64]) begin
                            output_data <= (((operand_a << 1) - (operand_b << 32)) << 1) + 64'd1;
                        end
                        else begin
                            output_data <= operand_a << 2;
                        end
                    end
                    else if (Counter != 31) begin
                        output_data <= (div_temp3[64]) ? output_data << 1 : ((output_data - (operand_b << 32)) << 1) + 64'd1;
                    end
                    else if (Counter == 31) begin
                        output_data <= (div_temp3[64]) ? {(div_temp5[64:32] >> 1) , div_temp5[31:0]} : {(div_temp4[64:32] >> 1) , div_temp4[31:0]};
                    end
                    else output_data <= output_data;
                end
                // div_temp  = (output_data + (operand_a << 1));
                // div_temp2 = (div_temp - (operand_b << 32));
                // div_temp3 = (output_data - (operand_b << 32)); 
                // div_temp4 = ((output_data - (operand_b << 32)) << 1) + 1;
                // div_temp5 = (output_data << 1);
                default: begin
                    output_data <= output_data;
                    output_data2 <= output_data2;
                end
            endcase
        end
        else output_data <= output_data;
    end
    
    // Todo: output valid signal

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            output_valid <= 0;
        end
        else if (state_nxt == S_ONE_CYCLE_OP) begin
            output_valid <= 1;
        end
        else if (state == S_MULTI_CYCLE_OP && Counter == 5'd31) begin
            output_valid <= 1;
        end
        else output_valid <= 0;
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
        end
    end

endmodule