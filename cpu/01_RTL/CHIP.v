//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration

    // opcode
    parameter OP_AUIPC = 7'b0010111;

    parameter OP_JAL   = 7'b1101111;

    parameter OP_JALR  = 7'b1100111;

    parameter OP_ADD   = 7'b0110011;
    parameter OP_SUB   = 7'b0110011;
    parameter OP_AND   = 7'b0110011;
    parameter OP_XOR   = 7'b0110011;
    parameter OP_MUL   = 7'b0110011;

    parameter OP_ADDI  = 7'b0010011;
    parameter OP_SLLI  = 7'b0010011;
    parameter OP_SLTI  = 7'b0010011;
    parameter OP_SRAI  = 7'b0010011;

    parameter OP_LW    = 7'b0000011;

    parameter OP_SW    = 7'b0100011;

    parameter OP_BEQ   = 7'b1100011;
    parameter OP_BGE   = 7'b1100011;
    parameter OP_BLT   = 7'b1100011;
    parameter OP_BNE   = 7'b1100011;

    parameter OP_ECALL = 7'b1110011;

    // funct3 (if same opcode)
    parameter FUNC3_ADD  = 3'b000;
    parameter FUNC3_SUB  = 3'b000;
    parameter FUNC3_AND  = 3'b111;
    parameter FUNC3_XOR  = 3'b100;
    parameter FUNC3_MUL  = 3'b000;

    parameter FUNC3_ADDI = 3'b000;
    parameter FUNC3_SLLI = 3'b001;
    parameter FUNC3_SLTI = 3'b010;
    parameter FUNC3_SRAI = 3'b101;

    parameter FUNC3_BEQ  = 3'b000;
    parameter FUNC3_BGE  = 3'b101;
    parameter FUNC3_BLT  = 3'b100;
    parameter FUNC3_BNE  = 3'b001;

    // funct7
    parameter FUNC7_ADD = 7'b0000000;
    parameter FUNC7_SUB = 7'b0100000;
    parameter FUNC7_AND = 7'b0000000;
    parameter FUNC7_XOR = 7'b0000000;
    parameter FUNC7_MUL = 7'b0000001;

    // state
    parameter STATE_IDLE = 0;
    parameter STATE_OP   = 1;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        //wire mem_cen, mem_wen;
        //wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        //wire mem_stall;
        reg [1:0] state, next_state;
        reg [6:0] opcode, func7;
        reg [2:0] func3;
        reg [4:0] rd, rs1, rs2;
        reg [BIT_W-1:0] imm, rd_data;
        wire[BIT_W-1:0] rs1_data, rs2_data;
        reg o_DMEM_cen_reg, o_DMEM_wen_reg, write_reg, finish, o_proc_finish_reg; 
        reg [BIT_W-1:0] o_DMEM_addr_reg, o_DMEM_wdata_reg;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr  = PC;
    assign o_IMEM_cen   = 1;
    assign o_DMEM_cen   = o_DMEM_cen_reg;
    assign o_DMEM_wen   = o_DMEM_wen_reg;
    assign o_DMEM_addr  = o_DMEM_addr_reg;
    assign o_DMEM_wdata = o_DMEM_wdata_reg;
    assign o_finish = finish;
    assign o_proc_finish = o_proc_finish_reg;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (write_reg),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (rd_data),             
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );

    // MULDIV connection

    wire mul_finish, do_mul;
    wire [31:0] mul_result;
    wire [2:0] mul_inst;
    reg do_mul_reg;
    assign do_mul = do_mul_reg;
    assign mul_inst = 3'd6;

    MULDIV_unit mul0(
        .i_clk(i_clk),
        .i_valid(do_mul),
        .i_rst_n(i_rst_n),
        .i_A(rs1_data),
        .i_B(rs2_data),
        .i_inst(mul_inst),
        .rd(mul_result),
        .o_done(mul_finish)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit
    
    // combinational
    always @(*) begin
            opcode = i_IMEM_data[6:0];
            func3  = i_IMEM_data[14:12];
            func7  = i_IMEM_data[31:25];
            rd     = i_IMEM_data[11:7];
            rs1    = i_IMEM_data[19:15];
            rs2    = i_IMEM_data[24:20];
            imm    = 32'd0;
            case (opcode)
                OP_AUIPC  : imm[31:12] = i_IMEM_data[31:12];
                OP_JAL    : imm[20:1]  = {i_IMEM_data[31], i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21]};
                OP_JALR   : imm[11:0]  = i_IMEM_data[31:20];
                7'b0110011: imm        = 32'd0;
                7'b0010011: begin
                    if (func3 == FUNC3_ADDI || func3 == FUNC3_SLTI) imm[11:0] = i_IMEM_data[31:20];
                    else imm[4:0] = i_IMEM_data[24:20];
                end
                OP_LW     : imm[11:0]  = i_IMEM_data[31:20];
                OP_SW     : imm[11:0]  = {i_IMEM_data[31:25], i_IMEM_data[11:7]};
                7'b1100011: imm[12:1]  = {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8]};
                OP_ECALL  : imm[11:0]  = i_IMEM_data[31:20];
                default: imm = 32'd0;
            endcase    
    end

    always @(*) begin
        next_PC = (!i_DMEM_stall) ? PC + 4 : PC;
        write_reg = 0;
        rd_data = 32'd0;
        o_DMEM_cen_reg = 0;
        o_DMEM_wen_reg = 0;
        o_DMEM_addr_reg = 32'd0;
        o_DMEM_wdata_reg = 32'd0;
        do_mul_reg = 0;
        finish = 0;
        o_proc_finish_reg = 0;
        case (opcode)
            OP_AUIPC: begin
                write_reg = 1;
                rd_data = PC + imm;
            end
            OP_JAL : begin
                write_reg = 1;
                next_PC = $signed({1'd0, PC}) + $signed(imm[20:0]);
                rd_data = PC + 4;
            end
            OP_JALR: begin
                write_reg = 1;
                next_PC = $signed({1'd0, rs1_data}) + $signed(imm[11:0]);
                rd_data = PC + 4;
            end
            7'b0110011 : begin
                case ({func3, func7})
                    {FUNC3_ADD, FUNC7_ADD}: begin
                        write_reg = 1;
                        rd_data = $signed(rs1_data) + $signed(rs2_data);
                    end
                    {FUNC3_SUB, FUNC7_SUB}: begin
                        write_reg = 1;
                        rd_data = $signed(rs1_data) - $signed(rs2_data);
                    end
                    {FUNC3_AND, FUNC7_AND}: begin
                        write_reg = 1;
                        rd_data = rs1_data & rs2_data;
                    end
                    {FUNC3_XOR, FUNC7_XOR}: begin
                        write_reg = 1;
                        rd_data = rs1_data ^ rs2_data;
                    end
                    {FUNC3_MUL, FUNC7_MUL}: begin
                        write_reg = (mul_finish) ? 1 : 0;
                        do_mul_reg = (!mul_finish);
                        rd_data = mul_result;
                        next_PC = (mul_finish) ? PC + 4 : PC;
                    end
                    default: begin
                        next_PC = PC;
                        write_reg = 0;
                        rd_data = 32'd0;
                    end
                endcase
            end
            7'b0010011 : begin
                case (func3)
                    FUNC3_ADDI: begin
                        write_reg = 1;
                        rd_data = $signed(rs1_data) + $signed(imm[11:0]);
                    end
                    FUNC3_SLLI: begin
                        write_reg = 1;
                        rd_data = rs1_data << imm[4:0];
                    end
                    FUNC3_SLTI: begin
                        write_reg = 1;
                        rd_data = ($signed(rs1_data) < $signed(imm[11:0])) ? 1 : 0;
                    end
                    FUNC3_SRAI: begin
                        write_reg = 1;
                        rd_data = $signed(rs1_data) >>> imm[4:0];
                    end
                    default: begin
                        write_reg = 0;
                        rd_data = 32'd0;
                    end
                endcase
            end
            OP_LW: begin
                write_reg = 1;
                o_DMEM_cen_reg = 1;
                o_DMEM_addr_reg = $signed({1'd0, rs1_data}) + $signed(imm[11:0]);
                rd_data = i_DMEM_rdata;
            end
            OP_SW: begin
                o_DMEM_cen_reg = 1;
                o_DMEM_wen_reg = 1;
                o_DMEM_addr_reg = $signed({1'd0, rs1_data}) + $signed(imm[11:0]);
                o_DMEM_wdata_reg = rs2_data;
            end
            7'b1100011 : begin
                case (func3)
                    FUNC3_BEQ: begin
                        next_PC = (rs1_data == rs2_data) ? $signed({1'd0, PC}) + $signed(imm[12:0]) : PC + 4;
                    end
                    FUNC3_BGE: begin
                        next_PC = ($signed(rs1_data) >= $signed(rs2_data)) ? $signed({1'd0, PC}) + $signed(imm[12:0]) : PC + 4;
                    end
                    FUNC3_BLT: begin
                        next_PC = ($signed(rs1_data) < $signed(rs2_data)) ? $signed({1'd0, PC}) + $signed(imm[12:0]) : PC + 4;
                    end
                    FUNC3_BNE: begin
                        next_PC = (rs1_data != rs2_data) ? $signed({1'd0, PC}) + $signed(imm[12:0]) : PC + 4;
                    end
                    default: next_PC = PC;
                endcase
            end
            OP_ECALL : begin
                o_proc_finish_reg = 1;
                finish = (i_cache_finish) ? 1 : 0;
                next_PC = PC;
            end
            default: begin
                next_PC = PC;
                write_reg = 0;
                rd_data = 32'd0;
                o_DMEM_cen_reg = 0;
                o_DMEM_wen_reg = 0;
                o_DMEM_addr_reg = 0;
                o_DMEM_wdata_reg = 32'd0;
                finish = 0;
                o_proc_finish_reg = 0;
            end
        endcase
    end

    // sequential

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= STATE_OP;
        end
        else begin
            PC <= next_PC;
            state <= next_state;
        end
    end

    // FSM
    always @(*) begin
        case (state)
            STATE_IDLE : next_state = (!i_DMEM_stall)? STATE_OP : STATE_IDLE;
            STATE_OP   : next_state = (!i_DMEM_stall)? STATE_OP : STATE_IDLE;
            default: next_state = state;
        endcase
    end

endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit#(
    parameter DATA_W = 32
)
(
        input i_clk,
        input i_valid,
        input i_rst_n,
        input [31:0] i_A, 
        input [31:0] i_B,
        input [2:0]  i_inst,
        output [31:0] rd,
        output o_done
);

// Parameters
    // FSM
    parameter S_IDLE           = 3'd0;
    parameter S_ONE_CYCLE_OP   = 3'd1;
    parameter S_MULTI_CYCLE_OP = 3'd2;
    parameter S_BUFFER         = 3'd3;

// Wires & Regs
    // state
    reg  [         2: 0] state, state_nxt;
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;
    wire [63:0] o_data;

// Wire Assignments

    reg [4:0] Counter;
    reg [64:0] output_data2, output_data2_next;
    assign o_data = (i_inst == 3'd6) ? output_data2[63:0] : 0;
    assign rd = output_data2_next[31:0];
    reg output_valid;
    assign o_done = output_valid;

    wire [31:0] add_result;
    assign add_result = operand_a + operand_b;
    wire [31:0] sub_result;
    assign sub_result = operand_a - operand_b;
    wire [31:0] shift_ans = operand_a<<(operand_b);

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
    // FSM
    always @(*) begin
        case(state)
            S_IDLE           : state_nxt = (i_valid) ? S_BUFFER : S_IDLE;
            S_BUFFER         : state_nxt = ((i_inst[2] && i_inst[1]) ? S_MULTI_CYCLE_OP : S_ONE_CYCLE_OP);
            S_ONE_CYCLE_OP   : state_nxt = S_IDLE;
            S_MULTI_CYCLE_OP : state_nxt = (Counter == 5'd31)? S_IDLE : S_MULTI_CYCLE_OP;
            default : state_nxt = state;
        endcase
    end

    // Counter

    always @(negedge i_rst_n or posedge i_clk) begin
        if(!i_rst_n) Counter <= 5'd0;
        else if(state_nxt == S_BUFFER    && Counter == 5'd31) Counter <= 5'd0;
        else if(state_nxt == S_MULTI_CYCLE_OP) Counter <= Counter + 5'd1;
        else Counter <= Counter;
    end

    // ALU output
    
    always @(*) begin
        output_data2_next = 65'd0;
        if(i_valid || state_nxt == S_MULTI_CYCLE_OP || state == S_MULTI_CYCLE_OP) begin
            case (i_inst)
                3'd6: begin // mul
                    if(operand_b[Counter]) begin
                        output_data2_next = (output_data2 + (operand_a << 32)) >> 1;
                    end
                    else begin
                        output_data2_next = output_data2 >> 1;
                    end
                end

                default: begin
                    output_data2_next = output_data2;
                end
            endcase
        end
        else output_data2_next = output_data2;
    end
    
    // output valid signal

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            output_valid <= 0;
        end
        else if (state_nxt == S_ONE_CYCLE_OP) begin
            output_valid <= 1;
        end
        else if (state == S_MULTI_CYCLE_OP && Counter == 5'd30) begin
            output_valid <= 1;
        end
        else output_valid <= 0;
    end

    // Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
            output_data2 <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
            output_data2 <= output_data2_next;
        end
    end

endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    // assign o_mem_cen = i_proc_cen;              //
    // assign o_mem_wen = i_proc_wen;              //
    // assign o_mem_addr = i_proc_addr;            //
    // assign o_mem_wdata = i_proc_wdata;          //
    // assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    // assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS
        // state
        parameter STATE_IDLE       = 0;
        parameter STATE_READ       = 1;
        parameter STATE_WRITE      = 2;
        parameter STATE_WB         = 3;
        parameter STATE_ALLO       = 4;
        parameter STATE_FINALSTORE = 5;
        

        // declare
        parameter block_num = 16; // 16 blocks cache
        reg [2:0] state, next_state;
        reg o_proc_stall_reg, o_mem_cen_reg, o_mem_wen_reg, cen, cen_next, wen, wen_next, hit, finish;        
        reg [1:0] block_offset, block_offset_next;
        reg [3:0] index, index_next, counter;
        reg [23:0] tag [0:block_num-1], tag_next[0:block_num-1]; // 32 - 2(byte offset) - 2(block offset) - 4(numbers of blocks)
        reg valid [0:block_num-1], valid_next[0:block_num-1], dirty [0:block_num-1], dirty_next[0:block_num-1];
        reg [4*BIT_W-1:0] data [0:block_num-1], data_next [0:block_num-1];
        reg [ADDR_W-1:0] proc_addr, real_addr, o_mem_addr_reg;
        wire [31:0] mem_addr;
        

        assign o_mem_cen = o_mem_cen_reg;
        assign o_mem_wen = o_mem_wen_reg;
        assign mem_addr = i_proc_addr - i_offset; // corresponding address word  in memory
        assign o_mem_addr = o_mem_addr_reg;
        assign o_proc_stall = o_proc_stall_reg;
        assign o_mem_wdata = data[index];
        assign o_proc_rdata = data[index][(ADDR_W * block_offset) +: ADDR_W];
        assign o_cache_finish = finish;
        
        // FSM
        always @(posedge i_clk or negedge i_rst_n) begin
            if(!i_rst_n) state <= STATE_IDLE;
            else state <= next_state;
        end

        always @(*) begin
            case(state)
                STATE_IDLE : begin
                    if(i_proc_finish) next_state = STATE_FINALSTORE;
                    else begin
                        if (i_proc_cen && !i_proc_wen) next_state = STATE_READ;
                        else if (i_proc_cen && i_proc_wen) next_state = STATE_WRITE;
                        else next_state = STATE_IDLE;
                    end
                end
                STATE_READ : begin
                    if(hit) next_state = STATE_IDLE;
                    else begin
                        if(dirty[index]) next_state = STATE_WB;
                        else next_state = STATE_ALLO;
                    end
                end
                STATE_WRITE : begin
                    if(hit) next_state = STATE_IDLE;
                    else next_state = STATE_WB;
                end
                STATE_WB : begin
                    if(!i_mem_stall && i_proc_wen) next_state = STATE_WRITE;
                    else if(!i_mem_stall) next_state = STATE_ALLO;
                    else next_state = STATE_WB;
                end
                STATE_ALLO : begin
                    if(!i_mem_stall) next_state = STATE_READ;
                    else next_state = STATE_ALLO;
                end
                STATE_FINALSTORE : begin
                    if(counter == (block_num-1) && !i_mem_stall) next_state = STATE_IDLE; // store all valid data back to memory
                    else next_state = STATE_FINALSTORE;
                end
                default : next_state = state;
            endcase
        end

        // finish
        always @(*) begin
            if((state == STATE_FINALSTORE) && (counter == (block_num-1) && !i_mem_stall)) finish = 1;
            else finish = 0;
        end

        // combinational
        integer i;
        always @(*) begin
            cen_next = cen;
            wen_next = wen;
            o_mem_cen_reg = 0;
            o_mem_wen_reg = 0;
            o_mem_addr_reg = i_proc_addr;
            block_offset = mem_addr[3:2];
            index = mem_addr[7:4];
            hit = ((tag[index] == mem_addr[31:8]) && valid[index]) ? 1 : 0;
            for (i=0;i<block_num;i=i+1) begin
                valid_next[i] = 0;
                dirty_next[i] = 0;
                tag_next[i] = 0;
                data_next[i] = 0;
            end
            case(state)
                STATE_IDLE : begin      
                    o_proc_stall_reg = i_proc_cen;
                    cen_next = i_proc_cen;
                    wen_next = i_proc_wen;
                    valid_next[index] = valid[index];
                    dirty_next[index] = dirty[index];
                    tag_next[index] = tag[index];
                    data_next[index] = data[index];
                end
                STATE_READ : begin
                    o_proc_stall_reg = !hit;
                    valid_next[index] = 1;
                    dirty_next[index] = dirty[index];
                    tag_next[index] = tag[index];
                    data_next[index] = data[index];
                end
                STATE_WRITE: begin
                    o_proc_stall_reg = !hit;
                    valid_next[index] = 1;
                    dirty_next[index] = wen ? 1 : dirty[index];
                    tag_next[index] = mem_addr[31:8];
                    data_next[index] = data[index];
                    data_next[index][(ADDR_W * block_offset) +: ADDR_W] = (hit && wen) ? i_proc_wdata : data[index][(ADDR_W * block_offset) +: ADDR_W];
                end
                STATE_WB : begin
                    o_proc_stall_reg = 1;
                    o_mem_cen_reg = 1;
                    o_mem_wen_reg = 1;
                    index = mem_addr[7:4];
                    o_mem_addr_reg = {tag[index], index, 4'd0} + i_offset;
                    valid_next[index] = valid[index];
                    dirty_next[index] = dirty[index];
                    tag_next[index] = tag[index];
                    data_next[index] = data[index];
                end
                STATE_ALLO : begin
                    o_proc_stall_reg = 1;
                    o_mem_cen_reg = 1;
                    valid_next[index] = 1;
                    dirty_next[index] = dirty[index];
                    tag_next[index] = mem_addr[31:8];
                    data_next[index] = i_mem_rdata;
                end
                STATE_FINALSTORE : begin
                    o_proc_stall_reg = 1;
                    index = counter;
                    o_mem_cen_reg = valid[index];
                    o_mem_wen_reg = valid[index];
                    o_mem_addr_reg = {tag[index], index, 4'd0} + i_offset;
                    valid_next[index] = valid[index];                 
                    dirty_next[index] = 0;
                    tag_next[index] = tag[index];
                    data_next[index] = data[index];
                end
                default : begin
                    o_proc_stall_reg = 0;
                    cen_next = 0;
                    wen_next = 0;
                    o_mem_cen_reg = 0;
                    o_mem_wen_reg = 0;
                    valid_next[index] = valid[index];
                    dirty_next[index] = dirty[index];
                    tag_next[index] = tag[index];
                    data_next[index] = 0;
                end
            endcase
        end


        // only store valid data back to memory after processor done
        wire [5:0] compare;
        assign compare = index;

        always @(posedge i_clk) begin
            if (state == STATE_FINALSTORE) begin
                if (!i_mem_stall) begin
                    if ((compare + 14) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else if(valid[index+9])  counter <= counter + 9;
                        else if(valid[index+10])  counter <= counter + 10;
                        else if(valid[index+11])  counter <= counter + 11;
                        else if(valid[index+12])  counter <= counter + 12;
                        else if(valid[index+13])  counter <= counter + 13;
                        else counter <= counter + 14;
                    end
                    else if ((compare + 13) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else if(valid[index+9])  counter <= counter + 9;
                        else if(valid[index+10])  counter <= counter + 10;
                        else if(valid[index+11])  counter <= counter + 11;
                        else if(valid[index+12])  counter <= counter + 12;
                        else counter <= counter + 13;
                    end
                    else if ((compare + 12) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else if(valid[index+9])  counter <= counter + 9;
                        else if(valid[index+10])  counter <= counter + 10;
                        else if(valid[index+11])  counter <= counter + 11;
                        else counter <= counter + 12;
                    end
                    else if ((compare + 11) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else if(valid[index+9])  counter <= counter + 9;
                        else if(valid[index+10])  counter <= counter + 10;
                        else counter <= counter + 11;
                    end
                    else if ((compare + 10) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else if(valid[index+9])  counter <= counter + 9;
                        else counter <= counter + 10;
                    end
                    else if ((compare + 9) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else if(valid[index+8])  counter <= counter + 8;
                        else counter <= counter + 9;
                    end
                    else if ((compare + 8) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else if(valid[index+7])  counter <= counter + 7;
                        else counter <= counter + 8;
                    end
                    else if ((compare + 7) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else if(valid[index+6])  counter <= counter + 6;
                        else counter <= counter + 7;
                    end
                    else if ((compare + 6) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else if(valid[index+5])  counter <= counter + 5;
                        else counter <= counter + 6;
                    end
                    else if ((compare + 5) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else if(valid[index+4])  counter <= counter + 4;
                        else counter <= counter + 5;
                    end
                    else if ((compare + 4) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else if(valid[index+3])  counter <= counter + 3;
                        else counter <= counter + 4;
                    end
                    else if ((compare + 3) <= (block_num -1)) begin
                        if(valid[index+1]) counter <=  counter + 1;
                        else if(valid[index+2])  counter <= counter + 2;
                        else counter <= counter + 3;
                    end
                    else begin
                        if ((compare + 2) <= (block_num -1)) begin
                            counter <= (valid[index+1])?  counter + 1 : counter + 2;
                        end
                        else counter <= counter + 1;
                    end
                end
                else counter <= counter;
            end
            else counter <= 0;
        end


        // sequential
        always @(posedge i_clk or negedge i_rst_n) begin
            if(!i_rst_n) begin
                cen <= 0;
                wen <= 0; 
                for(i=0;i<block_num;i=i+1) begin
                    valid[i] <= 0;
                    dirty[i] <= 0;
                    tag[i] <= 0;
                    data[i] <= 0;
                end
            end
            else begin
                cen <= cen_next;
                wen <= wen_next;                
                if(cen) begin
                    valid[index] <= valid_next[index];
                    dirty[index] <= dirty_next[index];
                    tag[index]  <= tag_next[index];
                    data[index] <= data_next[index];
                end
            end
        end

endmodule