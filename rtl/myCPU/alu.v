module alu(
  input         clk,
  input         reset,
  input         flush,
  input         es_mtc0,//mtc0的结果为alu_src2
  input         ex_stop,//位于异常指令后的指令，不能写hi和lo寄存器
  input         es_valid,
  input  [19:0] alu_op,
  input  [31:0] alu_src1,
  input  [31:0] alu_src2,
  output [31:0] alu_result,
  output [31:0] mem_addr,//优化时序，减少延迟,
  output        div_stall,
  output        over_flow//add addi sub subi溢出判断
);

wire op_add;   //�ӷ�����
wire op_sub;   //��������
wire op_slt;   //�з��űȽϣ�С����λ
wire op_sltu;  //�޷��űȽϣ�С����λ
wire op_and;   //��λ��
wire op_nor;   //��λ���
wire op_or;    //��λ��
wire op_xor;   //��λ���
wire op_sll;   //�߼�����
wire op_srl;   //�߼�����
wire op_sra;   //��������
wire op_lui;   //���������ڸ߰벿��
wire op_mult;
wire op_multu;
wire op_div;
wire op_divu;
wire op_mfhi;
wire op_mflo;
wire op_mthi;
wire op_mtlo;

// control code decomposition
assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
assign op_nor  = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];
assign op_mult = alu_op[12];
assign op_multu= alu_op[13];
assign op_div  = alu_op[14];
assign op_divu = alu_op[15];
assign op_mfhi = alu_op[16];
assign op_mflo = alu_op[17];
assign op_mthi = alu_op[18];
assign op_mtlo = alu_op[19];

wire [31:0] add_sub_result; 
wire [31:0] slt_result; 
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result; 
wire [63:0] sr64_result; 
wire [31:0] sr_result; 
wire [63:0] mult_result;
wire [63:0] div_result;
reg  [31:0] hi=0,lo=0;

//mem result
//因为lw和rw的地址只可能是alu_src1+alu_src2，可以直接运算，绕开alu_result经过的多路选择器
assign mem_addr=alu_src1+alu_src2;

// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;
assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

//over flow
assign over_flow = (adder_a[31] ~^ adder_b[31]) & (adder_result[31] ^ adder_a[31]);//同号相加，结果异号

// ADD, SUB result
assign add_sub_result = adder_result;

// SLT result
assign slt_result[31:1] = 31'b0;
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

// SLTU result
assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;

// bitwise operation
assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = {alu_src2[15:0], 16'b0};

// SLL result 
assign sll_result = alu_src2 << alu_src1[4:0];

// SRL, SRA result
assign sr64_result = {{32{op_sra & alu_src2[31]}}, alu_src2[31:0]} >> alu_src1[4:0];

assign sr_result   = sr64_result[31:0];

//mult,multu,div,divu result
wire [63:0] mult_out;
wire [63:0] multu_out;
mymult mymult(
  .a(alu_src1),
  .b(alu_src2),
  .p(mult_out)
);
mymultu mymultu(
  .a(alu_src1),
  .b(alu_src2),
  .p(multu_out)
);
assign mult_result = {64{op_mult}} & mult_out
                   | {64{op_multu}} & multu_out;

wire div_valid,divu_valid;
wire [63:0] div_out,divu_out;
div_fsm div_fsm(
  .clk(clk),
  .reset(reset),
  .flush(flush),
  .src1(alu_src1),
  .src2(alu_src2),
  .data_valid(op_div),
  .div_out(div_out),
  .res_valid(div_valid)
);
divu_fsm divu_fsm(
  .clk(clk),
  .reset(reset),
  .flush(flush),
  .src1(alu_src1),
  .src2(alu_src2),
  .data_valid(op_divu),
  .div_out(divu_out),
  .res_valid(divu_valid)
);
assign div_stall  = (op_div & ~div_valid) | (op_divu & ~divu_valid);
assign div_result[63:32] = {32{op_div}} & div_out[31:0]
                  | {32{op_divu}} & divu_out[31:0];
assign div_result[31:0] = {32{op_div}} & div_out[63:32]
                  | {32{op_divu}} & divu_out[63:32];

//mthi mtlo
always @(posedge clk) begin
  if(op_div | op_divu) begin
    {hi,lo}=div_result;  
  end
  else if(op_mult | op_multu) begin
    {hi,lo}=mult_result;
  end
  else if(op_mthi) begin
    hi=alu_src1;
  end
  else if(op_mtlo) begin
    lo=alu_src1;
  end  
end

// final result mux
assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                  | ({32{op_slt       }} & slt_result)
                  | ({32{op_sltu      }} & sltu_result)
                  | ({32{op_and       }} & and_result)
                  | ({32{op_nor       }} & nor_result)
                  | ({32{op_or        }} & or_result)
                  | ({32{op_xor       }} & xor_result)
                  | ({32{op_lui       }} & lui_result)
                  | ({32{op_sll       }} & sll_result)
                  | ({32{op_srl|op_sra}} & sr_result)
                  | ({32{op_mfhi}} & hi)
                  | ({32{op_mflo}} & lo)
                  | ({32{es_mtc0}} & alu_src2);

endmodule
