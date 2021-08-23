`include "mycpu.h"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //stall
    input                          stallD        ,
    output                         ds_use_rs     ,//是否使用到寄存器rs
    output                         ds_use_rt     ,//是否使用到寄存器rt
    output [4:0]                   rs_addr       ,
    output [4:0]                   rt_addr       ,//检查ALU操作数是否存在冲突
    //forward
    input [1:0]                    forward_rs    ,
    input [1:0]                    forward_rt    ,
    input [35:0]                   ms_to_ds_bus  ,//mem阶段前递数据
    input [31:0]                   es_to_ds_bus  ,//exe阶段前递数据
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    input  [`FS_EX_BUS_WD    -1:0] fs_ex_bus     ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    output [4:0]                   ds_load_mem_bus,
    output [3:0]                   ds_save_mem_bus,
    output [`DS_EX_BUS_WD    -1:0] ds_ex_bus     ,
    //from cp0
    input                          flush         ,
    input                          es_ex         ,
    input                          ms_ex         ,
    input                          ws_ex         ,  
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus
);

reg         ds_valid   ;
wire        ds_ready_go;

wire [31                 :0] fs_pc;
reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;
reg  [`FS_EX_BUS_WD    -1:0] fs_ex_bus_r;

wire [31:0] ds_inst;
wire [31:0] ds_pc  ;
assign {ds_inst,
        ds_pc  } = fs_to_ds_bus_r;
assign fs_pc = ds_pc + 3'h4;

wire [ 3:0] rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
assign {rf_we   ,  //40:37 寄存器堆写使能
        rf_waddr,  //36:32 写地址
        rf_wdata   //31:0  写数据
       } = ws_to_rf_bus;//write back阶段写回总线

wire [ 3:0] ms_rf_we;
wire [31:0] ms_rf_wdata;
assign {ms_rf_wdata ,
        ms_rf_we
       } = ms_to_ds_bus;

wire        br_taken;
wire [31:0] br_target;

wire [19:0] alu_op;
wire        load_op;
wire        src1_is_sa;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_8;
wire        src2_zero_extend;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;
wire [ 4:0] dest;
wire [15:0] imm;
wire [31:0] rs_value;
wire [31:0] rt_value;

wire [ 5:0] op;
wire [ 4:0] rs;
wire [ 4:0] rt;
wire [ 4:0] rd;
wire [ 4:0] sa;
wire [ 5:0] func;
wire [25:0] jidx;
wire [63:0] op_d;
wire [31:0] rs_d;
wire [31:0] rt_d;
wire [31:0] rd_d;
wire [31:0] sa_d;
wire [63:0] func_d;

wire        inst_add;
wire        inst_addu;
wire        inst_sub;
wire        inst_subu;
wire        inst_mult;
wire        inst_multu;
wire        inst_div;
wire        inst_divu;
wire        inst_mfhi;
wire        inst_mflo;
wire        inst_mthi;
wire        inst_mtlo;
wire        inst_slt;
wire        inst_slti;
wire        inst_sltu;
wire        inst_sltiu;
wire        inst_and;
wire        inst_andi;
wire        inst_or;
wire        inst_ori;
wire        inst_xor;
wire        inst_xori;
wire        inst_nor;
wire        inst_sll;
wire        inst_srl;
wire        inst_sra;
wire        inst_sllv;
wire        inst_srlv;
wire        inst_srav;
wire        inst_addi;
wire        inst_addiu;
wire        inst_lui;
wire        inst_lb;
wire        inst_lbu;
wire        inst_lh;
wire        inst_lhu;
wire        inst_lw;
wire        inst_lwl;
wire        inst_lwr;
wire        inst_sw;
wire        inst_sb;
wire        inst_sh;
wire        inst_swl;
wire        inst_swr;
wire        inst_beq;
wire        inst_bne;
wire        inst_bgtz;
wire        inst_bgez;
wire        inst_blez;
wire        inst_bltz;
wire        inst_bgezal;
wire        inst_bltzal;
wire        inst_jal;
wire        inst_jr;
wire        inst_j;
wire        inst_jalr;

//branch & jump
wire        branch_zero;
wire        is_branch;

wire [1:0]  load_width;
wire        load_sign;
wire        load_lr;

wire        save_op;
wire [3:0]  save_width;
wire [1:0]  save_lr;

wire        dst_is_r31;  
wire        dst_is_rt;   

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire        rs_eq_rt;
wire        rs_gt_z;
wire        rs_lt_z;

assign br_bus       = {br_taken,br_target};

assign ds_to_es_bus = {alu_op      ,  //144:125
                       load_op     ,  //124:124
                       src1_is_sa  ,  //123:123
                       src1_is_pc  ,  //122:122
                       src2_is_imm ,  //121:121
                       src2_is_8   ,  //120:120
                       src2_zero_extend,//119:119
                       gr_we       ,  //118:118
                       mem_we      ,  //117:117
                       dest        ,  //116:112
                       imm         ,  //111:96
                       rs_value    ,  //95 :64
                       rt_value    ,  //63 :32
                       ds_pc //31 :0
                      };

assign ds_ready_go    = ~stallD;
assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin;
assign ds_to_es_valid = ds_valid && ds_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ds_valid<=1'b0;
    end
    else if(flush) begin
        ds_valid<=1'b0;
    end
    else if(ds_allowin) begin
        ds_valid<=fs_to_ds_valid;
    end
    
    if(flush) begin
        fs_to_ds_bus_r <=0;
        fs_ex_bus_r    <=0;    
    end
    else if (fs_to_ds_valid && ds_allowin) begin
        fs_to_ds_bus_r <= fs_to_ds_bus;
        fs_ex_bus_r    <= fs_ex_bus;
    end
end

assign op   = ds_inst[31:26];
assign rs   = ds_inst[25:21];
assign rt   = ds_inst[20:16];
assign rd   = ds_inst[15:11];
assign sa   = ds_inst[10: 6];
assign func = ds_inst[ 5: 0];
assign imm  = ds_inst[15: 0];
assign jidx = ds_inst[25: 0];

decoder_6_64 u_dec0(.in(op  ), .out(op_d  ));
decoder_6_64 u_dec1(.in(func), .out(func_d));
decoder_5_32 u_dec2(.in(rs  ), .out(rs_d  ));
decoder_5_32 u_dec3(.in(rt  ), .out(rt_d  ));
decoder_5_32 u_dec4(.in(rd  ), .out(rd_d  ));
decoder_5_32 u_dec5(.in(sa  ), .out(sa_d  ));

//exception bus : ds -> es
wire        inst_sys;
wire        inst_mfc0;
wire        inst_mtc0;
wire        inst_eret;
wire        inst_break;
wire        ov_valid;
wire        ds_adel;
wire        ds_ri;
wire [31:0] ds_badvaddr;
wire        ds_ex;
wire        ds_stop;
assign {ds_adel,ds_badvaddr} = fs_ex_bus_r;
assign ds_ex_bus = {is_branch   ,//branch delay
                    inst_sys    ,//sys call
                    inst_mfc0   ,//mfc0
                    inst_mtc0   ,//mtc0
                    inst_eret   ,//eret
                    inst_break  ,
                    ov_valid    ,
                    ds_adel     ,
                    ds_ri       ,
                    rd          ,
                    ds_badvaddr
                   };

//inst decode
assign inst_add    = op_d[6'h00] & func_d[6'h20] & sa_d[5'h00];
assign inst_addu   = op_d[6'h00] & func_d[6'h21] & sa_d[5'h00];
assign inst_sub    = op_d[6'h00] & func_d[6'h22] & sa_d[5'h00];
assign inst_subu   = op_d[6'h00] & func_d[6'h23] & sa_d[5'h00];
assign inst_mult   = op_d[6'h00] & func_d[6'h18] & sa_d[5'h00] & rd_d[5'h00];
assign inst_multu  = op_d[6'h00] & func_d[6'h19] & sa_d[5'h00] & rd_d[5'h00];
assign inst_div    = op_d[6'h00] & func_d[6'h1a] & sa_d[5'h00] & rd_d[5'h00];
assign inst_divu   = op_d[6'h00] & func_d[6'h1b] & sa_d[5'h00] & rd_d[5'h00];
assign inst_mfhi   = op_d[6'h00] & func_d[6'h10] & rs_d[5'h00] & rt_d[5'h00] & sa_d[5'h00];
assign inst_mflo   = op_d[6'h00] & func_d[6'h12] & rs_d[5'h00] & rt_d[5'h00] & sa_d[5'h00];
assign inst_mthi   = op_d[6'h00] & func_d[6'h11] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_mtlo   = op_d[6'h00] & func_d[6'h13] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_slt    = op_d[6'h00] & func_d[6'h2a] & sa_d[5'h00];
assign inst_sltu   = op_d[6'h00] & func_d[6'h2b] & sa_d[5'h00];
assign inst_and    = op_d[6'h00] & func_d[6'h24] & sa_d[5'h00];
assign inst_or     = op_d[6'h00] & func_d[6'h25] & sa_d[5'h00];
assign inst_xor    = op_d[6'h00] & func_d[6'h26] & sa_d[5'h00];
assign inst_nor    = op_d[6'h00] & func_d[6'h27] & sa_d[5'h00];
assign inst_sll    = op_d[6'h00] & func_d[6'h00] & rs_d[5'h00];
assign inst_srl    = op_d[6'h00] & func_d[6'h02] & rs_d[5'h00];
assign inst_sra    = op_d[6'h00] & func_d[6'h03] & rs_d[5'h00];
assign inst_sllv   = op_d[6'h00] & func_d[6'h04] & sa_d[5'h00];
assign inst_srlv   = op_d[6'h00] & func_d[6'h06] & sa_d[5'h00];
assign inst_srav   = op_d[6'h00] & func_d[6'h07] & sa_d[5'h00];
assign inst_addi   = op_d[6'h08];
assign inst_addiu  = op_d[6'h09];
assign inst_slti   = op_d[6'h0a];
assign inst_sltiu  = op_d[6'h0b];
assign inst_andi   = op_d[6'h0c];
assign inst_ori    = op_d[6'h0d];
assign inst_xori   = op_d[6'h0e];
assign inst_lui    = op_d[6'h0f] & rs_d[5'h00];
assign inst_lb     = op_d[6'h20];
assign inst_lbu    = op_d[6'h24];
assign inst_lh     = op_d[6'h21];
assign inst_lhu    = op_d[6'h25];
assign inst_lw     = op_d[6'h23];
assign inst_lwl    = op_d[6'h22];
assign inst_lwr    = op_d[6'h26];
assign inst_sw     = op_d[6'h2b];
assign inst_sb     = op_d[6'h28];
assign inst_sh     = op_d[6'h29];
assign inst_swl    = op_d[6'h2a];
assign inst_swr    = op_d[6'h2e];
assign inst_beq    = op_d[6'h04];
assign inst_bne    = op_d[6'h05];
assign inst_bgez   = op_d[6'h01] & rt_d[5'h01];
assign inst_bgtz   = op_d[6'h07] & rt_d[5'h00];
assign inst_blez   = op_d[6'h06] & rt_d[5'h00];
assign inst_bltz   = op_d[6'h01] & rt_d[5'h00];
assign inst_bgezal = op_d[6'h01] & rt_d[5'h11];
assign inst_bltzal = op_d[6'h01] & rt_d[5'h10];
assign inst_jal    = op_d[6'h03];
assign inst_jr     = op_d[6'h00] & func_d[6'h08] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_j      = op_d[6'h02];
assign inst_jalr   = op_d[6'h00] & func_d[6'h09] & rt_d[5'h00] & sa_d[5'h00];

//alu encode
assign alu_op[ 0] = inst_add | inst_addu | inst_addi | inst_addiu | load_op | save_op | inst_jal | inst_bgezal | inst_bltzal | inst_jalr;//add
assign alu_op[ 1] = inst_sub | inst_subu;//sub
assign alu_op[ 2] = inst_slt | inst_slti;//slt
assign alu_op[ 3] = inst_sltu | inst_sltiu;//sltu
assign alu_op[ 4] = inst_and | inst_andi;//and
assign alu_op[ 5] = inst_nor;//nor
assign alu_op[ 6] = inst_or | inst_ori;//or
assign alu_op[ 7] = inst_xor | inst_xori;//xor
assign alu_op[ 8] = inst_sll | inst_sllv;//sll
assign alu_op[ 9] = inst_srl | inst_srlv;//srl
assign alu_op[10] = inst_sra | inst_srav;//sra
assign alu_op[11] = inst_lui;//lui
assign alu_op[12] = inst_mult;
assign alu_op[13] = inst_multu;
assign alu_op[14] = inst_div;
assign alu_op[15] = inst_divu;
assign alu_op[16] = inst_mfhi;
assign alu_op[17] = inst_mflo;
assign alu_op[18] = inst_mthi;
assign alu_op[19] = inst_mtlo;

//exception decode
assign inst_sys    = op_d[6'h00] & func_d[6'h0c];
assign inst_break  = op_d[6'h00] & func_d[6'h0d];
assign inst_mfc0   = op_d[6'h10] & rs_d[5'h00] & (ds_inst[10:3]==8'h00);
assign inst_mtc0   = op_d[6'h10] & rs_d[5'h04] & (ds_inst[10:3]==8'h00);
assign inst_eret   = op_d[6'h10] & func_d[6'h18] & ds_inst[25] & (ds_inst[24:6]==19'b0);
assign ov_valid    = inst_add | inst_addi | inst_sub;
assign ds_ri       = alu_op==20'b0 && ~save_op && ~load_op 
                   && ~inst_bne && ~inst_beq && ~inst_j && ~inst_jr
                   && ~inst_bltz && ~inst_blez && ~inst_bgtz && ~inst_bgez
                   && ~inst_sys && ~inst_break && ~inst_mfc0
                   && ~inst_mtc0 && ~inst_eret
                   && ds_inst!=32'b0; 
assign ds_ex       = inst_sys | inst_mfc0 | inst_mtc0 | inst_eret | inst_break | ds_ri;
assign ds_stop     = ds_ex | es_ex | ms_ex | ws_ex;
//load op
assign load_op      = inst_lw | inst_lb | inst_lbu | inst_lh | inst_lhu | load_lr;//是否从内存中获取数据
assign load_width   = {2{inst_lb | inst_lbu}} & 2'b01 
                    | {2{inst_lh | inst_lhu}} & 2'b10
                    | {2{inst_lw | load_lr}} & 2'b11;
assign load_sign    = inst_lb | inst_lh;
assign load_lr      = inst_lwl | inst_lwr;
assign ds_load_mem_bus = {load_width,load_sign,inst_lwl,inst_lwr};
//load op

//save op
assign save_op      = inst_sw | inst_sb | inst_sh | inst_swl | inst_swr;
assign save_width   = {2{inst_sw}} & 2'b11
                    | {2{inst_sh}} & 2'b10
                    | {2{inst_sb}} & 2'b01;
                    //default: swl swr -> 2'b00
assign save_lr      = {inst_swl,inst_swr}; 
assign ds_save_mem_bus = {save_width,save_lr};
//save op

//需要与0比较的跳转信号
assign branch_zero  = inst_bgez | inst_bgtz | inst_blez | inst_bltz | inst_bgezal | inst_bltzal;

assign src1_is_sa   = inst_sll   | inst_srl | inst_sra;
assign src1_is_pc   = inst_jal | inst_bgezal | inst_bltzal | inst_jalr;
//需要用到rs寄存器
assign ds_use_rs    = (~(src1_is_sa | src1_is_pc | inst_j | ds_ex) | inst_bgezal | inst_bltzal | inst_jalr) & ds_valid;
assign rs_addr      = rs;

assign src2_is_imm  = inst_addi | inst_addiu | inst_slti | inst_sltiu | inst_andi | inst_ori | inst_xori | inst_lui | load_op | save_op;
assign src2_is_8    = inst_jal | inst_bgezal | inst_bltzal | inst_jalr;
assign src2_zero_extend = inst_andi | inst_ori | inst_xori;
//sw需要将rt中的值写入内存，因此也需要检查写后读相关
assign ds_use_rt    = (~(src2_is_imm | src2_is_8 | inst_mthi | inst_mtlo | branch_zero | inst_j | ds_ex) | save_op | inst_mtc0) & ds_valid;
assign rt_addr      = rt;

assign res_from_mem = load_op;
assign dst_is_r31   = inst_jal | inst_bgezal | inst_bltzal;
//需要写rt寄存器
assign dst_is_rt    = inst_addi | inst_addiu | inst_slti | inst_sltiu | inst_andi | inst_ori | inst_xori | inst_lui | load_op | inst_mfc0;

//寄存器写使能
assign gr_we        = (~save_op & ~inst_beq & ~inst_bne & ~inst_jr & ~branch_zero & ~inst_j & ~ds_ex) | (inst_bgezal | inst_bltzal | inst_mfc0);
assign mem_we       = save_op;

assign dest         = dst_is_r31 ? 5'd31 :
                      dst_is_rt  ? rt    : 
                                   rd;

assign rf_raddr1 = rs;
assign rf_raddr2 = rt;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

//data forward
wire [31:0] ms_forward_data;
wire [31:0] ws_forward_data;
forward_data ms_forward(
    .a   (ms_rf_wdata)    ,
    .b   (rf_rdata1)      ,
    .we  (ms_rf_we)       ,
    .out (ms_forward_data)
);
forward_data ws_forward(
    .a   (rf_wdata)       ,
    .b   (rf_rdata2)      ,
    .we  (rf_we)          ,
    .out (ws_forward_data)
);

assign rs_value = {32{forward_rs==2'b11}} & es_to_ds_bus
                | {32{forward_rs==2'b10}} & ms_forward_data 
                | {32{forward_rs==2'b01}} & ws_forward_data
                | {32{forward_rs==2'b00}} & rf_rdata1;//mux 根据冲突情况选择数据源
assign rt_value = {32{forward_rt==2'b11}} & es_to_ds_bus
                | {32{forward_rt==2'b10}} & ms_forward_data 
                | {32{forward_rt==2'b01}} & ws_forward_data
                | {32{forward_rt==2'b00}} & rf_rdata2;
                
//branch
assign rs_eq_rt = (rs_value == rt_value);
assign rs_gt_z  = ($signed(rs_value) > 0);
assign rs_lt_z  = ($signed(rs_value) < 0);
assign is_branch = (inst_beq | inst_bne | branch_zero | inst_jal | inst_jr | inst_j | inst_jalr) & ds_valid; 
assign br_taken = (   inst_beq  &&  rs_eq_rt
                   || inst_bne  && !rs_eq_rt 
                   || inst_bgtz && rs_gt_z
                   || (inst_bgez || inst_bgezal) && !rs_lt_z
                   || (inst_bltz || inst_bltzal) && rs_lt_z
                   || inst_blez && !rs_gt_z
                   || inst_jal
                   || inst_jr
                   || inst_j
                   || inst_jalr
                  ) && ds_valid && !ds_stop && !stallD;
assign br_target = (inst_beq || inst_bne || branch_zero) ? (fs_pc  + {{14{imm[15]}}, imm[15:0], 2'b0}) :
                   (inst_jr || inst_jalr)                ? rs_value :
                   {fs_pc[31:28], jidx[25:0], 2'b0};// j & jal

endmodule
