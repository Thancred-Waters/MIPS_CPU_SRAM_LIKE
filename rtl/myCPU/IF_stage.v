`include "mycpu.h"

module if_stage(
    input                          clk            ,
    input                          reset          ,
    //allwoin
    input                          ds_allowin     ,
    //br bus
    input  [`BR_BUS_WD       -1:0] br_bus         ,
    //to ds
    output                         fs_to_ds_valid ,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus   ,
    output [`FS_EX_BUS_WD    -1:0] fs_ex_bus      ,
    //from cp0
    input                          ex_flush       ,
    input                          eret_flush     ,
    input  [31:0]                  eret_pc        ,
    // inst sram interface
    output        inst_sram_req,
    output        inst_sram_wr,
    output [ 1:0] inst_sram_size,
    output [31:0] inst_sram_addr,
    output [ 3:0] inst_sram_wstrb,
    output [31:0] inst_sram_wdata,
    input         inst_sram_addr_ok,
    input         inst_sram_data_ok,
    input  [31:0] inst_sram_rdata
);

reg         fs_valid;
wire        fs_ready_go;
wire        fs_allowin;
wire        to_fs_valid;
wire        to_fs_ready_go;

wire [31:0] seq_pc;
wire [31:0] nextpc;

wire         br_taken;//是否选择将nextpc置为跳转地址
wire [ 31:0] br_target;//pc跳转目标
wire [ 31:0] br_target_r;
wire         br_taken_r;
assign br_taken  = br_bus[32];
assign br_target = br_bus[31:0];

wire [31:0] fs_inst;
wire        fs_inst_valid;
reg  [31:0] fs_pc;
assign fs_to_ds_bus = {fs_inst ,
                       fs_pc   };

br_fsm br_fsm(//branch指令状态机
    .clk(clk),
    .reset(reset),
    .flush(ex_flush | eret_flush) ,
    .br_taken(br_taken),
    .br_target(br_target),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .br_taken_r(br_taken_r),
    .br_target_r(br_target_r)
);

//exception
wire        fs_adel;

assign fs_adel      = fs_pc[1:0]!=2'b00;
assign fs_ex_bus    = {fs_adel,fs_pc};

// pre-IF stage
assign to_fs_ready_go = inst_sram_req && inst_sram_addr_ok;
assign to_fs_valid    = ~reset & to_fs_ready_go;
assign seq_pc         = fs_pc + 3'h4;//pc+4
assign nextpc         = ex_flush===1'b1   ? 32'hbfc00380 :
                        eret_flush===1'b1 ? eret_pc      :
                        br_taken_r===1'b1 ? br_target_r  :
                        seq_pc; 

fetch_fsm fetch_fsm(
    .clk            (clk)                   ,
    .reset          (reset)                 ,
    .flush          (ex_flush | eret_flush) ,
    .nextpc         (nextpc)                , 
    .ready_go       (!fs_valid && ds_allowin) ,
    .inst_sram_addr_ok (inst_sram_addr_ok)  ,
    .inst_sram_data_ok (inst_sram_data_ok)  ,
    .inst_sram_rdata(inst_sram_rdata)       ,
    .inst_sram_req  (inst_sram_req)         ,
    .inst_sram_addr (inst_sram_addr)        ,
    .fs_inst        (fs_inst)               ,
    .fs_inst_valid  (fs_inst_valid)
);

// IF stage
assign fs_ready_go    = fs_inst_valid;
assign fs_allowin     = !fs_valid || fs_ready_go && ds_allowin;//接收使能信号，在无合法数据或者流水线正常运转时有效
assign fs_to_ds_valid = fs_valid && fs_ready_go;//数据传递有效信号
always @(posedge clk) begin
    if (reset) begin//复位
        fs_valid <= 1'b0;
    end
    else if (fs_allowin || ex_flush || eret_flush) begin//可接受新数据，注意发生异常时将状态强制刷新，变更为异常入口pc取指状态
        fs_valid <= to_fs_valid;
    end

    if (reset) begin
        fs_pc <= 32'hbfbffffc;  //trick: to make nextpc be 0xbfc00000 during reset 
    end
    else if (to_fs_valid && (fs_allowin || ex_flush || eret_flush)) begin//保证和valid信号同拍更新
        fs_pc <= inst_sram_addr;
    end
end

assign inst_sram_wr    = 1'b0;
assign inst_sram_size  = 2'b10;
assign inst_sram_wstrb = 4'h0;
assign inst_sram_wdata = 32'b0;

endmodule
