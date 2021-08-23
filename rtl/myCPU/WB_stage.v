`include "mycpu.h"

module wb_stage(
    input                           clk           ,
    input                           reset         ,
    //stall
    output                          ws_write_reg  ,
    output [4:0]                    ws_reg_dest   ,
    //allowin
    output                          ws_allowin    ,
    //from ms
    input                           ms_to_ws_valid,
    input  [`MS_TO_WS_BUS_WD -1:0]  ms_to_ws_bus  ,
    input  [`MS_EX_BUS_WD    -1:0]  ms_ex_bus     ,
    //to rf: for write back
    output [`WS_TO_RF_BUS_WD -1:0]  ws_to_rf_bus  ,
    //to cp0
    output  [ 8:0] c0_exception,
    output  [ 4:0] c0_addr,
    output  [31:0] c0_wdata,
    output         c0_wb_valid,
    output         c0_wb_bd,
    output  [31:0] c0_wb_pc,
    output  [31:0] ws_badvaddr,
    //from cp0
    input          c0_valid,
    input [31:0]   c0_res,
    input          flush,
    //trace debug interface
    output [31:0] debug_wb_pc     ,
    output [ 3:0] debug_wb_rf_wen ,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata,
    //eret
    output                          ws_ex
);

reg         ws_valid;
wire        ws_ready_go;

reg [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus_r;
reg [`MS_EX_BUS_WD    -1:0] ms_ex_bus_r;
wire [ 3:0] ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
assign {ws_gr_we       ,  //72:69
        ws_dest        ,  //68:64
        ws_final_result,  //63:32
        ws_pc             //31:0
       } = ms_to_ws_bus_r;

//exception
wire        ws_bd;
wire        ws_sys;
wire        ws_mfc0;
wire        ws_mtc0;
wire        ws_eret;
wire        ws_break;
wire        ws_over_flow;
wire        ws_adel;
wire        ws_ades;
wire        ws_ri;
wire [ 4:0] ws_addr;
assign {ws_bd        ,
        ws_sys       ,
        ws_mfc0      ,
        ws_mtc0      ,
        ws_eret      ,
        ws_break     ,
        ws_over_flow ,
        ws_adel      ,
        ws_ades      ,
        ws_ri        ,
        ws_addr      ,
        ws_badvaddr
       } = ms_ex_bus_r;
reg is_branch;
always @(posedge clk) begin
    if(reset) begin
        is_branch<=0;
    end
    else if(ms_to_ws_valid && ws_allowin) begin
        if(is_branch) is_branch<=0;
        else is_branch<=ws_bd;
    end
end
assign ws_ex        = (ws_eret | ws_sys | ws_break | ws_over_flow | ws_adel | ws_ades | ws_ri) && ws_valid;
assign c0_exception = {ws_sys,ws_mfc0,ws_mtc0,ws_eret,ws_break,ws_over_flow,ws_adel,ws_ades,ws_ri};
assign c0_addr      = ws_addr;
assign c0_wdata     = ws_final_result;
assign c0_wb_valid  = ws_valid;
assign c0_wb_bd     = is_branch;
assign c0_wb_pc     = ws_pc;

//判断是否存在寄存器冲突
assign ws_write_reg=(ws_gr_we!=4'h0) && ws_valid;
assign ws_reg_dest=ws_dest;

wire [3 :0] rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;
assign ws_to_rf_bus = {rf_we   ,  //40:37
                       rf_waddr     ,  //36:32
                       rf_wdata        //31:0
                      };

assign ws_ready_go = 1'b1;
assign ws_allowin  = !ws_valid || ws_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ws_valid <= 1'b0;
    end
    else if (flush) begin
        ws_valid <= 1'b0;
    end
    else if (ws_allowin) begin
        ws_valid <= ms_to_ws_valid;
    end

    if(flush) begin
        ms_to_ws_bus_r <= 0;
        ms_ex_bus_r    <= 0;
    end
    if (ms_to_ws_valid && ws_allowin) begin
        ms_to_ws_bus_r <= ms_to_ws_bus;
        ms_ex_bus_r    <= ms_ex_bus;
    end
end

assign rf_we    = ws_gr_we & {4{ws_valid & ~ws_ex}};
assign rf_waddr = ws_dest;
assign rf_wdata = c0_valid ? c0_res : ws_final_result;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_wen   = rf_we;
assign debug_wb_rf_wnum  = ws_dest;
assign debug_wb_rf_wdata = c0_valid ? c0_res : ws_final_result;

endmodule
