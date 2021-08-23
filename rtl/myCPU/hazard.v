`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/08/07 21:19:01
// Design Name: 
// Module Name: hazard
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module hazard(
//input
input        ds_use_rs,
input [4:0]  rs_addr,
input        ds_use_rt,
input [4:0]  rt_addr,
input        es_write_reg,
input [4:0]  es_reg_dest,
input        es_read_mem,//exe阶段指令是否需要读内存
input        es_mfc0,
input        alu_stall,
input        ms_write_reg,
input [4:0]  ms_reg_dest,
input        ms_mfc0,
input        ws_write_reg,
input [4:0]  ws_reg_dest,
//exception
input        es_ex,
input        ms_ex,
input        ws_ex,
//output
output       stallD,
output       stallE,
output [1:0] forward_rs,
output [1:0] forward_rt
    );
    //forward
    wire ws_forward_rs,ws_forward_rt;
    assign ws_forward_rs=(ds_use_rs===1'b1 && rs_addr!==5'b0) && 
                         (ws_write_reg===1'b1 && rs_addr===ws_reg_dest);
    assign ws_forward_rt=(ds_use_rt===1'b1 && rt_addr!==5'b0) &&
                         (ws_write_reg===1'b1 && rt_addr===ws_reg_dest);

    wire ms_forward_rs,ms_forward_rt;
    assign ms_forward_rs=(ds_use_rs===1'b1 && rs_addr!==5'b0 && ms_mfc0==1'b0) &&
                         (ms_write_reg===1'b1 && rs_addr===ms_reg_dest);
    assign ms_forward_rt=(ds_use_rt===1'b1 && rt_addr!==5'b0 && ms_mfc0==1'b0) && 
                         (ms_write_reg===1'b1 && rt_addr===ms_reg_dest);

    wire es_forward_rs,es_forward_rt;
    assign es_forward_rs=(ds_use_rs===1'b1 && rs_addr!==5'b0 && es_mfc0===1'b0) &&
                         (es_write_reg===1'b1 && es_read_mem===1'b0 && rs_addr===es_reg_dest);
    assign es_forward_rt=(ds_use_rt===1'b1 && rt_addr!==5'b0 && es_mfc0===1'b0) &&
                         (es_write_reg===1'b1 && es_read_mem===1'b0 && rt_addr===es_reg_dest);                    


    assign forward_rs=es_forward_rs ? 2'b11 ://优先选择器，exe阶段数据最新，前推优先级最高
                      ms_forward_rs ? 2'b10 :
                      ws_forward_rs ? 2'b01 :
                      2'b00;
    assign forward_rt=es_forward_rt ? 2'b11 :
                      ms_forward_rt ? 2'b10 :
                      ws_forward_rt ? 2'b01 :
                      2'b00;

    //data stall
    wire rs_stall,rt_stall;
    assign rs_stall=(ds_use_rs===1'b1 && rs_addr!==5'b0) &&
                    (es_read_mem===1'b1 && rs_addr===es_reg_dest
                    || es_mfc0===1'b1 && rs_addr===es_reg_dest
                    || ms_mfc0===1'b1 && rs_addr===ms_reg_dest);
    assign rt_stall=(ds_use_rt===1'b1 && rt_addr!==5'b0) &&
                    (es_read_mem===1'b1 && rt_addr===es_reg_dest
                    || es_mfc0===1'b1 && rt_addr===es_reg_dest
                    || ms_mfc0===1'b1 && rt_addr===ms_reg_dest);
    assign stallE=~(es_ex | ms_ex | ws_ex) & alu_stall;
    assign stallD=~(es_ex | ms_ex | ws_ex) & (rs_stall | rt_stall);//流水线中有异常时，不阻塞流水线
endmodule
