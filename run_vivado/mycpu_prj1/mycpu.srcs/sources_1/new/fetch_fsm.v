

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/08/18 21:07:03
// Design Name: 
// Module Name: fetch_fsm
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


module fetch_fsm(
input         clk               ,
input         reset             ,
input         flush             ,
input  [31:0] nextpc            ,
input         ready_go          ,
input  [31:0] inst_sram_rdata   ,
input         inst_sram_addr_ok ,
input         inst_sram_data_ok ,
output        inst_sram_req     ,
output [31:0] inst_sram_addr    ,
output [31:0] fs_inst           ,
output        fs_inst_valid
    );
    localparam IDLE = 0, REQ=1, READ=2, CANCEL=3;
    reg [ 1:0] state,next_state;
    wire [31:0] req_addr;
    reg [31:0] data;
    reg        data_valid;

    always @(posedge clk) begin
        if(reset) state <= IDLE;
        else      state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:begin
                if(ready_go)          next_state = REQ;
                else                  next_state = IDLE;    
            end
            REQ:begin
                if(inst_sram_addr_ok) next_state = READ;
                else                  next_state = REQ;    
            end
            READ:begin
                if(inst_sram_data_ok) begin
                    if(flush) next_state = REQ;
                    else      next_state = IDLE;
                end
                else if(flush)        next_state = CANCEL;
                else                  next_state = READ;
            end
            CANCEL:begin
                if(inst_sram_data_ok) next_state = REQ;
                else                  next_state = CANCEL;    
            end
        endcase
    end

    assign req_addr = flush===1'b1 ? nextpc :
                      state===IDLE && ready_go===1'b1 ? nextpc :
                      req_addr;

    always @(*) begin
        if(next_state==IDLE) begin
            if(inst_sram_data_ok) begin
                data_valid = 1;
                data       = inst_sram_rdata;
            end
            //latch
        end
        else begin
            data_valid     = 0;
        end
    end

    assign inst_sram_req  = state == REQ;
    assign inst_sram_addr = req_addr;
    assign fs_inst        = data;
    assign fs_inst_valid  = data_valid;
endmodule
