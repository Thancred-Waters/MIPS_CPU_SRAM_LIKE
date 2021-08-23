`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/08/19 12:32:40
// Design Name: 
// Module Name: mem_fsm
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


module mem_fsm(
input             clk,
input             reset,
input             op_load,
input             op_save,
input             addr_ok,
input             data_ok,
input      [31:0] data_sram_rdata,
output            data_sram_req,
output reg        data_valid,
output reg [31:0] data
    );
    localparam IDLE=0, REQ=1, WORKING=2;
    reg [ 1:0] state,next_state;
    
    always @(posedge clk) begin
        if(reset) state <= IDLE;
        else      state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:begin
                if(op_load | op_save) next_state=REQ;
                else                  next_state=IDLE;    
            end
            REQ:begin
                if(addr_ok)           next_state=WORKING;
                else                  next_state=REQ;
            end
            WORKING:begin
                if(data_ok)           next_state=IDLE;
                else                  next_state=WORKING;
            end
        endcase
    end

    always @(*) begin
        if(next_state==IDLE) begin
            if(data_ok) begin
                data_valid=1;
                data=data_sram_rdata;
            end
        end
        else begin
            data_valid=0;
        end    
    end

    assign data_sram_req = state == REQ;
endmodule
