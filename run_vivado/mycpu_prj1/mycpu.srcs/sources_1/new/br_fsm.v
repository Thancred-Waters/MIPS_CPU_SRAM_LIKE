`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/08/22 09:58:26
// Design Name: 
// Module Name: br_fsm
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


module br_fsm(
input clk,
input reset,
input flush,
input br_taken,
input [31:0] br_target,
input inst_sram_addr_ok,
input inst_sram_data_ok,
output br_taken_r,
output [31:0] br_target_r
    );
    localparam IDLE = 0, WA1=1, WA2=2, DONE=3;
    reg [ 1:0] state,next_state;
    reg [31:0] br_addr;
    reg        br_valid;

    always @(posedge clk) begin
        if(reset)      state <= IDLE;
        else if(flush) state <= IDLE;
        else           state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:begin
                if(~br_taken) next_state=IDLE;
                else if(inst_sram_addr_ok) next_state=WA2;
                else next_state=WA1;
            end
            WA1:begin
                if(inst_sram_addr_ok) next_state=WA2;
                else next_state=WA1;
            end
            WA2:begin
                if(inst_sram_data_ok) next_state=DONE;
                else next_state=WA2;
            end
            DONE:begin
                if(inst_sram_addr_ok) next_state=IDLE;
                else next_state=DONE;
            end
        endcase
    end

    always @(posedge clk) begin
        if(state==IDLE && br_taken)
            br_addr <= br_target;
    end

    always @(posedge clk) begin
        if(next_state==DONE && !flush)
            br_valid <= 1;
        else
            br_valid <= 0; 
    end

    assign br_target_r = br_addr;
    assign br_taken_r  = br_valid;
endmodule
