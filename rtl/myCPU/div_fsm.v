`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/08/10 22:13:33
// Design Name: 
// Module Name: div_fsm
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


module div_fsm(
input clk,
input reset,
input flush,
input [31:0] src1,
input [31:0] src2,
input data_valid,
output [63:0] div_out,
output res_valid
    );
    localparam IDLE=0,Div_Busy=1,Div_Working=2;

    reg [1:0] state,next_state;
    wire src1_ready,src2_ready;
    reg div_valid;

    always @(posedge clk) begin
        if(reset) state<=IDLE;
        else if(flush) state<=IDLE;
        else state<=next_state;
    end

    always @(*) begin
        case(state)
            IDLE:begin
                if(data_valid) next_state=Div_Busy;
                else next_state=IDLE;    
            end
            Div_Busy:begin
                if(src1_ready & src2_ready) next_state=Div_Working;
                else next_state=Div_Busy;
            end
            Div_Working:begin
                if(res_valid) next_state=IDLE;
                else next_state=Div_Working;    
            end
        endcase    
    end

    always @(posedge clk) begin
        case(state)
            IDLE:div_valid<=data_valid;
            Div_Busy:div_valid<=src1_ready & src2_ready ? 1'b0:div_valid;
            Div_Working:div_valid<=1'b0;
        endcase
    end

    mydiv div(
        .aclk(clk),                                      // input wire aclk
        .s_axis_divisor_tvalid(div_valid),    // input wire s_axis_divisor_tvalid
        .s_axis_divisor_tready(src2_ready),    // output wire s_axis_divisor_tready
        .s_axis_divisor_tdata(src2),      // input wire [31 : 0] s_axis_divisor_tdata
        .s_axis_dividend_tvalid(div_valid),  // input wire s_axis_dividend_tvalid
        .s_axis_dividend_tready(src1_ready),  // output wire s_axis_dividend_tready
        .s_axis_dividend_tdata(src1),    // input wire [31 : 0] s_axis_dividend_tdata
        .m_axis_dout_tvalid(res_valid),          // output wire m_axis_dout_tvalid
        .m_axis_dout_tdata(div_out)            // output wire [63 : 0] m_axis_dout_tdata
    );
endmodule
