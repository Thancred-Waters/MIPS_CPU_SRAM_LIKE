`include "mycpu.h"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //stall
    output                         ms_write_reg  ,
    output [4:0]                   ms_reg_dest   ,
    output                         ms_mfc0_stall ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    input  [6:0]                   es_load_mem_bus,
    input  [`ES_EX_BUS_WD    -1:0] es_ex_bus     ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    output [`MS_EX_BUS_WD    -1:0] ms_ex_bus     ,
    //from cp0
    input                          flush         ,
    //from data-sram
    input  [31                 :0] data_sram_rdata,
    //forward: to ds
    output [35:0]                  ms_to_ds_bus  ,
    //eret
    output                         ms_ex
);

reg         ms_valid;
wire        ms_ready_go;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
reg [`ES_EX_BUS_WD    -1:0] es_ex_bus_r;
reg [6:0] es_load_mem_bus_r;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
assign {ms_res_from_mem,  //70:70 是否来自内存
        ms_gr_we       ,  //69:69 寄存器写使能
        ms_dest        ,  //68:64 目标寄存器
        ms_alu_result  ,  //63:32 运算结果
        ms_pc             //31:0 pc
       } = es_to_ms_bus_r;

//exception
wire        ms_bd;
wire        ms_sys;
wire        ms_mtc0;
wire        ms_eret;
wire        ms_break;
wire        ms_over_flow;
wire        ms_adel;
wire        ms_ades;
wire        ms_ri;
wire [ 4:0] ms_addr;
wire [31:0] ms_badvaddr;
assign {ms_bd        ,
        ms_sys       ,
        ms_mfc0      ,
        ms_mtc0      ,
        ms_eret      ,
        ms_break     ,
        ms_over_flow ,
        ms_adel      ,
        ms_ades      ,
        ms_ri        ,
        ms_addr      ,
        ms_badvaddr
       } = es_ex_bus_r; 
assign ms_ex = (ms_eret | ms_sys | ms_break | ms_over_flow | ms_adel | ms_ades | ms_ri) && ms_valid;
assign ms_mfc0_stall = ms_mfc0 && ms_valid;
assign ms_ex_bus = {ms_bd        ,
                    ms_sys       ,
                    ms_mfc0      ,
                    ms_mtc0      ,
                    ms_eret      ,
                    ms_break     ,
                    ms_over_flow ,
                    ms_adel      , 
                    ms_ades      ,
                    ms_ri        ,
                    ms_addr      ,
                    ms_badvaddr
                    };

//load-type inst
wire [1:0] load_width;
wire       load_sign;
wire [1:0] load_lr;
wire [1:0] load_offset;
wire [3:0] load_we_l;
wire [3:0] load_we_r;
wire [3:0] reg_we;
assign {load_width,
        load_sign ,
        load_lr   ,
        load_offset
       } = es_load_mem_bus_r;
assign load_we_l = {4{load_lr[1] && load_offset==2'b00}} & 4'b1000
                 | {4{load_lr[1] && load_offset==2'b01}} & 4'b1100
                 | {4{load_lr[1] && load_offset==2'b10}} & 4'b1110
                 | {4{load_lr[1] && load_offset==2'b11}} & 4'b1111;
assign load_we_r = {4{load_lr[0] && load_offset==2'b00}} & 4'b1111
                 | {4{load_lr[0] && load_offset==2'b01}} & 4'b0111
                 | {4{load_lr[0] && load_offset==2'b10}} & 4'b0011
                 | {4{load_lr[0] && load_offset==2'b11}} & 4'b0001;
assign reg_we    = load_we_l | load_we_r | {4{load_lr==2'b00 && ms_gr_we}};
//load-type inst

//判断是否存在寄存器冲突
assign ms_write_reg=ms_gr_we && ms_to_ws_valid;
assign ms_reg_dest=ms_dest;

wire [31:0] mem_result;
wire [15:0] mem_half_result;
wire [7 :0] mem_byte_result;
wire [31:0] mem_word_result;
wire [63:0] tmp;
wire [4 :0] index,index_8;
wire [31:0] mem_lr_result;//lwl lwr
wire [31:0] ms_final_result;

assign ms_to_ws_bus = {reg_we        ,  //72:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };

assign ms_ready_go    = 1'b1;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (flush) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end

    if(flush) begin
        es_to_ms_bus_r    <= 0;
        es_load_mem_bus_r <= 0;
        es_ex_bus_r       <= 0;
    end
    if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r    <= es_to_ms_bus;
        es_load_mem_bus_r <= es_load_mem_bus;
        es_ex_bus_r       <= es_ex_bus;
    end
end

assign tmp             = {data_sram_rdata,data_sram_rdata};
assign index           = {load_offset,3'b0};
assign index_8           = index+8;
assign mem_lr_result   = {32{load_lr[1]}} & tmp[index_8+:32]
                       | {32{load_lr[0]}} & tmp[index+:32];
assign mem_word_result = {32{load_lr==2'b00}} & data_sram_rdata
                       | {32{load_lr!=2'b00}} & mem_lr_result;
assign mem_half_result = data_sram_rdata[16*load_offset[1]+:16];
assign mem_byte_result = data_sram_rdata[8*load_offset+:8];

assign mem_result = {32{load_width==2'b01}} & {{24{load_sign & mem_byte_result[7]}},mem_byte_result}
                  | {32{load_width==2'b10}} & {{16{load_sign & mem_half_result[15]}},mem_half_result}
                  | {32{load_width==2'b11}} & mem_word_result;

assign ms_final_result = ms_res_from_mem ? mem_result
                                         : ms_alu_result;

assign ms_to_ds_bus = {ms_final_result,reg_we};

endmodule
