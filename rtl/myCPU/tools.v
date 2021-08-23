module decoder_5_32(
    input  [ 4:0] in,
    output [31:0] out
);

genvar i;
generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_6_64(
    input  [ 5:0] in,
    output [63:0] out
);

genvar i;
generate for (i=0; i<64; i=i+1) begin : gen_for_dec_6_64
    assign out[i] = (in == i);
end endgenerate

endmodule

module forward_data(
    input  [31:0] a,
    input  [31:0] b,
    input  [ 3:0] we,
    output [31:0] out
);
    assign out[7:0]   = we[0]===1'b1 ? a[7:0]   : b[7:0];
    assign out[15:8]  = we[1]===1'b1 ? a[15:8]  : b[15:8];
    assign out[23:16] = we[2]===1'b1 ? a[23:16] : b[23:16];
    assign out[31:24] = we[3]===1'b1 ? a[31:24] : b[31:24];
endmodule

