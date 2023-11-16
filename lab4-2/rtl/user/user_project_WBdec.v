
`default_nettype none
module WBdecoder (
    input [31:0] wbs_adr_i,
    output reg  bram_valid, axi_valid,
    input [31:0] wbs_dat_o_axi, wbs_dat_o_bram,
    input wbs_ack_o_axi, wbs_ack_o_bram,
    output reg [31:0] wbs_dat_o,
    output reg wbs_ack_o
);

    always @(*) begin
        if(wbs_adr_i[31:20] == 12'h380) begin
            bram_valid = 1;
            axi_valid = 0;
        end
        else if(wbs_adr_i[31:20] == 12'h300) begin
            bram_valid = 0;
            axi_valid = 1;           
        end
        else begin
            bram_valid = 0;
            axi_valid = 0;             
        end
    end

    always @(*) begin
        case({bram_valid, axi_valid}) 
            2'b01: {wbs_ack_o, wbs_dat_o} = {wbs_ack_o_axi, wbs_dat_o_axi};
            2'b10: {wbs_ack_o, wbs_dat_o} = {wbs_ack_o_bram, wbs_dat_o_bram};
            default: {wbs_ack_o, wbs_dat_o} = {1'b0, 32'd0};
        endcase
    end


endmodule
`default_nettype none