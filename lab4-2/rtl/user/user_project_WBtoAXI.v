`default_nettype none

module WB2AXI #(
    parameter BITS = 32,
    parameter DELAYS = 5,
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32
)(
    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,         //0 is read
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output reg [31:0] wbs_dat_o,

    //NEW
    input axi_valid,

    //AXI_LITE
    input                            awready,
    input                            wready,
    output reg                       awvalid,
    output reg [(pADDR_WIDTH-1): 0]  awaddr,
    output reg                       wvalid,
    output reg [(pDATA_WIDTH-1) : 0] wdata,

    input                            arready,
    input                            rvalid,
    output reg                       rready,
    output reg                       arvalid,
    output reg [(pADDR_WIDTH-1): 0]  araddr,
    input      [(pDATA_WIDTH-1): 0]  rdata,

    //SS
    output reg                       ss_tvalid,
    output reg [(pDATA_WIDTH-1) : 0] ss_tdata,
    output reg                       ss_tlast,
    input                            ss_tready,

    //SM
    output reg                       sm_tready,
    input                            sm_tvalid,
    input [(pDATA_WIDTH-1) : 0]      sm_tdata,
    input                            sm_tlast    
);

//================================================================
    //old
    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;
    wire wb_rst_n;
    reg ready;
    reg [BITS-17:0] delayed_count;

    //NEW
    reg [31:0] addr_temp;
    reg lite_flag, ss_flag, sm_flag;
    reg  lite_write_state, lite_write_next;
    reg lite_read_state, lite_read_next;
    reg sm_state, sm_next;
    reg ss_state, ss_next;
    // reg [31:0] outdata, outdata_reg;
    reg output_mux_sel;
    // reg sm_reg_en, liter_reg_en;
    reg sm_tready_reg;

//=================================================================
    //rst
    assign wb_rst_n = ~wb_rst_i;
    // decide to respond WB cycle   //
    assign valid = wbs_cyc_i && wbs_stb_i && axi_valid; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};           //0 is read, 1 is write
    assign wbs_ack_o = ready;

    //counter count what time respond ack to WB//
    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if (!wb_rst_n) begin
            ready <= 1'b0;
            delayed_count <= 16'b0;
        end else begin
            ready <= 1'b0;
            if ( valid && !ready ) begin
                if ( delayed_count == DELAYS ) begin
                    delayed_count <= 16'b0;
                    ready <= 1'b1;
                end else begin
                    delayed_count <= delayed_count + 1;
                end
            end
        end
    end

    //decode addr to decide which AXI tpye //
    always @(*) begin
        awaddr = wbs_adr_i[11:0];
        araddr = wbs_adr_i[11:0];
    end

    always @(*) begin
        if(valid && wbs_adr_i[7:0] >= 8'h00 && wbs_adr_i[7:0] < 8'h7F) begin
            lite_flag = 1;
            ss_flag = 0;
            sm_flag = 0;
        end
        else if(valid && wbs_adr_i[7:0] == 8'h80) begin
            lite_flag = 0;
            ss_flag = 1;
            sm_flag = 0;            
        end
        else if(valid && wbs_adr_i[7:0] == 8'h84) begin
            lite_flag = 0;
            ss_flag = 0;
            sm_flag = 1;
        end
        else begin
            lite_flag = 0;
            ss_flag = 0;
            sm_flag = 0;
        end
    end
    //data_i & data_o//
    always @(*) begin
        wdata = wbs_dat_i;
        ss_tdata = wbs_dat_i;
    end

    always @(*) begin   //output mux
        if(lite_flag && !wstrb) begin
            wbs_dat_o = rdata; 
        end
        else if(sm_flag) begin
            wbs_dat_o = sm_tdata;
        end
        else begin
            wbs_dat_o = 32'd0;
        end
    end

    // always @(posedge wb_clk_i, negedge wb_rst_n) begin
    //     if(!wb_rst_n)
    //         outdata <= 32'd0;
    //     else    
    //         if(sm_reg_en | liter_reg_en)
    //             outdata <= outdata_reg;
    //         else
    //             outdata <= 32'd0;
    // end    

    //AXI cycle//
    //counter = 1 is mean that WB cycle is accept

    //Lite_write
    always @(*) begin
        case (lite_write_state)
            1'b0: begin
                if(lite_flag && wstrb && delayed_count[3:0] == 4'd1) begin 
                    awvalid = 1;
                    wvalid = 1;
                    lite_write_next = 1'b1;
                end
                else begin
                    awvalid = 0;
                    wvalid = 0;                    
                    lite_write_next = 1'b0;
                end
            end
            1'b1: begin
                if(wready) begin
                    awvalid = 1;
                    wvalid = 1;   
                    lite_write_next = 1'b0;     
                end
                else begin
                    awvalid = 1;
                    wvalid = 1;       
                    lite_write_next = 1'b1;         
                end
            end
        endcase
    end

    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if(!wb_rst_n)
            lite_write_state <= 0;
        else    
            lite_write_state <= lite_write_next;
    end         

    //Lite_read
    always @(*) begin
        case (lite_read_state)
            1'b0: begin
                if(lite_flag && !wstrb && delayed_count[3:0] == 4'd1) begin
                    arvalid = 1;
                    rready = 1;
                    lite_read_next = 1'b1;
                end
                else begin
                    arvalid = 0;
                    rready = 0;                    
                    lite_read_next = 1'b0;
                end
            end
            1'b1: begin
                if(rvalid) begin
                    arvalid = 1;
                    rready = 1;   
                    lite_read_next = 1'b0;     
                end
                else begin
                    arvalid = 1;
                    rready = 1;       
                    lite_read_next = 1'b1;         
                end
            end
        endcase
    end

    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if(!wb_rst_n)
            lite_read_state <= 0;
        else    
            lite_read_state <= lite_read_next;
    end

    //SS
    always @(*) begin
        case (ss_state)
            1'b0: begin
                if(ss_flag && wstrb &&(delayed_count[3:0] == 4'd1)) begin
                    ss_tvalid = 0;
                    ss_next = 1'b1;
                end
                else begin
                    ss_tvalid = 0;
                    ss_next = 1'b0;
                end
            end
            1'b1: begin
                if(ss_tready) begin
                    ss_tvalid = 1;
                    ss_next = 1'b0;     
                end
                else begin
                    ss_tvalid = 1; 
                    ss_next = 1'b1;         
                end
            end
        endcase
    end

    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if(!wb_rst_n)
            ss_state <= 0;
        else    
            ss_state <= ss_next;
    end

    //SM
    always @(*) begin
        case (sm_state)
            1'b0: begin
                if(sm_flag && !wstrb &&(delayed_count[3:0] == 4'd1) ) begin
                    sm_tready_reg = 0;
                    sm_next = 1'b1;
                end
                else begin
                    sm_tready_reg = 0;
                    sm_next = 1'b0;
                end
            end
            1'b1: begin
                if(sm_tvalid) begin
                    sm_tready_reg = 1;
                    sm_next = 1'b0;     
                end
                else begin
                    sm_tready_reg = 0; 
                    sm_next = 1'b1;         
                end
            end
        endcase
    end    
    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if(!wb_rst_n)
            sm_tready <= 0;
        else    
            sm_tready <= sm_tready_reg;
    end 

    always @(posedge wb_clk_i, negedge wb_rst_n) begin
        if(!wb_rst_n)
            sm_state <= 0;
        else    
            sm_state <= sm_next;
    end    

endmodule

`default_nettype wire