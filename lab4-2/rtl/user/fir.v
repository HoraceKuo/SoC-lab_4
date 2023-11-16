//`timescale 1ns / 1ns
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //AXI_lite
    output  reg                      awready,
    output  reg                      wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  reg                      arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                      rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata,    

    //AXI_stream_slave
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 

    //AXI_stream_master
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                      sm_tlast, 
    
    // bram for tap RAM
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    //system
    input   wire                     axis_clk,
    input   wire                     axis_rst_i

);


//======================================//
//============AXI_LITE_MODULE===========//
//======================================//

    //parameter
    parameter   ADDR_BLOCK = 12'h00,
                ADDR_LEN = 12'h10,
                ADDR_TAP_MIN = 12'h40,          //lab3:12'h20
                ADDR_TAP_MAX = 12'h68;          //lab3:12'h48

    parameter   WC_AWVALID = 3'b000,
                WC_AWREADY = 3'b001,
                WC_WVALID = 3'b010,
                WC_WREADY = 3'b011,
                WC_BACK = 3'b100;

    parameter   RC_ARVALID = 3'b000,
                RC_ARREADY = 3'b001,
                RC_RVALID = 3'b010,
                RC_RREADY = 3'b011,
                RC_BACK = 3'b100;

    wire axis_rst_n;
    //variable write
    reg [11:0] write_addr_reg_out;
    reg write_addr_reg_en;
    reg [31:0] write_data_reg_out;
    reg write_data_reg_en;
    reg [11:0] write_addr_count_out;
    reg w_compa_wt, w_compa_wc;
    reg awready_reg, wready_reg, wskip, wskip_reg; //move wskip
    reg rst_write_addr_reg, fir_rst_len;
    reg [2:0] wc_state, wc_state_reg;

    //variable read//
    reg [11:0] read_addr_reg_out;
    reg read_addr_reg_en;
    reg [31:0] read_data_reg_in;
    reg read_data_reg_en;    
    reg [11:0] read_addr_count_out;
    reg rvalid_reg, rvalid_reg_en, arready_reg, rskip, rskip_reg;
    reg r_compa_wt, r_compa_wc;
    reg [1:0] read_mux_en;
    reg rst_read_addr_reg;
    reg r_waitd_reg, r_waitd;
    reg [2:0] rc_state, rc_state_reg;

    //variable togerther
    reg [31:0] len_reg_out;
    reg len_reg_en;
    reg ap_start_reg_out, ap_done_reg_out, ap_idle_reg_out;
    reg block_pro_reg_en;
    reg [1:0]  WorR_mux_en;     //00:close to access bram, 01:write, 10:read

    //tap controller variable 
    reg [3:0] tap_control_state, tap_control_next;
    reg wt, wc;       //wt=write_tap(check write tap bram), t_EN=tap_EN
    reg [11:0] t_A;
    reg [11:0] axi_tap_A;
    reg [3:0] axi_tap_WE;
    reg axi_tap_EN;

    //FIR
    reg fir_w_ap_idle, fir_w_ap_done, block_pro_idle_reg_en, block_pro_done_reg_en;
    reg ss_tready_reg;

    //new
    reg ap_Xn_reg_out, ap_Xn_reg, ap_Xn_reg_en, ap_Yn_reg_out, ap_Yn_reg, ap_Yn_reg_en, ap_res_reg_out, ap_res_reg, ap_res_reg_en;

//======================================
assign axis_rst_n = ~axis_rst_i;
//=======write_channel=======
//write_FSM//

    always @(*) begin

        rst_write_addr_reg = 0;
        case(wc_state)
            WC_AWVALID: begin
                if(awvalid) begin   //5'b10000
                    awready_reg = 1; 
                    wready_reg = 0;
                    WorR_mux_en = 2'b01;
                    write_addr_reg_en = 1;
                    write_data_reg_en = 0;
                    wskip_reg = 0;

                    wc_state_reg = WC_AWREADY;
                end
                else begin
                    awready_reg = 0;
                    wready_reg = 0;
                    WorR_mux_en = 2'b00;  
                    write_addr_reg_en = 0;
                    write_data_reg_en = 0;  
                    wskip_reg = 0;    
                    rst_write_addr_reg = 1;  

                    wc_state_reg = WC_AWVALID;
                end
            end
            WC_AWREADY: begin
                if(awvalid && awready && !wskip) begin   //5'b11000
                    awready_reg = 0; 
                    wready_reg = 0;
                    WorR_mux_en = 2'b01;
                    write_addr_reg_en = 0;
                    write_data_reg_en = 0;           

                    wc_state_reg = WC_WVALID;       
                end
                else begin
                    wc_state_reg = WC_AWREADY;
                end
            end
            WC_WVALID: begin        
                if(wvalid && !wskip) begin    //5'b00100
                    awready_reg = 0; 
                    wready_reg = 1;
                    WorR_mux_en = 2'b01;
                    write_addr_reg_en = 0;
                    write_data_reg_en = 1;  

                    wc_state_reg = WC_WREADY;           
                end
                else begin
                    wc_state_reg = WC_WVALID;
                end
            end
            WC_WREADY: begin
                if(wvalid && wready && !wskip) begin  //5'b00110
                    awready_reg = 0; 
                    wready_reg = 0;
                    WorR_mux_en = 2'b01;
                    write_addr_reg_en = 0;
                    write_data_reg_en = 0;
                    wskip_reg = 1;    

                    wc_state_reg = WC_BACK;                  
                end
                else begin
                    wc_state_reg = WC_WREADY;
                end
            end
            WC_BACK: begin
                if(wskip) begin    //5'b?0001
                    awready_reg = 0;
                    wready_reg = 0;
                    WorR_mux_en = 2'b00;  
                    write_addr_reg_en = 0;
                    write_data_reg_en = 0;  
                    wskip_reg = 0;    
                    rst_write_addr_reg = 1;  

                    wc_state_reg = WC_AWVALID;                    
                end
                else begin
                    wc_state_reg = WC_BACK;
                end
            end
            default: begin
                awready_reg = 0;
                wready_reg = 0;
                WorR_mux_en = 2'b00;  
                write_addr_reg_en = 0;
                write_data_reg_en = 0;  
                wskip_reg = 0;  

                wc_state_reg = WC_AWVALID;                
            end

        endcase
    end

    // always @(*) begin

    //     rst_write_addr_reg = 0;
    //     casez ({awvalid, awready, wvalid, wready, wskip})
    //         5'b10000: begin
    //             awready_reg = 1; 
    //             wready_reg = 0;
    //             WorR_mux_en = 2'b01;
    //             write_addr_reg_en = 1;
    //             write_data_reg_en = 0;
    //             wskip_reg = 0;
    //         end
    //         5'b11000: begin
    //             awready_reg = 0; 
    //             wready_reg = 0;
    //             WorR_mux_en = 2'b01;
    //             write_addr_reg_en = 0;
    //             write_data_reg_en = 0;
    //         end
    //         5'b00100: begin
    //             awready_reg = 0; 
    //             wready_reg = 1;
    //             WorR_mux_en = 2'b01;
    //             write_addr_reg_en = 0;
    //             write_data_reg_en = 1;
    //         end
    //         5'b00110: begin
    //             awready_reg = 0; 
    //             wready_reg = 0;
    //             WorR_mux_en = 2'b01;
    //             write_addr_reg_en = 0;
    //             write_data_reg_en = 0;
    //             wskip_reg = 1;
    //         end

    //         5'b?0001: begin          //reset write_or_read_addr_MUX
    //             awready_reg = 0;
    //             wready_reg = 0;
    //             WorR_mux_en = 2'b00;  
    //             write_addr_reg_en = 0;
    //             write_data_reg_en = 0;  
    //             wskip_reg = 0;    
    //             rst_write_addr_reg = 1;        
    //         end

    //         default: begin
    //             awready_reg = 0;
    //             wready_reg = 0;
    //             WorR_mux_en = 2'b00;  
    //             write_addr_reg_en = 0;
    //             write_data_reg_en = 0;  
    //             wskip_reg = 0;   
    //         end
    //     endcase
    // end

    always @(posedge axis_clk, negedge axis_rst_n) begin    //write channel
        if(!axis_rst_n)
            wc_state <= 3'd0;
        else    
            wc_state <= wc_state_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //awready_reg
        if(!axis_rst_n)
            awready <= 0;
        else    
            awready <= awready_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //wready_reg
        if(!axis_rst_n)
            wready <= 0;
        else    
            wready <= wready_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  
        if(!axis_rst_n)
            wskip <= 1'd0;
        else    
            wskip <= wskip_reg;
    end

//write_addr_compare//w_compa_wt, w_compa_wc
    always @(*) begin
        if(write_addr_reg_out == ADDR_LEN) begin
            w_compa_wt = 1'b0;     //tap controller idle state
            w_compa_wc = 1'b1;           

            len_reg_en = 1;
            block_pro_reg_en = 0;
        end
        else if((write_addr_reg_out >= ADDR_TAP_MIN) && (write_addr_reg_out <= ADDR_TAP_MAX)) begin
            w_compa_wt = 1'b1;     //can weite bram
            w_compa_wc = 1;           

            len_reg_en = 0;
            block_pro_reg_en = 0;
        end
        else if(write_addr_reg_out == ADDR_BLOCK) begin
            w_compa_wt = 1'b0;     
            w_compa_wc = 1;           

            len_reg_en = 0;
            block_pro_reg_en = 1;        
        end
        else begin
            w_compa_wt = 1'b0;     
            w_compa_wc = 1;           

            len_reg_en = 0;
            block_pro_reg_en = 0;          
        end
    end
//AXI_Lite tap_Din connect//
    always @(*) begin
        tap_Di = write_data_reg_out;
    end 

//write_addr_count//
    always @(*) begin
        write_addr_count_out = (write_addr_reg_out - ADDR_TAP_MIN) >> 2;
    end

//write_addr_reg//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n | rst_write_addr_reg)
            write_addr_reg_out <= 12'hfff;
        else begin
            if(write_addr_reg_en)
                write_addr_reg_out <= awaddr;
            else
                write_addr_reg_out <= write_addr_reg_out;
        end
    end

//write_data_reg//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            write_data_reg_out <= 32'd0;
        else begin
            if(write_data_reg_en)
                write_data_reg_out <= wdata;
            else
                write_data_reg_out <= write_data_reg_out;
        end
    end

//=======read_channel========
//read_FSM//

    always @(*) begin
        
        rst_read_addr_reg = 0;
        rvalid_reg_en = 0; 
        r_waitd_reg = 0;
        case(rc_state) 
        RC_ARVALID: begin   
            if(arvalid) begin   //6'b10_00_00
                arready_reg = 1; 
                rvalid_reg = 0;
                WorR_mux_en = 2'b10;
                read_addr_reg_en = 1;
                read_data_reg_en = 0;
                rskip_reg = 0;

                rc_state_reg = RC_ARREADY;
            end
            else begin
                arready_reg = 0;    //like back state
                rvalid_reg = 0;
                rvalid_reg_en = 1;
                WorR_mux_en = 2'b00;  
                read_addr_reg_en = 0;
                read_data_reg_en = 0;  
                rskip_reg = 0;    
                rst_read_addr_reg = 1;     

                block_pro_done_reg_en = 0;

                rc_state_reg = RC_ARVALID;
            end
        end
        RC_ARREADY: begin   
            if(arvalid && arready && !rskip && !r_waitd) begin    //6'b11_00_00
                arready_reg = 0; 
                rvalid_reg = 0;
                WorR_mux_en = 2'b10;
                read_addr_reg_en = 0;
                read_data_reg_en = 0;
                rskip_reg = 0;

                rc_state_reg = RC_RVALID;
            end
            else begin
                rc_state_reg = RC_ARREADY;
            end
        end
        RC_RVALID: begin    
            if(rready && !rskip && !r_waitd) begin   //6'b00_10_00
                arready_reg = 0; 
                rvalid_reg = 0;
                WorR_mux_en = 2'b10;
                read_addr_reg_en = 0;
                read_data_reg_en = 0;
                rskip_reg = 0;
                r_waitd_reg = 1;

                rc_state_reg = RC_RREADY;
            end
            else begin
                rc_state_reg = RC_RVALID;
            end 
        end
        RC_RREADY: begin    
            if(rready && !rvalid && !rskip && r_waitd) begin    //6'b00_10_01
                arready_reg = 0; 
                rvalid_reg = 1;
                rvalid_reg_en = 1;
                WorR_mux_en = 2'b10;
                read_addr_reg_en = 0;
                read_data_reg_en = 1;
                rskip_reg = 1;

                if(read_addr_reg_out == ADDR_BLOCK) begin
                    block_pro_done_reg_en = 1;              //everytime send rvalid rst ap_done once time
                    fir_w_ap_done = 0;
                    // ap_Yn_reg_en = 1;
                    // ap_Yn_reg = 0;
                end
                else begin
                    block_pro_done_reg_en = 0;             
                    fir_w_ap_done = 0;
                    // ap_Yn_reg_en = 0;
                    // ap_Yn_reg = 0;
                end

                rc_state_reg = RC_BACK;
            end
            else begin
                rc_state_reg = RC_RREADY;
            end             
        end
        RC_BACK: begin      
            if(rready && rvalid && rskip && !r_waitd) begin //6'b?0_11_10
                arready_reg = 0;
                rvalid_reg = 0;
                rvalid_reg_en = 1;
                WorR_mux_en = 2'b00;  
                read_addr_reg_en = 0;
                read_data_reg_en = 0;  
                rskip_reg = 0;    
                rst_read_addr_reg = 1;     

                block_pro_done_reg_en = 0;

                rc_state_reg = RC_ARVALID;
            end
            else begin
                rc_state_reg = RC_BACK;
            end               
        end
        default: begin
                arready_reg = 0;
                rvalid_reg = 0;
                WorR_mux_en = 2'b00;  
                read_addr_reg_en = 0;
                read_data_reg_en = 0;  
                rskip_reg = 0; 
                
                rc_state_reg = RC_ARVALID;
        end

        endcase
    end

    // always @(*) begin
    //     rst_read_addr_reg = 0;
    //     rvalid_reg_en = 0; 
    //     r_waitd_reg = 0;
    //     casez ({arvalid, arready, rready, rvalid, rskip, r_waitd})
    //         6'b10_00_00: begin
    //             arready_reg = 1; 
    //             rvalid_reg = 0;
    //             WorR_mux_en = 2'b10;
    //             read_addr_reg_en = 1;
    //             read_data_reg_en = 0;
    //             rskip_reg = 0;
    //         end
    //         6'b11_00_00: begin
    //             arready_reg = 0; 
    //             rvalid_reg = 0;
    //             WorR_mux_en = 2'b10;
    //             read_addr_reg_en = 0;
    //             read_data_reg_en = 0;
    //             rskip_reg = 0;
    //         end
    //         6'b00_10_00: begin         //wait data
    //             arready_reg = 0; 
    //             rvalid_reg = 0;
    //             WorR_mux_en = 2'b10;
    //             read_addr_reg_en = 0;
    //             read_data_reg_en = 0;
    //             rskip_reg = 0;
    //             r_waitd_reg = 1;
    //         end
    //         6'b00_10_01: begin          //set rdata signal
    //             arready_reg = 0; 
    //             rvalid_reg = 1;
    //             rvalid_reg_en = 1;
    //             WorR_mux_en = 2'b10;
    //             read_addr_reg_en = 0;
    //             read_data_reg_en = 1;
    //             rskip_reg = 1;

    //             block_pro_done_reg_en = 1;              //everytime send rvalid rst ap_done once time
    //             fir_w_ap_done = 0;
    //         end

    //         6'b?0_11_10: begin          //reset write_or_read_addr_MUX
    //             arready_reg = 0;
    //             rvalid_reg = 0;
    //             rvalid_reg_en = 1;
    //             WorR_mux_en = 2'b00;  
    //             read_addr_reg_en = 0;
    //             read_data_reg_en = 0;  
    //             rskip_reg = 0;    
    //             rst_read_addr_reg = 1;     

    //             block_pro_done_reg_en = 0;
    //         end

    //         default: begin
    //             arready_reg = 0;
    //             rvalid_reg = 0;
    //             WorR_mux_en = 2'b00;  
    //             read_addr_reg_en = 0;
    //             read_data_reg_en = 0;  
    //             rskip_reg = 0;   
    //         end
    //     endcase
    // end

    always @(posedge axis_clk, negedge axis_rst_n) begin//read channel state
        if(!axis_rst_n)
            rc_state <= 0;
        else    
            rc_state <= rc_state_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //arready_reg
        if(!axis_rst_n)
            arready <= 0;
        else    
            arready <= arready_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //rvalid_reg
        if(!axis_rst_n)
            rvalid <= 0;
        else begin  
            if(rvalid_reg_en)                             //with rdata out to channel
                rvalid <= rvalid_reg;
            else    
                rvalid <= rvalid;

        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //rskip
        if(!axis_rst_n)
            rskip <= 1'd0;
        else    
            rskip <= rskip_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin  //read channel wait data
        if(!axis_rst_n)
            r_waitd <= 1'd0;
        else    
            r_waitd <= r_waitd_reg;
    end

//read_addr_compare//
    always @(*) begin
        if(read_addr_reg_out == ADDR_LEN) begin
            r_compa_wt = 0;           
            r_compa_wc = 1;

            read_mux_en = 2'b00;
        end
        else if((read_addr_reg_out >= ADDR_TAP_MIN) && (read_addr_reg_out <= ADDR_TAP_MAX)) begin
            r_compa_wt = 1;           
            r_compa_wc = 0;          

            read_mux_en = 2'b01;
        end
        else if(read_addr_reg_out == ADDR_BLOCK) begin
            r_compa_wt = 0;           
            r_compa_wc = 1;          

            read_mux_en = 2'b10;       


        end
        else begin
            r_compa_wt = 0;           
            r_compa_wc = 1;          

            read_mux_en = 2'b11;        
        end
    end
//read_addr_count//
    always @(*) begin
        read_addr_count_out = (read_addr_reg_out - ADDR_TAP_MIN) >> 2;
    end
//read_MUX//
//00:len  01:bram 10:block_reg 
    always @(*) begin
        case(read_mux_en)
            4'b00: read_data_reg_in = len_reg_out;
            4'b01: read_data_reg_in = tap_Do;
            4'b10: read_data_reg_in = {26'd0, ap_Yn_reg_out, ap_Xn_reg_out, ap_res_reg_out, ap_idle_reg_out, ap_done_reg_out, ap_start_reg_out};      
            4'b11: read_data_reg_in = 32'd0;

        endcase
    end

//read_addr_reg//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n | rst_read_addr_reg)
            read_addr_reg_out <= 12'hfff;
        else begin
            if(read_addr_reg_en)
                read_addr_reg_out <= araddr;
            else
                read_addr_reg_out <= read_addr_reg_out;
        end
    end

//read_data_reg//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            rdata <= 32'd0;
        else begin
            if(read_data_reg_en)
                rdata <= read_data_reg_in;
            else
                rdata <= rdata;
        end
    end

//==========together===========
//write_or_read_addr_MUX//
//00:close to access bram, 01:write, 10:read
    always @(*) begin
        case(WorR_mux_en)
            2'b00: {wt, wc, t_A} = {1'b0 ,1'b1, 12'hfff};                            
            2'b01: {wt, wc, t_A} = {w_compa_wt, w_compa_wc, write_addr_count_out};       
            2'b10: {wt, wc, t_A} = {r_compa_wt ,r_compa_wc, read_addr_count_out}; 
            default: {wt, wc, t_A} = {1'b0 ,1'b1, 12'hfff}; 
        endcase
    end

//length_reg//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n | fir_rst_len)
            len_reg_out <= 32'd0;                   //fir work over need reset
        else begin
            if(len_reg_en)
                len_reg_out <= write_data_reg_out;
            else
                len_reg_out <= len_reg_out;
        end
    end


//block_protocol_reg//
//[0]:ap_start  [1]:ap_done  [2]:ap_idle

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_start_reg_out <= 1'd0;
        else if (!ap_idle_reg_out)
            ap_start_reg_out <= 1'd0;
        else begin
            if(block_pro_reg_en && ap_idle_reg_out)
                ap_start_reg_out <= write_data_reg_out[0];
            else
                ap_start_reg_out <= ap_start_reg_out;
        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_done_reg_out <= 1'd0;
        else begin
            if(block_pro_done_reg_en)
                ap_done_reg_out <= fir_w_ap_done;        //connect to FIR engine, can't connect wdata
            else
                ap_done_reg_out <= ap_done_reg_out;
        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_idle_reg_out <= 1'd1;
        else begin
            if(block_pro_idle_reg_en)
                ap_idle_reg_out <= fir_w_ap_idle;                           //connect to FIR engine, can't connect wdata
            else
                ap_idle_reg_out <= ap_idle_reg_out;
        end
    end
//new flag for lab4-2//
//[3] reserved  [4] ready to accept input [5] ready to read->reset by 0X00 is read


    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_Xn_reg_out <= 1'd0;
        else begin
            if(ap_Xn_reg_en)
                ap_Xn_reg_out <= ap_Xn_reg;                           
            else
                ap_Xn_reg_out <= ap_Xn_reg_out;
        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_Yn_reg_out <= 1'd0;
        else begin
            if(ap_Yn_reg_en)
                ap_Yn_reg_out <= ap_Yn_reg;                           
            else
                ap_Yn_reg_out <= ap_Yn_reg_out;
        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin    //read 0
        if (!axis_rst_n)
            ap_res_reg_out <= 1'd0;
        else begin
            if(ap_res_reg_en)
                ap_res_reg_out <= ap_res_reg;                           
            else
                ap_res_reg_out <= ap_res_reg_out;
        end
    end



//======================================//
//======TAP_BRAM_CONTROLLER_MODULE======//
//======================================//

//======================================
//000:idle
    always @(*) begin
        axi_tap_WE = 4'b0000;
        axi_tap_EN = 1'b0;
        axi_tap_A  = t_A;

        case(tap_control_state)

        //idle//t_EN
        4'b0000: begin
                case({wt, wc})       
                    2'b01: tap_control_next = 4'b0000;            
                    2'b11: tap_control_next = 4'b0001;
                    2'b10: tap_control_next = 4'b1001;
                    default: tap_control_next = 4'b0000;  
                endcase
        end
        //write
        4'b0001: tap_control_next = 4'b0010;
        4'b0010: begin
                axi_tap_WE = 4'b1111;
                axi_tap_EN = 1'b1;
                tap_control_next = 4'b011;
        end 
        4'b0011: begin
                axi_tap_WE = 4'b0000;
                axi_tap_EN = 1'b1;
                tap_control_next = 4'b0000;
        end 

        //read
        4'b1001: begin
                axi_tap_EN = 1'b0;
                tap_control_next = 4'b1010;
        end
        4'b1010: begin
                axi_tap_EN = 1'b1;
                tap_control_next = 4'b1011;
        end
        4'b1011: begin
                axi_tap_EN = 1'b0;
                tap_control_next = 4'b0000;
        end


        //default
        default: tap_control_next = 4'b0000;
        endcase
        
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            tap_control_state <= 4'b0000;
        else
            tap_control_state <= tap_control_next;
    end

//======================================//
//============FIR_ENGINE_MODULE=========//
//======================================//

    //parameter
    parameter FIR_IDLE = 3'b000,
              FIR_RST_DBRAM = 3'b001,
              FIR_RST_POINTER = 3'b010,
              FIR_MOVE_DATA = 3'b011,
              FIR_WORK = 3'b100,
              FIR_SEND_DATA = 3'b101;

    parameter IDLE = 2'b00,
              SEND = 2'b01,
              EAT  = 2'b10;

    parameter DPOINTER_IDLE = 3'b000,
              DPOINTER_RST_DBRAM = 3'b001,
              DPOINTER_MOVE_D = 3'b010,
              DPOINTER_WORK = 3'b011,
              DPOINTER_PIPE_2 = 3'b100;

    parameter WIDLE = 1'b0,
              WORK = 1'b1;
    parameter COMPA = 2'b00,
              Y_COMPA = 2'b01,
              N_COMPA = 2'b10;


    //variable 
    reg [31:0] fir_data_reg_out, fir_data_reg, fir_tap_reg_out, fir_tap_reg;
    reg [63:0] fir_result_out_reg_out, fir_result_out_reg;
    reg fir_data_reg_en, fir_tap_reg_en, fir_result_out_reg_en;

    reg [2:0] fir_controller_state, fir_controller_next;
    reg dbram_rst_finish, rst_dbram_en;
    reg [2:0] data_poiter_state, data_poiter_next;
    reg [1:0] dpoiter_rst_dbram_state, dpoiter_rst_dbram_next;
    reg [1:0] dpoiter_move_dbram_state, dpoiter_move_dbram_next;
    reg work_en, work_finish, small_counter_start;
    reg [4:0] small_counter, small_counter_reg, small_counter_rst;
    reg mid_counter_start, small_counter_write;
    reg [4:0] mid_counter, mid_counter_reg;
    reg move_dbram_en, dbram_move_finish;
    reg fir_rst_dbram_state, fir_rst_dbram_next, last_eat, last_eat_reg;
    reg [3:0] tap_counter, tap_counter_reg, tap_counter_rst;
    reg tap_counter_start, axiL_or_fir_mux;
    reg [4:0] pipe_counter, pipe_counter_reg;
    reg pipe_counter_start;
    reg [11:0] fir_tap_A;
    reg [3:0] fir_tap_WE;
    reg fir_tap_EN, dpointer_work_state, dpointer_work_next, pipe_counter_rst;
    reg [31:0] fir_out_sm_reg_out, fir_out_sm_reg;
    reg fir_out_sm_reg_en, mid_counter_rst;
    reg [1:0] fir_send_data_state, fir_send_data_next;
    reg [9:0] big_counter, big_counter_reg;
    reg big_counter_start, big_counter_rst;
    reg fir_result_out_reg_rst, fir_tap_reg_rst, fir_data_reg_rst;

//======================================
//FIR_controller//
//000:idle 001:set_memory 010:set_poiter 011:work 100:send_data  
    always @(*) begin
        axiL_or_fir_mux = 0;
        ap_Yn_reg_en = 0;


        case (fir_controller_state)

            FIR_IDLE: begin       //idle
                fir_rst_dbram_next = 0;     //close up state signal 
                fir_rst_len = 0;
                block_pro_done_reg_en = 0;
                fir_w_ap_done = 0;
                block_pro_idle_reg_en = 0;
                fir_w_ap_idle = 0;
                sm_tlast = 0;
                sm_tvalid = 0;

                small_counter_start = 0;
                small_counter_write = 0;
                small_counter_rst = 1;
                mid_counter_start = 0;
                mid_counter_rst = 1;
                big_counter_start = 0;
                big_counter_rst =1;
                tap_counter_start = 0;
                tap_counter_rst = 1;
                pipe_counter_start = 0;
                pipe_counter_rst = 1;

                if(len_reg_out != 0)
                    fir_controller_next = FIR_RST_DBRAM;
                else    
                    fir_controller_next = FIR_IDLE;
            end

            FIR_RST_DBRAM: begin         //reset data_bram
                small_counter_start = 0;
                small_counter_write = 0;
                small_counter_rst = 0;
                mid_counter_start = 0;
                mid_counter_rst = 0;
                big_counter_start = 0;
                big_counter_rst = 0;
                tap_counter_start = 0;
                tap_counter_rst = 0;
                pipe_counter_start = 0;
                pipe_counter_rst = 0;

                case(fir_rst_dbram_state)
                    1'b0: begin
                        if(dbram_rst_finish) begin
                            rst_dbram_en = 0;
                            fir_rst_dbram_next = 1;
                        end
                        else begin
                            rst_dbram_en = 1;
                            fir_rst_dbram_next = 0;
                        end                      
                    end
                    1'b1: begin
                        if(ap_start_reg_out) begin
                            fir_rst_dbram_next = 0;
                            fir_controller_next = FIR_RST_POINTER;

                            block_pro_idle_reg_en = 1;
                            fir_w_ap_idle = 0;
                        end
                        else 
                            fir_rst_dbram_next = 1;
                    end
                endcase
            end

            FIR_RST_POINTER: begin       //reset pointer
                block_pro_idle_reg_en = 0;      //let ap_idle set 1 cc

                fir_data_reg_rst = 1;             //rst fir reg
                fir_tap_reg_rst = 1;
                fir_result_out_reg_rst = 1;

                axiL_or_fir_mux = 1;
                fir_controller_next = FIR_MOVE_DATA;

                sm_tvalid = 0;
                mid_counter_start = 0;
                big_counter_start = 0;
            end

            FIR_MOVE_DATA: begin
                axiL_or_fir_mux = 1;
                fir_data_reg_rst = 0;             //rst fir reg
                fir_tap_reg_rst = 0;
                fir_result_out_reg_rst = 0;

                if(dbram_move_finish) begin
                    move_dbram_en = 0;
                    fir_controller_next = FIR_WORK;
                end
                else begin
                    move_dbram_en = 1;
                    fir_controller_next = FIR_MOVE_DATA;
                end
            end

            FIR_WORK: begin              //work
                axiL_or_fir_mux = 1;
                if(work_finish) begin
                    work_en = 0;
                    fir_controller_next = FIR_SEND_DATA;
                end
                else begin
                    work_en = 1;
                    fir_controller_next = FIR_WORK;
                end
            end

            FIR_SEND_DATA: begin         //send data out to fir
                fir_out_sm_reg_en = 0;

                case(fir_send_data_state)
                    COMPA: begin
                        if(big_counter == (len_reg_out-1))
                            fir_send_data_next = Y_COMPA;
                        else    
                            fir_send_data_next = N_COMPA;
                    end
                    Y_COMPA: begin
                        block_pro_idle_reg_en = 1;          //ap_idle
                        fir_w_ap_idle = 1;
                        sm_tvalid = 1;
                        sm_tlast = 1;
                        ap_Yn_reg_en = 1;
                        ap_Yn_reg = 1;

                        if(sm_tready) begin
                            block_pro_done_reg_en = 1;      //ap_done
                            fir_w_ap_done = 1;
                            fir_send_data_next = COMPA;     //back state
                            fir_controller_next = FIR_IDLE;

                            fir_rst_len = 1;                //rest len
                            ap_Yn_reg = 0;
                        end
                        else begin
                            fir_send_data_next = Y_COMPA;
                        end

                    end
                    N_COMPA: begin
                        sm_tvalid = 1;
                        ap_Yn_reg_en = 1;
                        ap_Yn_reg = 1;

                        if(sm_tready) begin
                            mid_counter_start = 1; 
                            big_counter_start = 1;

                            fir_send_data_next = COMPA;
                            fir_controller_next = FIR_RST_POINTER;

                            ap_Yn_reg_en = 1;
                            ap_Yn_reg = 0;
                        end
                        else begin
                            fir_send_data_next = N_COMPA;
                            ap_Yn_reg_en = 1;
                            ap_Yn_reg = 1;
                        end
                    end

                    default: begin
                        fir_send_data_next = COMPA;
                    end
                endcase
            end
        endcase
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            fir_controller_state <= 0;
        else 
            fir_controller_state <= fir_controller_next;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            fir_rst_dbram_state <= 0;
        else 
            fir_rst_dbram_state <= fir_rst_dbram_next;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //FIR_SEND_DATA
        if(!axis_rst_n)
            fir_send_data_state <= 0;
        else 
            fir_send_data_state <= fir_send_data_next;
    end    


//AXI_LITE or FIR_ENGINE access tap bram MUX//
//0:AXI_Lite  1:FIR
    always @(*) begin
        if(axiL_or_fir_mux)
            {tap_WE, tap_EN, tap_A} = {fir_tap_WE, fir_tap_EN, fir_tap_A};
        else
            {tap_WE, tap_EN, tap_A} = {axi_tap_WE, axi_tap_EN, axi_tap_A};
    end

//FIR//
    always @(*) begin
        fir_result_out_reg = (fir_data_reg_out * fir_tap_reg_out) + fir_result_out_reg_out;
    end

    always @(*) begin
        fir_out_sm_reg = fir_result_out_reg_out[31:0];        
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //data_reg
        if(!axis_rst_n)
            fir_data_reg_out <= 0;
        else begin   
            if(fir_data_reg_rst)
                fir_data_reg_out <= 0;
            else if(fir_data_reg_en)
                fir_data_reg_out <= fir_data_reg;
            else 
                fir_data_reg_out <= fir_data_reg_out;
        end

    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //tap_reg
        if(!axis_rst_n)
            fir_tap_reg_out <= 0;
        else begin
            if(fir_tap_reg_rst)
                fir_tap_reg_out <= 0;
            else if(fir_tap_reg_en)
                fir_tap_reg_out <= fir_tap_reg;
            else
                fir_tap_reg_out <= fir_tap_reg_out;
        end

    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //fir_result_out_reg
        if(!axis_rst_n)
            fir_result_out_reg_out <= 0;
        else begin    
            if(fir_result_out_reg_rst)
                fir_result_out_reg_out <= 0;
            else    
                fir_result_out_reg_out <= fir_result_out_reg;
        end
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //fir_out_sm_reg
        if(!axis_rst_n)
            fir_out_sm_reg_out <= 0;
        else begin    
            if(fir_out_sm_reg_en)
                fir_out_sm_reg_out <= fir_out_sm_reg;
            else
                fir_out_sm_reg_out <= fir_out_sm_reg_out;
        end
    end

    always @(*) begin
        sm_tdata = fir_out_sm_reg_out;
    end

//data_poiter//
//the second FSM to controll data & tap pointer
//follow FIR controll signal & maintain every counter
    always @(*) begin
        ss_tready_reg = 0;
        ap_Xn_reg_en = 0;
        case(data_poiter_state)

            DPOINTER_IDLE: begin
                dbram_rst_finish = 0;
                dbram_move_finish = 0;  
                work_finish = 0;
                data_EN = 0;
                last_eat_reg = 0 ;
                dpoiter_rst_dbram_next = 0;




                if(rst_dbram_en)
                    data_poiter_next = DPOINTER_RST_DBRAM;
                else if(move_dbram_en)
                    data_poiter_next = DPOINTER_MOVE_D;
                else if(work_en) 
                    data_poiter_next = DPOINTER_WORK;
                else 
                    data_poiter_next = DPOINTER_IDLE;
            end

            DPOINTER_RST_DBRAM: begin
                case(dpoiter_rst_dbram_state)
                    IDLE: begin                           //idle
                        data_WE = 4'b0000;
                        data_EN = 1'b0;
                        data_Di = 32'd0;
                        data_A = 12'd0;

                        small_counter_start = 0;
                        small_counter_write = 0; 
                        small_counter_rst = 1; 

                        dpoiter_rst_dbram_next = SEND;
                        dbram_rst_finish = 1'b0;
                        last_eat_reg = 0;
                    end
                    SEND: begin                           //send WE, EN
                        data_WE = 4'b1111;
                        data_EN = 1'b1;
                        data_Di = 32'd0;
                        data_A = small_counter;

                        small_counter_start = 1;
                        small_counter_write = 0; 
                        small_counter_rst = 0; 

                        dpoiter_rst_dbram_next = EAT;
                        dbram_rst_finish = 1'b0;
                        if (small_counter == 10) begin
                            last_eat_reg = 1;
                        end
                        else
                            last_eat_reg = 0;
                    end
                    EAT: begin                           //dbram eat data
                        small_counter_start = 0;
                        small_counter_write = 0; 
                        small_counter_rst = 0; 
                        if(last_eat) begin
                            data_WE = 4'b0000;
                            data_EN = 1'b1; 
                            data_Di = 32'd0;
                            data_A = small_counter;

                            dpoiter_rst_dbram_next = IDLE;
                            data_poiter_next = DPOINTER_IDLE;
                            dbram_rst_finish = 1'b1;            //tell fir_FSM
                            last_eat_reg = 0;
                        end
                        else begin 
                            data_WE = 4'b0000;
                            data_EN = 1'b1; 
                            data_Di = 32'd0;
                            data_A = small_counter;

                            dpoiter_rst_dbram_next = SEND;
                            dbram_rst_finish = 1'b0;
                        end
                    end                      
                    default: begin
                        data_WE = 4'b0000;
                        data_EN = 1'b0;
                        data_Di = 32'd0;
                        data_A = 12'd0;

                        small_counter_start = 0;
                        small_counter_write = 0; 
                        small_counter_rst = 0; 

                        dpoiter_rst_dbram_next = IDLE;
                        dbram_rst_finish = 1'b0;
                    end
                endcase
            end

            DPOINTER_MOVE_D: begin
                case(dpoiter_move_dbram_state)
                    IDLE: begin
                        data_WE = 4'b0000;
                        data_EN = 1'b0;
                        data_Di = ss_tdata;
                        data_A = 12'd0;        

                        dpoiter_move_dbram_next = SEND;
                        dbram_move_finish = 0;  
                        ss_tready_reg =0; 

                        ap_Xn_reg_en = 1;           //can eat data, wait ss_tvalid
                        ap_Xn_reg =1;
                    end
                    SEND: begin
                        if(ss_tvalid) begin
                            data_WE = 4'b1111;
                            data_EN = 1'b1;
                            data_Di = ss_tdata;
                            data_A = mid_counter;  

                            dpoiter_move_dbram_next = EAT;    
                            dbram_move_finish = 0;  

                            ss_tready_reg = 1;  
                            ap_Xn_reg_en = 1;           //can eat data, wait ss_tvalid
                            ap_Xn_reg =0;
                        end
                        else begin
                            dpoiter_move_dbram_next = SEND;
                        end             
                    end
                    EAT: begin
                        data_WE = 4'b0000;
                        data_EN = 1'b1; 
                        data_Di = ss_tdata;
                        data_A = 12'd0; 

                        dpoiter_move_dbram_next = IDLE;  
                        data_poiter_next = DPOINTER_IDLE;
                        dbram_move_finish = 1; 
                        ss_tready_reg = 0;                    
                    end
                    default: begin
                        data_WE = 4'b0000;
                        data_EN = 1'b0; 
                        data_Di = 32'd0;
                        data_A = 12'd0; 
 
                        dpoiter_move_dbram_next = IDLE;
                        dbram_move_finish = 0; 
                        ss_tready_reg = 0; 
                    end
                endcase
            end

            DPOINTER_WORK: begin
                data_WE = 4'b0000;
                data_EN = 1'b0; 
                fir_tap_WE = 4'b0000;
                fir_tap_EN = 1'b0;
                work_finish = 0;

                case(dpointer_work_state)
                    WIDLE: begin
                        data_A = 12'd0;
                        fir_data_reg = data_Do;
                        fir_data_reg_en = 0;

                        fir_tap_A = 12'd0;
                        fir_tap_reg = tap_Do;
                        fir_tap_reg_en = 0;

                        small_counter_start = 0;
                        small_counter_write = 1;            //mid -> small
                        small_counter_rst = 0; 

                        tap_counter_start = 0;
                        tap_counter_rst = 1;                //rst tap_counter

                        pipe_counter_start = 0;
                        pipe_counter_rst = 1;               //rst pipe_counter

                        dpointer_work_next = WORK;
                        data_poiter_next = DPOINTER_WORK;

                    end
                    WORK: begin
                        if(pipe_counter == 13) begin
                            fir_out_sm_reg_en = 1;
                            work_finish = 1;

                            data_A = 12'd0;
                            fir_data_reg = data_Do;
                            fir_data_reg_en = 0;

                            fir_tap_A = 12'd0;
                            fir_tap_reg = tap_Do;
                            fir_tap_reg_en = 0;
                        
                            small_counter_start = 1;
                            tap_counter_start = 0;
                            tap_counter_rst = 0;
                            pipe_counter_start = 1;

                            dpointer_work_next = WIDLE;
                            data_poiter_next = DPOINTER_IDLE;
                        end
                        else begin
                            data_EN = 1'b1; 
                            data_A = small_counter;
                            fir_data_reg = data_Do;
                            fir_data_reg_en = 1;

                            fir_tap_EN = 1'b1;
                            fir_tap_A = tap_counter;
                            fir_tap_reg = tap_Do;
                            fir_tap_reg_en = 1;

                            small_counter_start = 1;
                            tap_counter_start = 1;
                            tap_counter_rst = 0;
                            pipe_counter_start = 1;

                            dpointer_work_next = WORK;
                            data_poiter_next = DPOINTER_WORK;
                        end
                    end
                    default: begin
                        dpointer_work_next = WIDLE;
                    end
                endcase
            end

            DPOINTER_PIPE_2: begin
                dpoiter_rst_dbram_next = DPOINTER_MOVE_D;
            end

            default: dpoiter_rst_dbram_next = DPOINTER_MOVE_D;
        endcase

    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //ss_tready
        if(!axis_rst_n)
            ss_tready <= 0;
        else 
            ss_tready <= ss_tready_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //pointer_state_reg
        if(!axis_rst_n)
            data_poiter_state <= 0;
        else 
            data_poiter_state <= data_poiter_next;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //DPOINTER_RST_DBRAM_state_reg
        if(!axis_rst_n)
            dpoiter_rst_dbram_state <= 0;
        else 
            dpoiter_rst_dbram_state <= dpoiter_rst_dbram_next;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin        //DPOINTER_MOVE_DBRAM_state_reg
        if(!axis_rst_n)
            dpoiter_move_dbram_state <= 0;
        else 
            dpoiter_move_dbram_state <= dpoiter_move_dbram_next;
    end
    always @(posedge axis_clk, negedge axis_rst_n) begin        //last_eat
        if(!axis_rst_n)
            last_eat <= 0;
        else 
            last_eat <= last_eat_reg;
    end
    
    always @(posedge axis_clk, negedge axis_rst_n) begin        //DPOINTER_WORK_reg
        if(!axis_rst_n)
            dpointer_work_state <= 0;
        else 
            dpointer_work_state <= dpointer_work_next;
    end
//==========together===========
//=============================
//pipe_counter//
//count 13c.c -> fir operate 11 data need 13 cycle with pipeline tec. 
    always @(*) begin
        if(pipe_counter_start) begin
            if(pipe_counter == 13)
                pipe_counter_reg = 5'd0;
            else
                pipe_counter_reg = pipe_counter + 5'd1;
        end
        else if(pipe_counter_rst)
            pipe_counter_reg = 4'd0;
        else
            pipe_counter_reg = pipe_counter;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            pipe_counter <= 5'd0;
        else
            pipe_counter <= pipe_counter_reg;
    end  

//tap_counter//
//read tap to fir
    always @(*) begin
        if(tap_counter_start) begin
            if(tap_counter == 10)
                tap_counter_reg = 4'd0;
            else
                tap_counter_reg = tap_counter + 4'd1;
        end
        else if(tap_counter_rst)
            tap_counter_reg = 4'd0;
        else
            tap_counter_reg = tap_counter;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            tap_counter <= 4'd0;
        else
            tap_counter <= tap_counter_reg;
    end

//small_counter//
//count rst bram 11cc & work pipeline 17cc
    always @(*) begin
        if(small_counter_start) begin
            if(small_counter == 10)
                small_counter_reg = 5'd0;
            else
                small_counter_reg = small_counter + 5'd1;
        end
        else if(small_counter_write)
            small_counter_reg = mid_counter;
        else if(small_counter_rst)
            small_counter_reg = 5'd0;
        else
            small_counter_reg = small_counter;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            small_counter <= 5'd0;
        else
            small_counter <= small_counter_reg;
    end

//mid_count//
//count write data from ss to bram address
    always @(*) begin
        if(mid_counter_start) begin
            if((mid_counter == 0))
                mid_counter_reg = 5'd10;
            else
                mid_counter_reg = mid_counter - 5'd1;
        end
        else if(mid_counter_rst)
            mid_counter_reg = 5'd10;
        else
            mid_counter_reg = mid_counter;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            mid_counter <= 5'd10;
        else
            mid_counter <= mid_counter_reg;
    end

//big_count//
//count 600cc
    always @(*) begin
        if(big_counter == len_reg_out)
            big_counter_reg = 10'd0;
        else if(big_counter_start)
            big_counter_reg = big_counter + 10'd1;
        else if(big_counter_rst)
            big_counter_reg = 10'd0;
        else
            big_counter_reg = big_counter;
        
    end

    
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(!axis_rst_n)
            big_counter <= 10'd0;
        else
            big_counter <= big_counter_reg;
    end


endmodule