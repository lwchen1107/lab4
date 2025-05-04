module exmem (
    input         wb_clk_i,
    input         wb_rst_i,
    input         wbs_stb_i,
    input         wbs_cyc_i,
    input         wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output        wbs_ack_o,
    output [31:0] wbs_dat_o
);
    reg [3:0] count;

    wire valid;
    wire we, en;
    
    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            count <= 0;
        end else begin
            if (count == 4'd10) count <= 0;
            else if (wbs_cyc_i & !wbs_we_i) count <= count + 1;
        end
    end

    assign valid = wbs_cyc_i & wbs_stb_i;
    assign we = valid & wbs_we_i;
    assign en = 1'b1;
    
    bram user_mem (
        .CLK(wb_clk_i),
        .WE0(wbs_sel_i & {4{we}}),
        .EN0(en),
        .Di0(wbs_dat_i),
        .Do0(wbs_dat_o),
        .A0(wbs_adr_i << 2)
    );

    assign wbs_ack_o = ((valid & wbs_we_i) | count == 4'd10);

endmodule