module wb2axi (
    input         wb_clk_i,
    input         wb_rst_i,
    input         wbs_stb_i,
    input         wbs_cyc_i,
    input         wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output        wbs_ack_o,
    output [31:0] wbs_dat_o,

    input         awready,
    input         wready,
    output        awvalid,
    output [11:0] awaddr,
    output        wvalid,
    output [31:0] wdata,

    input         arready,
    output        rready,
    output        arvalid,
    output [11:0] araddr,
    input         rvalid,
    input  [31:0] rdata,

    output        ss_tvalid,
    output [31:0] ss_tdata,
    output        ss_tlast,
    input         ss_tready,

    output        sm_tready,
    input         sm_tvalid,
    input  [31:0] sm_tdata,
    input         sm_tlast
);

    reg [31:0] ss_last_count;
    wire valid;
    wire axilite_ack, axis_ack, axis_s_ack, axis_m_ack;
    wire axilite_sel, axis_s_sel, axis_m_sel;
    reg arvalid_en;

    assign valid = wbs_cyc_i & wbs_stb_i;

    assign axilite_ack = (awready & wready) | rvalid;
    assign axis_ack = axis_m_ack | axis_s_ack;
    assign axis_s_ack = axis_s_sel & ss_tready;
    assign axis_m_ack = axis_m_sel & /*axis_m_sel_r &*/ sm_tvalid;

    assign axilite_sel = (wbs_adr_i[31:8] == 24'h3000_00)
                       & (wbs_adr_i != 32'h3000_0040) & (wbs_adr_i != 32'h3000_0044);
    assign axis_s_sel = (wbs_adr_i == 32'h3000_0040);
    assign axis_m_sel = (wbs_adr_i == 32'h3000_0044);

    // AXI to WB
    assign wbs_ack_o = wbs_cyc_i & (axilite_ack | axis_ack);
    assign wbs_dat_o = { 32{axis_m_sel} } & sm_tdata | { 32{rvalid} } & rdata;


    // WB to AXI-Lite
    // Write
    assign awvalid = valid & wbs_we_i & axilite_sel;
    assign awaddr = wbs_adr_i[11:0];
    assign wvalid = valid & wbs_we_i & axilite_sel;
    assign wdata = wbs_dat_i;

    // Read
    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) arvalid_en <= 1;
        else begin
            if (arvalid && arready) arvalid_en <= 0; 
            else if (rvalid && rready) arvalid_en <= 1;
        end
    end
    assign rready = wbs_cyc_i & !wbs_we_i & axilite_sel;
    assign arvalid = valid & !wbs_we_i & axilite_sel & arvalid_en;
    assign araddr = wbs_adr_i[11:0];


    // WB to AXIS
    // AXIS slave
    assign ss_tvalid = valid & wbs_we_i & axis_s_sel;
    assign ss_tdata = wbs_dat_i;
    assign ss_tlast = ss_tvalid & (ss_last_count == 1);

    // TODO: latch data_length
    always @(posedge wb_clk_i) begin
        if (valid & wbs_we_i & (wbs_adr_i == 32'h3000_0010)) begin
            ss_last_count <= wbs_dat_i;
        end else if (ss_tvalid & ss_tready) begin
            ss_last_count <= ss_last_count - 1;
        end
    end

    // AXIS master
/*
    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) axis_m_sel_r <= 0;
        else axis_m_sel_r <= axis_m_sel & ~axis_m_sel_r;
    end
*/
    assign sm_tready = wbs_cyc_i & !wbs_we_i & axis_m_sel/* & axis_m_sel_r*/;

endmodule