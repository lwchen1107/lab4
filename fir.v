`timescale 1ns / 1ps

// File: fir.v
// Auther: dqrengg
// Reference: https://github.com/bol-edu/caravel-soc_fpga-lab/tree/main/lab-fir
// Date: 2025 Mar 31

(* use_dsp = "no" *)

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,

    input   wire                     ss_tvalid,
    input   wire [(pDATA_WIDTH-1):0] ss_tdata,
    input   wire                     ss_tlast,
    output  wire                     ss_tready,

    input   wire                     sm_tready,
    output  wire                     sm_tvalid,
    output  wire [(pDATA_WIDTH-1):0] sm_tdata,
    output  wire                     sm_tlast,
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    
    // Local parameters

    // FSM
    localparam FIR_STATE_INIT = 3'b001;
    localparam FIR_STATE_WAIT = 3'b010;
    localparam FIR_STATE_CALC = 3'b100;

    // buffers
    localparam AXIS_BUF_ADDR_WIDTH = 1; // SIZE = 2^1
    localparam AXIS_BUF_SIZE = 1 << AXIS_BUF_ADDR_WIDTH;

    // Variables

    // 1. FSM
    reg [2:0] state;
    wire init_done, cfg_done, all_done;
    wire state_init, state_wait, state_calc;
    reg [2:0] fsm_out;

    reg [(pADDR_WIDTH-1):0] init_addr;

    // 2. cfg
    reg ap_start, ap_done, ap_idle;
    wire [(pDATA_WIDTH-1):0] ap_cfg_regs;

    wire assert_ap_start, deassert_ap_done;
    wire x_sampled, last_y_transferred;

    reg [(pDATA_WIDTH-1):0] data_length;

    // 3. buffers
    reg [(pDATA_WIDTH-1):0] x_buf[0:(AXIS_BUF_SIZE-1)];
    reg [(pDATA_WIDTH-1):0] y_buf[0:(AXIS_BUF_SIZE-1)];
    reg [AXIS_BUF_ADDR_WIDTH:0] x_buf_wp, x_buf_rp, y_buf_wp, y_buf_rp;
    wire x_buf_full, x_buf_empty, y_buf_full, y_buf_empty;
    wire x_buf_re, x_buf_we, y_buf_re, y_buf_we;
    wire [(pDATA_WIDTH-1):0] x_buf_in, x_buf_out, y_buf_in, y_buf_out;
    integer x_i, y_i;
    wire x_buf_valid, y_buf_ready;

    // 4. core
    wire [(pDATA_WIDTH-1):0] x, h;
    reg [(pDATA_WIDTH-1):0] mul, acc;

    wire x_ready, acc_valid;
    wire stall, x_stall, y_stall;   

    reg [(pADDR_WIDTH-1):0] tap_addr, tap_addr_pre;
    reg [(pADDR_WIDTH-1):0] mul_tap_addr, add_tap_addr, out_tap_addr;
    reg [(pADDR_WIDTH-1):0] data_addr, data_addr_pre;

    reg [(pDATA_WIDTH-1):0] y_counter;

    // 5. AXIS
    wire data_re, data_we;
    reg [(pADDR_WIDTH-1):0] data_A_r;
    reg ss_done;

    // 6. AXI-Lite
    wire tap_re, tap_we;
    reg [(pADDR_WIDTH-1):0] tap_A_r;
    reg [(pDATA_WIDTH-1):0] rdata_r;

    reg [(pADDR_WIDTH-1):0] araddr_buf;
    reg [(pDATA_WIDTH-1):0] rdata_buf;
    reg rdata_valid;

    reg arready_r, rvalid_r;
    reg awready_r, wready_r;

    // ========================================
    //  Section 1: FIR FSM
    // ========================================

    // next state logic
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            state <= FIR_STATE_INIT;
        end else begin
            case (state)
                FIR_STATE_INIT: state <= init_done ? FIR_STATE_WAIT : FIR_STATE_INIT;
                FIR_STATE_WAIT: state <= cfg_done  ? FIR_STATE_CALC : FIR_STATE_WAIT;
                FIR_STATE_CALC: state <= all_done  ? FIR_STATE_INIT : FIR_STATE_CALC;
                default: state <= FIR_STATE_INIT;
            endcase
        end
    end

    // output logic
    assign {state_init, state_wait, state_calc} = fsm_out;
    always @(*) begin
        case (state)
            FIR_STATE_INIT: fsm_out = 3'b1_0_0;
            FIR_STATE_WAIT: fsm_out = 3'b0_1_0;
            FIR_STATE_CALC: fsm_out = 3'b0_0_1;
            default:        fsm_out = 3'b1_0_0;
        endcase
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            init_addr <= 0;
        end else begin
            if (state_init && !init_done) begin
                init_addr <= init_addr + 1;
            end else begin
                init_addr <= 0;
            end
        end
    end

    // FSM inputs
    assign init_done = (init_addr == Tape_Num - 1);
    assign cfg_done = assert_ap_start;    // write ap start
    assign all_done = last_y_transferred; // read ap done

    // ========================================
    //  Section 2: Configuration
    // ========================================

    // ap_* protocol
    assign ap_cfg_regs = { {(pDATA_WIDTH-3){1'b0}}, ap_idle, ap_done, ap_start };

    assign assert_ap_start = (awaddr == 12'h000) & awvalid & awready    // address check 0x000
                           & (wdata[0] == 1'b1) & wvalid & wready       // value check
                        // & !ap_start                                  // can be asserted only one time, redundant logic
                           & state_wait;                                // start when engine is prepared
    assign deassert_ap_done = (araddr_buf == 12'h000) & rvalid & rready // address check 0x000
                            & ap_done;                                  // only when ap_done = 1
    
    assign x_sampled = ss_tvalid && ss_tready;
    assign last_y_transferred = sm_tvalid && sm_tready && sm_tlast;

    // ap_start
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_start <= 0;
        end else begin
            // set: when host program ap_start = 1
            if (assert_ap_start) begin
                ap_start <= 1;
            // reset: when the first X data is sampled
            end else if (ap_start && x_sampled) begin
                ap_start <= 0;
            end
        end
    end

    // ap_done
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_done <= 0;
        end else begin
            // set: when the last Y data is transferred
            if (last_y_transferred) begin
                ap_done <= 1;
            // reset: when ap_done is read by host
            //        or restart the engine without reading ap_done
            end else if (deassert_ap_done || assert_ap_start) begin
                ap_done <= 0;
            end
        end
    end

    // ap_idle
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_idle <= 1;
        end else begin
            // set to 0: when the fisrt X is sampled (and ap_start is sampled)
            if (ap_start && x_sampled) begin
                ap_idle <= 0;
            // set to 1: when the last Y data is transferred 
            end else if (last_y_transferred) begin
                ap_idle <= 1;
            end
        end
    end

    // data length
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            data_length <= 0;
        end else begin
            if (!state_calc && (awaddr == 12'h010)
                && awvalid && awready && wvalid && wready) begin // set
                data_length <= wdata;
            end else begin
                data_length <= data_length;
            end
        end
    end

    // ========================================
    //  Section 3: Buffer, zero cycle delay FIFO implementation
    // ========================================

    // FIXED: can handle conccruent read and write while empty or full

    // x buffer
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            for (x_i=0; x_i<AXIS_BUF_SIZE; x_i=x_i+1) x_buf[x_i] <= 0;
        end else begin
            if (state_init) begin
                for (x_i=0; x_i<AXIS_BUF_SIZE; x_i=x_i+1) x_buf[x_i] <= 0;
            end else if (x_buf_we && (!x_buf_full || x_buf_re)) begin
                x_buf[x_buf_wp[AXIS_BUF_ADDR_WIDTH-1:0]] <= x_buf_in;
            end
        end
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            x_buf_wp <= 0;
            x_buf_rp <= 0;
        end else begin
            if (state_init) begin
                x_buf_wp <= 0;
            end else if (x_buf_we && (!x_buf_full || x_buf_re)) begin
                x_buf_wp <= x_buf_wp + 1;
            end
            if (state_init) begin
                x_buf_rp <= 0;
            end else if (x_buf_re && (!x_buf_empty || x_buf_we)) begin
                x_buf_rp <= x_buf_rp + 1;
            end
        end
    end

    // x_buf outputs
    assign x_buf_out = (x_buf_empty) ? x_buf_in : x_buf[x_buf_rp[AXIS_BUF_ADDR_WIDTH-1:0]];
    assign x_buf_full = (x_buf_wp[AXIS_BUF_ADDR_WIDTH] != x_buf_rp[AXIS_BUF_ADDR_WIDTH]) &&
                        (x_buf_wp[AXIS_BUF_ADDR_WIDTH-1:0] == x_buf_rp[AXIS_BUF_ADDR_WIDTH-1:0]);
    assign x_buf_empty = (x_buf_wp == x_buf_rp);
    // x_buf inputs
    assign x_buf_in = ss_tdata;
    assign x_buf_we = ss_tvalid & ss_tready;
    assign x_buf_re = x_buf_valid & x_ready;
    // x_buf valid, for bypass
    assign x_buf_valid = !x_buf_empty | ss_tvalid;

    // y buffer
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            for (y_i=0; y_i<AXIS_BUF_SIZE; y_i=y_i+1) y_buf[y_i] <= 0;
        end else begin
            if (state_init) begin
                for (y_i=0; y_i<AXIS_BUF_SIZE; y_i=y_i+1) y_buf[y_i] <= 0;
            end else if (y_buf_we && (!y_buf_full || y_buf_re)) begin
                y_buf[y_buf_wp[AXIS_BUF_ADDR_WIDTH-1:0]] <= y_buf_in;
            end
        end
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            y_buf_wp <= 0;
            y_buf_rp <= 0;
        end else begin
            if (state_init) begin
                y_buf_wp <= 0;
            end else if (y_buf_we && (!y_buf_full || y_buf_re)) begin
                y_buf_wp <= y_buf_wp + 1;
            end
            if (state_init) begin
                y_buf_rp <= 0;
            end else if (y_buf_re && (!y_buf_empty || y_buf_we)) begin
                y_buf_rp <= y_buf_rp + 1;
            end
        end
    end

    // y_buf outputs
    assign y_buf_out = (y_buf_empty) ? y_buf_in : y_buf[y_buf_rp[AXIS_BUF_ADDR_WIDTH-1:0]];
    assign y_buf_full = (y_buf_wp[AXIS_BUF_ADDR_WIDTH] != y_buf_rp[AXIS_BUF_ADDR_WIDTH])
                      & (y_buf_wp[AXIS_BUF_ADDR_WIDTH-1:0] == y_buf_rp[AXIS_BUF_ADDR_WIDTH-1:0]);
    assign y_buf_empty = (y_buf_wp == y_buf_rp);
    // y_buf inputs
    assign y_buf_in = acc;
    assign y_buf_we = acc_valid & y_buf_ready;
    assign y_buf_re = sm_tvalid & sm_tready;
    // y_buf ready, for bypass
    assign y_buf_ready = !y_buf_full | sm_tready;

    // ========================================
    //  Section 4: FIR Core
    // ========================================

    // pipeline stage 1: address generation
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            tap_addr <= Tape_Num - 1;
            data_addr <= 0;
        end else begin
            if (state_calc && (!stall)) begin
                if (tap_addr == 0) tap_addr <= Tape_Num - 1;
                else               tap_addr <= tap_addr - 1;
                if (data_addr == Tape_Num - 2) data_addr <= 0;
                else                           data_addr <= data_addr + 1;
            end else if (state_init) begin
                tap_addr <= Tape_Num - 1;
                data_addr <= 0;
            end
        end
    end

    // latch the SRAM addr for stall
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            tap_addr_pre <= Tape_Num - 1;
            data_addr_pre <= 0;
        end else begin
            if (state_calc && (!stall)) begin
                tap_addr_pre <= tap_addr;
                data_addr_pre <= data_addr;
            end
        end
    end

    // for x buffer
    assign x_ready = state_calc & (tap_addr == 0);

    // stall
    assign x_stall = x_ready & (!x_buf_valid) & !ss_done;
    assign y_stall = acc_valid & (!y_buf_ready);
    assign stall = x_stall | y_stall;

    // pipeline stage 2: BRAM access
    assign x = data_Do;
    assign h = tap_Do;

    // pipeline stage 3: multiplication
    // pipeline stage 4: addition
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            mul <= 0;
            acc <= 0;
        end else begin
            if (state_calc && (!stall)) begin
                mul <= $signed(x) * $signed(h);
                acc <= (add_tap_addr == Tape_Num - 1) ? mul : ($signed(acc) + $signed(mul));
            end else if (state_init) begin
                mul <= 0;
                acc <= 0;
            end
        end
    end

    // pass tap address to each pipeline stage
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            add_tap_addr <= Tape_Num - 1;
            mul_tap_addr <= Tape_Num - 1;
            out_tap_addr <= Tape_Num - 1;
        end else begin
            if (state_calc && (!stall)) begin
                mul_tap_addr <= tap_addr;
                add_tap_addr <= mul_tap_addr;
                out_tap_addr <= add_tap_addr;
            end else if (state_init) begin
                add_tap_addr <= Tape_Num - 1;
                mul_tap_addr <= Tape_Num - 1;
                out_tap_addr <= Tape_Num - 1;
            end
        end
    end

    // for y buffer
    assign acc_valid = state_calc & (out_tap_addr == 0);

    // y counter for sm_tlast
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            y_counter <= 0;
        end else begin
            if (state_calc) begin
                if (sm_tvalid && sm_tready) y_counter <= y_counter + 1;
            end else if (state_init) begin
                y_counter <= 0;
            end
        end
    end
    
    // ========================================
    //  Section 5: AXIS
    // ========================================

    // AXIS Master

    // already handle sm_tready input before
    assign sm_tvalid = state_calc & (!y_buf_empty | acc_valid);
    assign sm_tlast = sm_tvalid & (y_counter == data_length - 1);
    assign sm_tdata = y_buf_out;

    // AXIS Slave

    // already handle ss_tvalid and ss_tdata inputs before
    assign ss_tready = state_calc & (!x_buf_full | x_ready);

    // handle ss_tlast to prevent pipeline stall when no X input
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ss_done <= 0;
        end else begin
            if (state_init) begin
                ss_done <= 0;
            end else if (ss_tvalid && ss_tready && ss_tlast) begin
                ss_done <= 1;
            end
        end
    end

    // Data RAM
    // can be always on, stall control by address
    assign data_we = state_init
                   | state_calc & x_ready & (!stall);
    assign data_WE = { 4{data_we} };
    assign data_EN = 1'b1;
    assign data_Di = (state_init) ? { pDATA_WIDTH{1'b0} } : x_buf_out;

    // Data BRAM address
    // always @(*) begin
    //     if (state_init) begin
    //         data_A_r = init_addr << 2;
    //     end else if (state_calc) begin
    //         data_A_r = (stall) ? (data_addr_pre << 2) : (data_addr << 2);
    //     end else begin
    //         data_A_r = 0 << 2;
    //     end
    // end
    assign data_A = (state_init) ? (init_addr << 2) : ((stall) ? (data_addr_pre << 2) : (data_addr << 2));

    // ========================================
    // Section 6: AXI-Lite
    // ========================================

    // for AXI handshake signals, during calculation, return the values as during configuration
    // invalid read: return 32'hFFFF_FFFF
    // invalid write: ignore
    // priority: read > write

    wire is_ram_addr_w, is_ram_addr_r;
    assign is_ram_addr_w = awaddr[(pADDR_WIDTH-1):7] == 'b1;
    assign is_ram_addr_r = araddr[(pADDR_WIDTH-1):7] == 'b1;

    // Tap RAM
    assign tap_we = (state_init | state_wait) & is_ram_addr_w & (awvalid & wvalid) & (!arvalid);
    assign tap_WE = { 4{tap_we} };
    assign tap_EN = 1'b1;
    
    assign tap_Di = wdata;

    // Tap BRAM address
    // cannot write 
    always @(*) begin
        case (state_calc)
            1'b1:    tap_A_r = (stall) ? (tap_addr_pre << 2) : (tap_addr << 2);
            1'b0:    tap_A_r = (arvalid) ? { {(pADDR_WIDTH-6){1'b0}}, araddr[5:0] } : { {(pADDR_WIDTH-6){1'b0}}, awaddr[5:0] };
            default: tap_A_r = (arvalid) ? { {(pADDR_WIDTH-6){1'b0}}, araddr[5:0] } : { {(pADDR_WIDTH-6){1'b0}}, awaddr[5:0] };
        endcase
    end
    assign tap_A = tap_A_r;

    // only handle RAM and ap_* address
    // can always return config register values,
    // but return 32'hFFFF_FFFF while accessing tap RAM during calculation
    always @(*) begin
        if (araddr == 12'h000) begin
            rdata_r = ap_cfg_regs;
        end else if (araddr == 12'h010) begin
            rdata_r = data_length;
        end else if (state_calc && is_ram_addr_r) begin
            rdata_r = 32'hffff_ffff;
        end else begin // (!state_calc && is_ram_addr_r)
            rdata_r = tap_Do;
        end
    end
    assign rdata = (rdata_valid) ? rdata_r : rdata_buf;

    // AXI Read
    // return data with one cycle latency
    // can receive read address in every cycle

    // arready
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            arready_r <= 1;
        end else begin
            // next cycle for write
            if (!(arvalid && arready) && awvalid && wvalid && !(awready && wready)) begin
                arready_r <= 0;
            // not need to de-assert unless writing
            end else begin
                arready_r <= 1;
            end
        end
    end
    // only when rdata being read, raddr can be accepted
    assign arready = arready_r && ((!rvalid) || (rvalid && rready));

    // rvalid
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            rvalid_r <= 0;
        end else begin
            // when address is accepted
            if (arvalid && arready) begin
                rvalid_r <= 1;
            // data is accepted by host
            end else if (rvalid && rready) begin
                rvalid_r <= 0;
            end
        end
    end
    assign rvalid = rvalid_r;

    // read buffer
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            araddr_buf <= 0;
        end else begin
            if (arvalid && arready)
                araddr_buf <= araddr;
        end
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            rdata_buf <= 0;
        end else begin
            if (rdata_valid)
                rdata_buf <= rdata_r;
        end
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            rdata_valid <= 0;
        end else begin
            rdata_valid <= (arvalid & arready);
        end
    end

    // AXI Write
    // 2 cycles per write operation

    // awready
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            awready_r <= 0;
        end else begin
            // must be de-asserted in next cycle
            if (awvalid && awready) begin
                awready_r <= 0;
            // when both address and data valid, and raddr not accepted
            end else if (!(arvalid && arready) && awvalid && wvalid) begin
                awready_r <= 1;
            end
        end
    end
    assign awready = awready_r;

    // wready 
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            wready_r <= 0;
        end else begin
            // must be de-asserted in next cycle
            if (wvalid && wready) begin
                wready_r <= 0;
            // when both address and data valid, and raddr not accepted
            end else if (!(arvalid && arready) && awvalid && wvalid) begin
                wready_r <= 1;
            end
        end
    end
    assign wready = wready_r;

endmodule