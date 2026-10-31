//-------------------------------------------------------------------
// tb_xk264_axi - protocol-level testbench for the AXI-wrapped xk264
// encoder (xk264_axi_top).
//
// * AXI-Lite master BFM (tasks axil_write / axil_read)
// * AXI4 slave "DDR" model: unified 64 kB byte array with read/write
//   latency. Because stores and loads go through the same array,
//   frame 1 reconstruction banks feed frame 2 reference fetches.
// * Stimulus: one intra frame, then one inter frame (4x4 MBs, 64x64).
// * Checks: register map, frame completion, bitstream length/content,
//   bank traffic, AXI protocol sanity, no timeout.
//-------------------------------------------------------------------
`timescale 1ns/1ps

module tb_xk264_axi;

    localparam MX = 4;   // MBs in x
    localparam MY = 4;   // MBs in y

    // DDR layout (64 kB space) - regions must not overlap:
    // YUV 16*384 = 0x1800 at 0x1000..0x2800, banks 16*512 = 0x2000 each
    localparam [31:0] A_YUV   = 32'h0000_1000;
    localparam [31:0] A_BANK0 = 32'h0000_4000;
    localparam [31:0] A_BANK1 = 32'h0000_6000;
    localparam [31:0] A_BS    = 32'h0000_8000;

    localparam [7:0] R_IDR    = 8'h00,
                     R_CTRL   = 8'h04,
                     R_STATUS = 8'h08,
                     R_ISTAT  = 8'h0C,
                     R_CFG    = 8'h10,
                     R_PIC    = 8'h14,
                     R_YUV    = 8'h18,
                     R_BANK0  = 8'h1C,
                     R_BANK1  = 8'h20,
                     R_BSADDR = 8'h24,
                     R_BSLEN  = 8'h28,
                     R_FRMCNT = 8'h2C;

    reg aclk = 0, aresetn = 0;
    always #5 aclk = ~aclk;

    integer errors = 0;

    // ---------------- AXI-Lite master ----------------
    reg         l_awvalid = 0;
    wire        l_awready;
    reg  [7:0]  l_awaddr = 0;
    reg         l_wvalid = 0;
    wire        l_wready;
    reg  [31:0] l_wdata = 0;
    reg  [3:0]  l_wstrb = 0;
    wire        l_bvalid;
    reg         l_bready = 0;
    reg  [7:0]  l_araddr = 0;
    reg         l_arvalid = 0;
    wire        l_arready, l_rvalid;
    reg         l_rready = 0;
    wire [31:0] l_rdata;

    task axil_write(input [7:0] a, input [31:0] d);
    begin
        @(negedge aclk);
        l_awvalid = 1; l_awaddr = a;
        l_wvalid  = 1; l_wdata  = d; l_wstrb = 4'hF;
        while (!(l_awready && l_wready)) @(negedge aclk);
        @(negedge aclk);
        l_awvalid = 0; l_wvalid = 0;
        while (!l_bvalid) @(negedge aclk);
        l_bready = 1;
        @(negedge aclk);
        l_bready = 0;
    end
    endtask

    task axil_read(input [7:0] a, output [31:0] d);
    begin
        @(negedge aclk);
        l_arvalid = 1; l_araddr = a;
        while (!l_arready) @(negedge aclk);
        @(negedge aclk);
        l_arvalid = 0;
        while (!l_rvalid) @(negedge aclk);
        d = l_rdata;
        l_rready = 1;
        @(negedge aclk);
        l_rready = 0;
    end
    endtask

    // ---------------- AXI4 DDR slave model ----------------
    reg [7:0] ddr [0:65535];
    // 64 kB memory space used by this test.

    // read channel FSM
    localparam RD_S0 = 0, RD_WAIT = 1, RD_BEAT = 2;
    reg [1:0]  rstate = RD_S0;
    reg [31:0] raddr_q = 0;
    reg [7:0]  rlen_q = 0;
    reg [8:0]  rbeat = 0;
    reg [7:0]  rdly = 0;
    reg        m_rvalid = 0;
    reg [63:0] m_rdata = 0;
    wire       m_arvalid, m_arready;
    wire [31:0] m_araddr;
    wire [7:0]  m_arlen;
    wire       m_rlast = (rbeat == {1'b0, rlen_q}) && (rstate == RD_BEAT);

    assign m_arready = (rstate == RD_S0);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rstate <= RD_S0; m_rvalid <= 0; rdly <= 0;
        end else begin
            case (rstate)
                RD_S0: begin
                    if (m_arvalid) begin
                        if (m_araddr[2:0] != 3'b0) begin
                            $display("TB ERR %0t: AR addr %h not 8-byte aligned", $time, m_araddr);
                            errors = errors + 1;
                        end
                        if (m_arlen > 8'd15) begin
                            $display("TB ERR %0t: ARLEN %0d > 15", $time, m_arlen);
                            errors = errors + 1;
                        end
                        raddr_q <= m_araddr;
                        rlen_q  <= m_arlen;
                        rbeat   <= 0;
                        rdly    <= 12;          // read latency
                        rstate  <= RD_WAIT;
                    end
                end
                RD_WAIT: begin
                    if (rdly == 0) begin
                        m_rvalid <= 1;
                        rstate   <= RD_BEAT;
                    end else
                        rdly <= rdly - 1;
                end
                RD_BEAT: begin
                    if (m_rvalid && m_axi_rready) begin
                        rbeat <= rbeat + 1;
                        if (rbeat == {1'b0, rlen_q}) begin
                            m_rvalid <= 0;
                            rstate   <= RD_S0;
                        end
                    end
                end
                default: rstate <= RD_S0;
            endcase
        end
    end

    // read data mux (combinational, per beat)
    always @(*) begin
        m_rdata = 64'b0;
        m_rdata[63:56] = ddr[raddr_q + 7 + rbeat*8];
        m_rdata[55:48] = ddr[raddr_q + 6 + rbeat*8];
        m_rdata[47:40] = ddr[raddr_q + 5 + rbeat*8];
        m_rdata[39:32] = ddr[raddr_q + 4 + rbeat*8];
        m_rdata[31:24] = ddr[raddr_q + 3 + rbeat*8];
        m_rdata[23:16] = ddr[raddr_q + 2 + rbeat*8];
        m_rdata[15:8]  = ddr[raddr_q + 1 + rbeat*8];
        m_rdata[7:0]   = ddr[raddr_q     + rbeat*8];
    end

    // write channel FSM
    localparam WR_S0 = 0, WR_DATA = 1, WR_BLAT = 2, WR_B = 3;
    reg [1:0]  wstate = WR_S0;
    reg [31:0] waddr_q = 0;
    reg [7:0]  wlen_q = 0;
    reg [8:0]  wbeat = 0;
    reg [7:0]  wdly = 0;
    reg        m_bvalid = 0;
    wire       m_awvalid, m_awready, m_wvalid, m_wready, m_wlast;
    wire [31:0] m_awaddr;
    wire [7:0]  m_awlen;
    wire [63:0] m_wdata;
    wire [7:0]  m_wstrb;

    assign m_awready = (wstate == WR_S0);
    assign m_wready  = (wstate == WR_DATA);

    integer wi;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wstate <= WR_S0; m_bvalid <= 0; wdly <= 0;
        end else begin
            case (wstate)
                WR_S0: begin
                    if (m_awvalid) begin
                        if (m_awaddr[2:0] != 3'b0) begin
                            $display("TB ERR %0t: AW addr %h not 8-byte aligned", $time, m_awaddr);
                            errors = errors + 1;
                        end
                        if (m_awlen > 8'd15) begin
                            $display("TB ERR %0t: AWLEN %0d > 15", $time, m_awlen);
                            errors = errors + 1;
                        end
                        waddr_q <= m_awaddr;
                        wlen_q  <= m_awlen;
                        wbeat   <= 0;
                        wstate  <= WR_DATA;
                    end
                end
                WR_DATA: begin
                    if (m_wvalid && m_wready) begin
                        for (wi = 0; wi < 8; wi = wi + 1)
                            if (m_wstrb[wi])
                                ddr[waddr_q + wbeat*8 + wi] <= m_wdata[wi*8 +: 8];
                        if (m_wlast) begin
                            if (wbeat != {1'b0, wlen_q}) begin
                                $display("TB ERR %0t: WLAST at beat %0d, AWLEN %0d", $time, wbeat, wlen_q);
                                errors = errors + 1;
                            end
                            wdly   <= 10;       // b latency
                            wstate <= WR_BLAT;
                        end else
                            wbeat <= wbeat + 1;
                    end
                end
                WR_BLAT: begin
                    if (wdly == 0) begin
                        m_bvalid <= 1;
                        wstate   <= WR_B;
                    end else
                        wdly <= wdly - 1;
                end
                WR_B: begin
                    if (m_axi_bready) begin
                        m_bvalid <= 0;
                        wstate   <= WR_S0;
                    end
                end
                default: wstate <= WR_S0;
            endcase
        end
    end

    wire irq;
    reg  irq_seen;

    xk264_axi_top dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axil_awaddr(l_awaddr), .s_axil_awprot(3'b0), .s_axil_awvalid(l_awvalid),
        .s_axil_awready(l_awready),
        .s_axil_wdata(l_wdata), .s_axil_wstrb(l_wstrb), .s_axil_wvalid(l_wvalid),
        .s_axil_wready(l_wready),
        .s_axil_bresp(), .s_axil_bvalid(l_bvalid), .s_axil_bready(l_bready),
        .s_axil_araddr(l_araddr), .s_axil_arprot(3'b0), .s_axil_arvalid(l_arvalid),
        .s_axil_arready(l_arready),
        .s_axil_rdata(l_rdata), .s_axil_rresp(), .s_axil_rvalid(l_rvalid),
        .s_axil_rready(l_rready),
        .m_axi_arid(), .m_axi_araddr(m_araddr), .m_axi_arlen(m_arlen),
        .m_axi_arsize(), .m_axi_arburst(), .m_axi_arlock(), .m_axi_arcache(),
        .m_axi_arprot(), .m_axi_arqos(), .m_axi_arvalid(m_arvalid),
        .m_axi_arready(m_arready),
        .m_axi_rid(), .m_axi_rdata(m_rdata), .m_axi_rresp(2'b00),
        .m_axi_rlast(m_rlast), .m_axi_rvalid(m_rvalid), .m_axi_rready(m_axi_rready),
        .m_axi_awid(), .m_axi_awaddr(m_awaddr), .m_axi_awlen(m_awlen),
        .m_axi_awsize(), .m_axi_awburst(), .m_axi_awlock(), .m_axi_awcache(),
        .m_axi_awprot(), .m_axi_awqos(), .m_axi_awvalid(m_awvalid),
        .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wlast(m_wlast),
        .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bid(), .m_axi_bresp(2'b00), .m_axi_bvalid(m_bvalid),
        .m_axi_bready(m_axi_bready),
        .irq(irq)
    );

    // ---------------- input frame generation ----------------
    function [7:0] luma_val(input integer fx, input integer fy, input integer frame);
        luma_val = (fx*3 + fy*5 + frame*23 + ((fx*fy) & 15)) & 8'hFF;
    endfunction
    function [7:0] chr_val(input integer fx, input integer fy, input integer frame, input integer uv);
        chr_val = (fx*7 + fy*11 + frame*31 + uv*51) & 8'hFF;
    endfunction

    // dump per-MB recon words (48 x 64-bit per MB) from a bank region
    task dump_bank_words(input [31:0] base);
        integer dbm, dbw, dbk, dbf;
        reg [63:0] dbword;
    begin
        dbf = $fopen("axi_bank0.dat", "wb");
        for (dbm = 0; dbm < 16; dbm = dbm + 1)
            for (dbw = 0; dbw < 48; dbw = dbw + 1) begin
                dbword = 64'b0;
                for (dbk = 7; dbk >= 0; dbk = dbk - 1)
                    dbword = {dbword[55:0], ddr[base + dbm*512 + dbw*8 + dbk]};
                $fwrite(dbf, "%0d %0d %016h\n", dbm, dbw, dbword);
            end
        $fclose(dbf);
    end
    endtask

    // write one frame image into DDR at A_YUV in the wrapper's
    // MB-tiled beat order (32 luma beats + 16 interleaved UV beats)
    task fill_input(input integer frame, input integer dx, input integer dy);
        integer mbx, mby, t, px, py, k, bidx, addr, v;
    begin
        for (mby = 0; mby < MY; mby = mby + 1)
        for (mbx = 0; mbx < MX; mbx = mbx + 1) begin
            // luma: beat t -> row t/2, cols (t%2)*8 .. +7
            // the core consumes the 64-bit beat with the first pixel at
            // bits [63:56] (big-endian), i.e. DDR byte addr+7 holds the
            // first pixel of the beat
            for (t = 0; t < 32; t = t + 1) begin
                py = t / 2;
                addr = A_YUV + (mby*MX + mbx)*384 + t*8;
                for (k = 0; k < 8; k = k + 1) begin
                    px = (t % 2)*8 + k;
                    v = luma_val(mbx*16+px+dx, mby*16+py+dy, frame);
                    ddr[addr + 7 - k] = v[7:0];
                end
            end
            // chroma: beat b -> row b/2, cols (b%2)*4 .. +3, u/v interleaved
            for (t = 0; t < 16; t = t + 1) begin
                py = t / 2;
                addr = A_YUV + (mby*MX + mbx)*384 + 256 + t*8;
                for (k = 0; k < 4; k = k + 1) begin
                    px = (t % 2)*4 + k;
                    v = chr_val(mbx*8+px+dx/2, mby*8+py+dy/2, frame, 0);
                    ddr[addr + 7 - 2*k] = v[7:0];
                    v = chr_val(mbx*8+px+dx/2, mby*8+py+dy/2, frame, 1);
                    ddr[addr + 6 - 2*k] = v[7:0];
                end
            end
        end
    end
    endtask

    task clear_region(input [31:0] base, input integer nbytes);
        integer i;
    begin
        for (i = 0; i < nbytes; i = i + 1)
            ddr[base + i] = 8'h00;
    end
    endtask

    task count_nonzero(input [31:0] base, input integer nbytes, output integer n);
        integer i;
    begin
        n = 0;
        for (i = 0; i < nbytes; i = i + 1)
            if (ddr[base + i] !== 8'h00) n = n + 1;
    end
    endtask

    // ---------------- bitstream dump (+dumpbs) ----------------
    // Writes each frame's raw encoder output (MB-layer RBSP, no NAL
    // headers) to bs_frame<N>.bin for post-processing.
    integer dump_bs;
    integer f_bs;
    initial dump_bs = $test$plusargs("dumpbs");

    task dump_frame_bs(input integer frame, input [31:0] len);
        integer b;
    begin
        if (dump_bs) begin
            f_bs = $fopen($sformatf("bs_frame%0d.bin", frame), "wb");
            for (b = 0; b < len; b = b + 1)
                $fwrite(f_bs, "%c", ddr[A_BS + b]);
            $fclose(f_bs);
            $display("TB: dumped %0d bytes to bs_frame%0d.bin", len, frame);
        end
    end
    endtask

    // ---------------- frame helper ----------------
    reg [31:0] rv;
    task run_frame(input intra, input integer dx, input integer dy);
        integer nz;
        integer t;
        integer frm_idx;
    begin
        frm_idx = 0;
        axil_read(R_FRMCNT, rv);
        frm_idx = rv[31:0];
        axil_write(R_ISTAT,  32'h0000_0001);   // clear frame_done
        axil_write(R_CFG,    {1'b0, intra, 6'd28});
        $display("TB %0t: frame start (intra=%0b)", $time, intra);
        axil_write(R_CTRL,   32'h0000_0003);   // start + keep irq_en
        // wait for done
        for (t = 0; t < 2000000; t = t + 1) begin
            axil_read(R_STATUS, rv);
            if (rv[1]) t = 32'd2000000; // break
        end
        if (!rv[1]) begin
            $display("TB ERR %0t: frame timeout (STATUS=%h)", $time, rv);
            errors = errors + 1;
        end
        axil_read(R_BSLEN,  rv);
        $display("TB: frame done, bslen=%0d", rv[31:0]);
        if (rv < 8) begin
            $display("TB ERR: bitstream too short (%0d)", rv[31:0]);
            errors = errors + 1;
        end
        count_nonzero(A_BS, 256, nz);
        if (nz < 8) begin
            $display("TB ERR: bitstream region empty (nz=%0d)", nz);
            errors = errors + 1;
        end
        dump_frame_bs(frm_idx, rv[31:0]);
    end
    endtask

    integer i, nz;
    initial begin
        for (i = 0; i < 65536; i = i + 1) ddr[i] = 8'h00;
        irq_seen = 0;
    end

    // irq capture
    always @(posedge aclk) if (irq) irq_seen = 1;

    // watchdog
    initial begin
        #200_000;
        $display("TB ERR: global timeout");
        $finish;
    end

    // periodic state probe (every 2 us) to see whether the run advances
    initial begin
        forever begin
            #2000;
            $display("TB %0t: probe fstate=%0d mbx=%0d mby=%0d tc=%0d ecnt=%0d incnt=%0d arv=%b awv=%b eS=%0d ld=%b fd=%b id=%b ed=%b dd=%b fdbS=%0d sef=%b scnt=%0d ma=%0d dbS=%0d ecS=%0d fld=%b fcd=%b fS=%0d freq=%b fec=%0d fx=%0d fxn=%0d fy=%0d fyn=%0d imS=%0d imN=%0d imd=%b fmd=%b mcd=%b mrun=%b mcnt=%0d tqc=%0d p16=%b chen=%b crv=%b crn=%0d mce=%b mca=%b is=%b idn=%b es=%b edn=%b ds=%b ddn=%b",
                $time, dut.fstate,
                dut.u_core.mb_x_load, dut.u_core.mb_y_load,
                dut.u_core.u_top_ctrl.curr_state, dut.u_core.u_top_ctrl.mb_done_flag,
                dut.in_count,
                dut.m_axi_arvalid, dut.m_axi_awvalid,
                dut.u_ext_ctrl.state,
                dut.u_core.u_top_ctrl.load_done_flag,
                dut.u_core.u_top_ctrl.fetch_db_done_flag,
                dut.u_core.u_top_ctrl.intra_done_flag,
                dut.u_core.u_top_ctrl.ec_done_flag,
                dut.u_core.u_top_ctrl.db_done_flag,
                dut.u_core.u_fetch_db.curr_state,
                dut.u_core.u_fetch_db.store_en_flag,
                dut.u_core.u_fetch_db.store_cnt,
                dut.u_core.u_mem_arb.curr_state,
                dut.u_core.u_db_top.contol.state,
                dut.u_core.u_cavlc_top.control_state,
                dut.u_core.u_top_ctrl.fetch_luma_done_flag, dut.u_core.u_top_ctrl.fetch_chroma_done_flag,
                dut.u_core.u_top_ctrl.ime_done_flag, dut.u_core.u_top_ctrl.fme_done_flag, dut.u_core.u_top_ctrl.mc_done_flag,
                dut.u_core.fetch.u_fetch_luma.curr_state, dut.u_core.fetch.u_fetch_luma.ext_load_req_r, dut.u_core.fetch.u_fetch_luma.ext_cst,
                dut.u_core.fetch.u_fetch_luma.ld_x_cnt_r, dut.u_core.fetch.u_fetch_luma.ld_x_num_r,
                dut.u_core.fetch.u_fetch_luma.ld_y_cnt_r, dut.u_core.fetch.u_fetch_luma.ld_y_num_r,
                dut.u_core.u_ime.u_ime_ctrl.curr_state, dut.u_core.u_ime.u_ime_ctrl.next_state,
                
                dut.u_core.u_mc.mc_run_r, dut.u_core.u_mc.mc_cnt_r, dut.u_core.u_mc.tq_cnt_r,
                dut.u_core.u_mc.tq_p16x16_en_o, dut.u_core.u_mc.tq_chroma_en_o,
                dut.u_core.tq_cr_val, dut.u_core.tq_cr_num,
                dut.u_core.mc_chroma_ld_en, dut.u_core.mc_chroma_ld_ack,
                dut.u_core.intra_start, dut.u_core.intra_done,
                dut.u_core.ec_start, dut.u_core.ec_done,
                dut.u_core.db_start, dut.u_core.db_done);
        end
    end

    // one-shot detailed dump when the inter frame stalls
    initial begin
        #80_000;
        $display("DUMP mc_chroma_top: state=%0d row_cn=%0d col_cn=%0d part_cn=%0d subpart_cn=%0d chroma_ip=%0b uvsw=%0b limit=%0d wm=%0b pixel_x=%0d pixel_y=%0d end_onerow=%b next_row=%b end_oneblk=%b end_onemb=%b mc_pixel=%0d chroma_wren=%b wr_cnt=%0b frac_v=%b input_v=%b x_mod8=%0d cnt_data=%0d",
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.state,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.row_cn,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.col_cn,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.part_cn_rd,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.subpart_cn_rd,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.chroma_ip,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.uv_blk2xn_switch,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.limit,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.working_mode,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.pixel_x,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.pixel_y,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.end_onerow,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.next_row,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.end_oneblk,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.end_onemb,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.mc_pixel,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.chroma_wren_o,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.wr_cnt,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.frac_chroma_v,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.input_v,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.x_mod8,
            dut.u_core.u_mc.u_mc_core.mc_chroma_top.cnt_data);
        $display("DUMP mc_core: state=%0d luma_rdy=%b chroma_rdy=%b luma_wren=%b wr_cnt=%0d wr_cnt_tmp=%b pred_rdy=%b pred_addr=%0d end_luma=%b end_chroma=%b launch_chroma=%b",
            dut.u_core.u_mc.u_mc_core.state,
            dut.u_core.u_mc.u_mc_core.luma_rdy,
            dut.u_core.u_mc.u_mc_core.chroma_rdy,
            dut.u_core.u_mc.u_mc_core.luma_wren,
            dut.u_core.u_mc.u_mc_core.wr_cnt,
            dut.u_core.u_mc.u_mc_core.wr_cnt_tmp,
            dut.u_core.u_mc.u_mc_core.mc_pred_rdy_o,
            dut.u_core.u_mc.u_mc_core.mc_pred_addr,
            dut.u_core.u_mc.u_mc_core.end_onemb_luma,
            dut.u_core.u_mc.u_mc_core.end_onemb_chroma,
            dut.u_core.u_mc.u_mc_core.launch_chroma);
        $display("DUMP bw: bw_state=%0d bw_active=%b bs_wr_out=%b bs_flushed=%b tail_valid=%b tail_strb=%h bs_nb=%0d wf_cnt=%0d wf_empty=%b wf_full=%b bsf_cnt=%0d bsf_empty=%b bs_byt=%0d bsw_valid=%b bsw_accept=%b bsw_done=%b frame_done_core=%b",
            dut.bw_state, dut.bw_active_r, dut.bs_wr_out_r, dut.bs_flushed_r,
            dut.tail_valid_r, dut.tail_strb_r, dut.bs_nb_r,
            dut.wf_count, dut.wf_empty, dut.wf_full,
            dut.bsf_count, dut.bsf_empty, dut.bs_byt_cnt_r,
            dut.bsw_cmd_valid, dut.bsw_cmd_accept, dut.bsw_done,
            dut.u_core.frame_done);
        $display("DUMP fme->mc: fmv_i=%h imv_i=%h mbtype_info=%h fmv_r0=%h fmv_r1=%h fmv_r2=%h fmv_r3=%h fmv_r4=%h fmv_r5=%h fmv_r6=%h fmv_r7=%h fmv_r8=%h fmv_r9=%h fmv_r10=%h fmv_r11=%h fmv_r12=%h fmv_r13=%h fmv_r14=%h fmv_r15=%h",
            dut.u_core.u_mc.fmeif_fmv_i, dut.u_core.u_mc.fmeif_imv_i,
            dut.u_core.u_mc.mb_type_info,
            dut.u_core.u_mc.u_mc_core.fmv_r[0], dut.u_core.u_mc.u_mc_core.fmv_r[1],
            dut.u_core.u_mc.u_mc_core.fmv_r[2], dut.u_core.u_mc.u_mc_core.fmv_r[3],
            dut.u_core.u_mc.u_mc_core.fmv_r[4], dut.u_core.u_mc.u_mc_core.fmv_r[5],
            dut.u_core.u_mc.u_mc_core.fmv_r[6], dut.u_core.u_mc.u_mc_core.fmv_r[7],
            dut.u_core.u_mc.u_mc_core.fmv_r[8], dut.u_core.u_mc.u_mc_core.fmv_r[9],
            dut.u_core.u_mc.u_mc_core.fmv_r[10], dut.u_core.u_mc.u_mc_core.fmv_r[11],
            dut.u_core.u_mc.u_mc_core.fmv_r[12], dut.u_core.u_mc.u_mc_core.fmv_r[13],
            dut.u_core.u_mc.u_mc_core.fmv_r[14], dut.u_core.u_mc.u_mc_core.fmv_r[15]);
        $display("DUMP fetch_chroma: ram_wren_lb=%b ram_wren_rb=%b ld_state=%0d",
            dut.u_core.u_fetch_chroma.ram_wren_lb,
            dut.u_core.u_fetch_chroma.ram_wren_rb,
            dut.u_core.u_fetch_chroma.current_state);
        $display("DUMP fetch_luma: sys_load_i=%b load_r=%b curr_state=%0d ext_cst=%b ext_start_o=%b ext_done_i=%b ext_valid_i=%b ext_load_req=%b",
            dut.u_core.fetch.u_fetch_luma.sys_load_i,
            dut.u_core.fetch.u_fetch_luma.load_r,
            dut.u_core.fetch.u_fetch_luma.curr_state,
            dut.u_core.fetch.u_fetch_luma.ext_cst,
            dut.u_core.fetch.u_fetch_luma.ext_start_o,
            dut.u_core.fetch.u_fetch_luma.ext_done_i,
            dut.u_core.fetch.u_fetch_luma.ext_valid_i,
            dut.u_core.fetch.u_fetch_luma.ext_load_req_r);
    end

    // event trace of bitstream write bursts (+trace to enable)
    reg trace_en;
    initial trace_en = $test$plusargs("trace");
    always @(posedge aclk) if (trace_en && dut.fstate != 0) begin
        if (dut.bsw_cmd_accept)
            $display("TRC %0t: BSW accept beats=%0d strb=%h wf=%0d state=%0d",
                $time, dut.bsw_cmd_beats, dut.bsw_cmd_wstrb, dut.wf_count, dut.bw_state);
        if (dut.bsw_wfire)
            $display("TRC %0t: BSW wfire beats_q=%0d wf=%0d", $time, dut.bw_beats_q, dut.wf_count);
        if (dut.bsw_done)
            $display("TRC %0t: BSW done active=%b beats_q=%0d wf=%0d", $time, dut.bw_active_r, dut.bw_beats_q, dut.wf_count);
        if (dut.wf_tail_push_r)
            $display("TRC %0t: TAIL push nb=%0d strb=%h", $time, dut.bs_nb_r, dut.tail_strb_r);
        // write arbiter internals
        if (dut.u_wr_arb.state != 0 || dut.m_axi_wvalid)
            if (dut.u_wr_arb.wfire || dut.u_wr_arb.state == 3 || dut.m_axi_awvalid)
                $display("TRC %0t: ARB st=%0d own=%b beatcnt=%0d beatsq=%0d wv=%b wl=%b c1wv=%b wf=%0d",
                    $time, dut.u_wr_arb.state, dut.u_wr_arb.owner,
                    dut.u_wr_arb.beat_cnt, dut.u_wr_arb.beats_q,
                    dut.m_axi_wvalid, dut.m_axi_wlast,
                    dut.bw_c1_wvalid, dut.wf_count);
    end

    initial begin
        // VCD disabled by default (slow with the DDR model); enable with +vcd
        if ($test$plusargs("vcd")) begin
            $dumpfile("/tmp/tb_xk264_axi.vcd");
            $dumpvars(0, tb_xk264_axi);
        end

        aresetn = 0;
        repeat (10) @(negedge aclk);
        aresetn = 1;
        repeat (5) @(negedge aclk);

        // --- register map check ---
        axil_read(R_IDR, rv);
        if (rv !== 32'h0264_0001) begin
            $display("TB ERR: IDR=%h", rv);
            errors = errors + 1;
        end

        // --- program ---
        axil_write(R_PIC,    (MY << 16) | MX);
        axil_write(R_YUV,    A_YUV);
        axil_write(R_BANK0,  A_BANK0);
        axil_write(R_BANK1,  A_BANK1);
        axil_write(R_BSADDR, A_BS);
        axil_write(R_CTRL,   32'h0000_0002);   // irq enable

        // --- frame 0: intra ---
        fill_input(0, 0, 0);
        clear_region(A_BANK0, 16*512);
        clear_region(A_BANK1, 16*512);
        clear_region(A_BS, 4096);
        run_frame(1, 0, 0);
        count_nonzero(A_BANK0, 16*512, nz);
        $display("TB: bank0 nonzero bytes = %0d", nz);
        if (nz < 16*384/2) begin
            $display("TB ERR: bank0 reconstruction looks empty");
            errors = errors + 1;
        end
        axil_read(R_FRMCNT, rv);
        if (rv[31:0] !== 32'd1) begin
            $display("TB ERR: FRMCNT=%0d", rv[31:0]);
            errors = errors + 1;
        end
        if (!irq_seen) begin
            $display("TB ERR: no interrupt after frame");
            errors = errors + 1;
        end

        // --- dump bank0 recon words (differential vs golden ref_mem) ---
        dump_bank_words(A_BANK0);

        // --- frame 1: inter (reference = bank0 via cur_bank flip) ---
        fill_input(1, 2, 1);
        run_frame(0, 2, 1);
        count_nonzero(A_BANK1, 16*512, nz);
        $display("TB: bank1 nonzero bytes = %0d", nz);
        if (nz < 16*384/2) begin
            $display("TB ERR: bank1 reconstruction looks empty");
            errors = errors + 1;
        end

        axil_read(R_FRMCNT, rv);
        $display("TB: FRMCNT=%0d", rv[31:0]);

        // dump first bitstream bytes
        $write("TB: BS:");
        for (i = 0; i < 32; i = i + 1) $write(" %02x", ddr[A_BS + i]);
        $write("\n");

        if (errors == 0)
            $display("TB: *** ALL CHECKS PASSED ***");
        else
            $display("TB: *** %0d ERRORS ***", errors);
        $finish;
    end


// ---- ext load replay beat log (differential vs golden) ----
integer f_el;
initial f_el = $fopen("ext_load_axi.dat","wb");
always @(posedge aclk) begin
    if (dut.ext_wen)
        $fwrite(f_el, "F%0d L %b %0d %0d %h\n", dut.frm_cnt_r, dut.ext_mode, dut.ext_mbx, dut.ext_mby, dut.ext_wdata);
end

// ---- db_rec RAM write/read event log (differential vs golden) ----
integer f_dw;
initial f_dw = $fopen("dbrec_axi.dat","wb");
always @(posedge aclk) begin
    if (dut.u_core.u_fetch_db.u_db_rec_128x64.we)
        $fwrite(f_dw, "%0t W %b %0d %h\n", $time,
                dut.u_core.u_fetch_db.u_db_rec_128x64.waddr[5],
                dut.u_core.u_fetch_db.u_db_rec_128x64.waddr[4:0],
                dut.u_core.u_fetch_db.u_db_rec_128x64.wdata);
    if (dut.u_core.u_fetch_db.u_db_rec_128x64.rd)
        $fwrite(f_dw, "%0t R %b %0d\n", $time,
                dut.u_core.u_fetch_db.u_db_rec_128x64.raddr[5],
                dut.u_core.u_fetch_db.u_db_rec_128x64.raddr[4:0]);
end

// ---- ext store capture log (differential vs golden) ----
integer f_es;
initial f_es = $fopen("ext_store_axi.dat","wb");
always @(posedge aclk) begin
    if (dut.u_ext_ctrl.cap_vld_q)
        $fwrite(f_es, "F%0d M%b %0d %0d A%0d %h\n", dut.frm_cnt_r, dut.ext_mode,
                dut.ext_mbx, dut.ext_mby, dut.u_ext_ctrl.cap_addr_q, dut.ext_rdata);
end

// ---- ext transaction stream log (differential vs golden) ----
integer f_xt; reg xt_q;
initial begin f_xt = $fopen("ext_txn_axi.dat","wb"); xt_q = 1'b0; end
always @(posedge aclk) begin
    xt_q <= dut.ext_start;
    if (dut.ext_start && !xt_q)
        $fwrite(f_xt, "F%0d S %b %0d %0d\n", dut.frm_cnt_r, dut.ext_mode, dut.ext_mbx, dut.ext_mby);
end

// ---- differential debug dumps: per-MB bitstream offsets + loaded pixels ----
integer f_mboff;
initial f_mboff = $fopen("mb_offset_axi.dat","wb");
always @(posedge aclk) begin
    if (dut.u_core.u_cavlc_top.start)
        $fwrite(f_mboff, "%0d %0d %0d\n", dut.u_core.u_cavlc_top.mb_x, dut.u_core.u_cavlc_top.mb_y, dut.u_core.u_bs_buf.bs_cnt_w);
    if (dut.u_core.frame_done)
        $fwrite(f_mboff, "END %0d\n", dut.u_core.u_bs_buf.bs_cnt_w);
end

integer f_cmb, ci, cj;
initial f_cmb = $fopen("cur_mb_axi.dat","wb");
always @(posedge aclk) begin
    if (dut.u_core.load_done) begin
        $fwrite(f_cmb, "\nMB_X =%0d, MB_Y =%0d\n", dut.u_core.mb_x_load, dut.u_core.mb_y_load);
        $fwrite(f_cmb, "==Y==\n");
        for (ci = 0; ci < 16; ci = ci + 1) begin
            for (cj = 0; cj < 16; cj = cj + 1)
                $fwrite(f_cmb, "%h ", dut.u_core.u_cur_mb.cur_y[16*ci+cj]);
            $fwrite(f_cmb, "\n");
        end
        $fwrite(f_cmb, "==U==\n");
        for (ci = 0; ci < 8; ci = ci + 1) begin
            for (cj = 0; cj < 8; cj = cj + 1)
                $fwrite(f_cmb, "%h ", dut.u_core.u_cur_mb.cur_u[8*ci+cj]);
            $fwrite(f_cmb, "\n");
        end
        $fwrite(f_cmb, "==V==\n");
        for (ci = 0; ci < 8; ci = ci + 1) begin
            for (cj = 0; cj < 8; cj = cj + 1)
                $fwrite(f_cmb, "%h ", dut.u_core.u_cur_mb.cur_v[8*ci+cj]);
            $fwrite(f_cmb, "\n");
        end
    end
end
endmodule
