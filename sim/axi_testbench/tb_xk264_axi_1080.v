//-------------------------------------------------------------------
// tb_xk264_axi_1080 - 1080p (1920x1088, 120x68 MBs) rolling color
// bars through the AXI-wrapped xk264 encoder.
//
// Same DDR slave model and register flow as tb_xk264_axi.v, but:
//  * word-based DDR array (16 MB space) to keep sim memory sane
//  * 8-bar rolling color bar stimulus (YUV420, shifts per frame)
//  * +NFRM plusarg: frame count (default 2: IDR + 1 P)
//  * writes raw per-frame bitstream to bs_frame<N>.bin
//  * no per-2us probe (log size); progress line every PROBE_US
//-------------------------------------------------------------------
`timescale 1ns/1ps

module tb_xk264_axi_1080;

    localparam MX = 120;  // MBs in x  (1920)
    localparam MY = 68;   // MBs in y  (1088, cropped to 1080)

    // DDR layout (16 MB word array, 8-byte words)
    localparam [31:0] A_YUV   = 32'h0010_0000;   // 8160*384  = 0x2FD0C0
    localparam [31:0] A_BANK0 = 32'h0040_0000;   // 8160*512  = 0x3FC000
    localparam [31:0] A_BANK1 = 32'h0080_0000;
    localparam [31:0] A_BS    = 32'h00C0_0000;   // 4 MB space
    localparam integer DDR_WORDS = 2097152;      // 16 MB

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

    localparam integer QP = 28;

    integer nfrm = 2;
    integer frame;

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

    // ---------------- AXI4 DDR slave model (word array) ----------------
    // 64-bit word array; byte strobes applied on write.
    reg [63:0] ddr [0:DDR_WORDS-1];

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
                        rdly    <= 12;
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

    always @(*) m_rdata = ddr[raddr_q[31:3] + rbeat];

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
                        // strobed word write applied by the block below
                        if (m_wlast) begin
                            if (wbeat != {1'b0, wlen_q}) begin
                                $display("TB ERR %0t: WLAST at beat %0d, AWLEN %0d", $time, wbeat, wlen_q);
                                errors = errors + 1;
                            end
                            wdly   <= 10;
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

    // combinational strobed write: apply on the clock edge of acceptance
    integer wi;
    reg [63:0] wr_word_c;
    always @(*) begin
        wr_word_c = ddr[waddr_q[31:3] + wbeat];
        for (wi = 0; wi < 8; wi = wi + 1)
            if (m_wstrb[wi]) wr_word_c[wi*8 +: 8] = m_wdata[wi*8 +: 8];
    end
    always @(posedge aclk) begin
        if (wstate == WR_DATA && m_wvalid && m_wready)
            ddr[waddr_q[31:3] + wbeat] <= wr_word_c;
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

    // ---------------- rolling color bars (BT.601, 75% bars) ----------------
    // 8 vertical bars, 240 luma px / 120 chroma px each; frame N is
    // shifted left by 16 luma px (8 chroma px) -> "rolling".
    function [7:0] bar_y(input integer bar);
        case (bar)
            0: bar_y = 180; 1: bar_y = 162; 2: bar_y = 131; 3: bar_y = 112;
            4: bar_y = 84;  5: bar_y = 65;  6: bar_y = 35;  7: bar_y = 16;
        endcase
    endfunction
    function [7:0] bar_u(input integer bar);
        case (bar)
            0: bar_u = 128; 1: bar_u = 44; 2: bar_u = 160; 3: bar_u = 72;
            4: bar_u = 184; 5: bar_u = 100; 6: bar_u = 212; 7: bar_u = 128;
        endcase
    endfunction
    function [7:0] bar_v(input integer bar);
        case (bar)
            0: bar_v = 128; 1: bar_v = 142; 2: bar_v = 16; 3: bar_v = 58;
            4: bar_v = 198; 5: bar_v = 212; 6: bar_v = 114; 7: bar_v = 128;
        endcase
    endfunction
    function integer bar_idx(input integer x, input integer shift, input integer width);
        integer xx;
    begin
        xx = (x + shift) % width;
        bar_idx = xx / (width / 8);
    end
    endfunction

    // DDR byte access on the word array
    task ddr_wr8(input [31:0] addr, input [7:0] d);
        reg [63:0] wv;
        integer wi8;
    begin
        wi8 = addr[2:0];
        wv = ddr[addr[31:3]];
        wv[wi8*8 +: 8] = d;
        ddr[addr[31:3]] = wv;
    end
    endtask
    function [7:0] ddr_rd8(input [31:0] addr);
        ddr_rd8 = ddr[addr[31:3]][addr[2:0]*8 +: 8];
    endfunction

    task clear_region(input [31:0] base, input integer nbytes);
        integer i;
        reg [7:0] z;
    begin
        z = 8'h00;
        for (i = 0; i < nbytes; i = i + 1)
            ddr_wr8(base + i, z);
    end
    endtask

    // write one frame of rolling bars into A_YUV in the wrapper's
    // MB-tiled beat order (32 luma beats + 16 interleaved UV beats)
    task fill_input(input integer frm);
        integer mbx, mby, t, k, addr, bidx;
        integer lsh, csh;
    begin
        lsh = frm * 16;         // luma shift px
        csh = frm * 8;          // chroma shift px
        for (mby = 0; mby < MY; mby = mby + 1)
        for (mbx = 0; mbx < MX; mbx = mbx + 1) begin
            // luma: beat t -> row t/2, cols (t%2)*8 .. +7
            for (t = 0; t < 32; t = t + 1) begin
                addr = A_YUV + (mby*MX + mbx)*384 + t*8;
                // core consumes beats big-endian: first pixel at DDR addr+7
                for (k = 0; k < 8; k = k + 1) begin
                    bidx = bar_idx(mbx*16 + (t%2)*8 + k, lsh, 1920);
                    ddr_wr8(addr + 7 - k, bar_y(bidx));
                end
            end
            // chroma: beat b -> row b/2, cols (b%2)*4 .. +3, u/v interleaved
            for (t = 0; t < 16; t = t + 1) begin
                addr = A_YUV + (mby*MX + mbx)*384 + 256 + t*8;
                for (k = 0; k < 4; k = k + 1) begin
                    bidx = bar_idx(mbx*8 + (t%2)*4 + k, csh, 960);
                    ddr_wr8(addr + 7 - 2*k, bar_u(bidx));
                    ddr_wr8(addr + 6 - 2*k, bar_v(bidx));
                end
            end
        end
    end
    endtask

    // ---------------- frame helper ----------------
    reg [31:0] rv;
    task run_frame(input intra);
        integer t, frm_idx, f_bs;
        integer bslen;
    begin
        axil_read(R_FRMCNT, rv);
        frm_idx = rv[31:0];
        axil_write(R_ISTAT,  32'h0000_0001);   // clear frame_done
        axil_write(R_CFG,    {1'b0, intra, 6'd28});
        $display("TB %0t: frame %0d start (intra=%0b)", $time, frm_idx, intra);
        axil_write(R_CTRL,   32'h0000_0003);   // start + keep irq_en
        // wait for done (poll; frame takes millions of cycles)
        rv = 0;
        for (t = 0; t < 40000000; t = t + 1) begin
            axil_read(R_STATUS, rv);
            if (rv[1]) t = 32'd40000000;
        end
        if (!rv[1]) begin
            $display("TB ERR %0t: frame timeout (STATUS=%h)", $time, rv);
            errors = errors + 1;
        end
        axil_read(R_BSLEN,  rv);
        bslen = rv[31:0];
        $display("TB %0t: frame %0d done, bslen=%0d", $time, frm_idx, bslen);
        if (bslen < 8 || bslen >= 4*1048576) begin
            $display("TB ERR: bad bitstream length (%0d)", bslen);
            errors = errors + 1;
        end
        // dump raw bitstream for NALU packaging
        f_bs = $fopen($sformatf("bs_frame%0d.bin", frm_idx), "wb");
        for (t = 0; t < bslen; t = t + 1)
            $fwrite(f_bs, "%c", ddr_rd8(A_BS + t));
        $fclose(f_bs);
        $display("TB: dumped %0d bytes to bs_frame%0d.bin", bslen, frm_idx);
    end
    endtask

    integer i;
    initial begin
        if ($value$plusargs("nfrm=%d", nfrm)) ;
        for (i = 0; i < DDR_WORDS; i = i + 1) ddr[i] = 64'h0;
        irq_seen = 0;
    end

    always @(posedge aclk) if (irq) irq_seen = 1;

    // watchdog: generous bound (I ~2.9M cyc, P ~5.8M cyc at 100 MHz)
    initial begin
        #200_000_000;
        $display("TB ERR: global timeout");
        $finish;
    end

    // sparse progress probe (every 500 us)
    initial begin
        forever begin
            #500_000;
            $display("TB %0t: probe fstate=%0d mbx=%0d mby=%0d bs_byt=%0d arv=%b awv=%b",
                $time, dut.fstate,
                dut.u_core.mb_x_load, dut.u_core.mb_y_load,
                dut.bs_byt_cnt_r, dut.m_axi_arvalid, dut.m_axi_awvalid);
        end
    end

    initial begin
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

        for (frame = 0; frame < nfrm; frame = frame + 1) begin
            fill_input(frame);
            clear_region(A_BS, 65536);   // stale bytes would pad the dump
            run_frame(frame == 0);
        end

        axil_read(R_FRMCNT, rv);
        $display("TB: FRMCNT=%0d", rv[31:0]);
        if (!irq_seen) begin
            $display("TB ERR: no interrupt");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB: *** ALL CHECKS PASSED ***");
        else
            $display("TB: *** %0d ERRORS ***", errors);
        $finish;
    end

endmodule
