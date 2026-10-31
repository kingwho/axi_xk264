//-------------------------------------------------------------------
// tb_axil_hostile - AXI-Lite read-path regression.
//
// The real Zynq PS / interconnect master only guarantees araddr
// stability until the AR handshake completes; it may change or drop
// araddr afterwards.  This TB trashes araddr right after the AR
// handshake while rvalid/rdata are still pending, and checks that
// every readable register still returns the correct value.
//-------------------------------------------------------------------
`timescale 1ns/1ps

module tb_axil_hostile;

    reg aclk = 0;
    reg aresetn = 0;
    always #5 aclk = ~aclk;

    // AXI-Lite master (hostile on AR)
    reg  [7:0]  l_awaddr = 0;  reg         l_awvalid = 0;
    wire        l_awready;
    reg  [31:0] l_wdata  = 0;  reg         l_wvalid  = 0;
    reg  [3:0]  l_wstrb  = 4'hF;
    wire        l_wready;
    wire [1:0]  l_bresp;       wire        l_bvalid;
    reg         l_bready = 0;
    reg  [7:0]  l_araddr = 0;  reg         l_arvalid = 0;
    wire        l_arready;
    wire [31:0] l_rdata;       wire [1:0]  l_rresp;
    wire        l_rvalid;      reg         l_rready = 0;

    // unused AXI4 master side tie-off (core idle: no frame started)
    wire [31:0] m_axi_araddr, m_axi_awaddr;
    wire [7:0]  m_axi_arlen, m_axi_awlen, m_axi_wstrb;
    wire [2:0]  m_axi_arsize, m_axi_arprot, m_axi_awsize, m_axi_awprot;
    wire [1:0]  m_axi_arburst, m_axi_awburst;
    wire [0:0]  m_axi_arlock, m_axi_arqos_dummy; // placeholder, unused
    wire [3:0]  m_axi_arcache, m_axi_awcache, m_axi_awqos;
    wire [0:0]  m_axi_arid, m_axi_awid, m_axi_rid, m_axi_bid;
    wire [63:0] m_axi_rdata;
    wire        m_axi_arvalid, m_axi_awvalid, m_axi_wvalid, m_axi_wlast;
    wire        m_axi_rlast, m_axi_bvalid;
    wire [1:0]  m_axi_rresp, m_axi_bresp;
    wire        irq;
    wire        tie_hi = 1'b1;

    integer errors = 0;

    xk264_axi_top dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axil_awaddr(l_awaddr), .s_axil_awprot(3'b0), .s_axil_awvalid(l_awvalid),
        .s_axil_awready(l_awready),
        .s_axil_wdata(l_wdata), .s_axil_wstrb(l_wstrb), .s_axil_wvalid(l_wvalid),
        .s_axil_wready(l_wready),
        .s_axil_bresp(l_bresp), .s_axil_bvalid(l_bvalid), .s_axil_bready(l_bready),
        .s_axil_araddr(l_araddr), .s_axil_arprot(3'b0), .s_axil_arvalid(l_arvalid),
        .s_axil_arready(l_arready),
        .s_axil_rdata(l_rdata), .s_axil_rresp(l_rresp),
        .s_axil_rvalid(l_rvalid), .s_axil_rready(l_rready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst), .m_axi_arlock(),
        .m_axi_arcache(m_axi_arcache), .m_axi_arprot(m_axi_arprot), .m_axi_arqos(),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(1'b1),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(1'b0), .m_axi_rready(),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst), .m_axi_awlock(),
        .m_axi_awcache(m_axi_awcache), .m_axi_awprot(m_axi_awprot), .m_axi_awqos(),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(1'b1),
        .m_axi_wdata(), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(1'b1),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(1'b0), .m_axi_bready(tie_hi),
        .irq(irq)
    );

    task axil_write(input [7:0] a, input [31:0] d);
    begin
        @(negedge aclk);
        l_awvalid = 1; l_awaddr = a;
        l_wvalid  = 1; l_wdata  = d;
        while (!(l_awready && l_wready)) @(negedge aclk);
        @(negedge aclk);
        l_awvalid = 0; l_wvalid = 0;
        while (!l_bvalid) @(negedge aclk);
        l_bready = 1;
        @(negedge aclk);
        l_bready = 0;
    end
    endtask

    // hostile read: trash araddr right after the AR handshake, sample rdata
    // at the R handshake posedge (where real hardware samples it)
    task hostile_read(input [7:0] a, input [31:0] expect_v, input [8*24-1:0] name);
    begin
        @(negedge aclk);
        l_arvalid = 1; l_araddr = a;
        @(posedge aclk);                    // AR handshake edge
        @(negedge aclk);
        l_arvalid = 0; l_araddr = 8'hFC;   // unmapped garbage, would hit default -> 0
        l_rready  = 1;
        @(posedge aclk);                    // R handshake edge
        if (l_rdata !== expect_v) begin
            errors = errors + 1;
            $display("FAIL %-24s @%02h: got %08h expect %08h", name, a, l_rdata, expect_v);
        end else
            $display("PASS %-24s @%02h = %08h", name, a, l_rdata);
        @(negedge aclk);
        l_rready = 0;
    end
    endtask

    initial begin
        repeat (5) @(negedge aclk);
        aresetn = 1;
        repeat (2) @(negedge aclk);

        // program some RW registers so readback has nonzero expect values
        axil_write(8'h10, 32'h0000_1234);   // CFG
        axil_write(8'h14, 32'h001E_0028);   // PIC  (40x30 MB)
        axil_write(8'h18, 32'h1000_0000);   // YUV
        axil_write(8'h1C, 32'h2000_0000);   // BANK0
        axil_write(8'h20, 32'h3000_0000);   // BANK1
        axil_write(8'h24, 32'h4000_0000);   // BSADDR

        hostile_read(8'h00, 32'h0264_0001, "REG_IDR");
        hostile_read(8'h08, 32'h0000_0000, "REG_STATUS");
        hostile_read(8'h0C, 32'h0000_0000, "REG_ISTAT");
        hostile_read(8'h10, 32'h0000_1234, "REG_CFG");
        hostile_read(8'h14, 32'h001E_0028, "REG_PIC");
        hostile_read(8'h18, 32'h1000_0000, "REG_YUV");
        hostile_read(8'h1C, 32'h2000_0000, "REG_BANK0");
        hostile_read(8'h20, 32'h3000_0000, "REG_BANK1");
        hostile_read(8'h24, 32'h4000_0000, "REG_BSADDR");
        hostile_read(8'h28, 32'h0000_0000, "REG_BSLEN");
        hostile_read(8'h2C, 32'h0000_0000, "REG_FRMCNT");

        // back-to-back reads: master drives the next address before the
        // first read response completes (two outstanding-capable master)
        @(negedge aclk);
        l_arvalid = 1; l_araddr = 8'h00;
        @(posedge aclk);                    // AR#1 handshake
        @(negedge aclk);
        l_araddr   = 8'h10;                 // next address already on the bus
        l_rready   = 1;
        @(posedge aclk);                    // R#1 handshake, hardware samples here
        if (l_rdata !== 32'h0264_0001) begin
            errors = errors + 1;
            $display("FAIL back-to-back IDR: got %08h expect 02640001", l_rdata);
        end else
            $display("PASS back-to-back IDR  = %08h", l_rdata);
        @(posedge aclk);                    // AR#2 handshake (arready re-asserted)
        @(negedge aclk);
        l_arvalid = 0; l_araddr = 8'hFC;
        @(posedge aclk);                    // R#2 handshake
        if (l_rdata !== 32'h0000_1234) begin
            errors = errors + 1;
            $display("FAIL back-to-back CFG: got %08h expect 00001234", l_rdata);
        end else
            $display("PASS back-to-back CFG  = %08h", l_rdata);
        @(negedge aclk);
        l_rready = 0;

        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("TOTAL ERRORS: %0d", errors);
        $finish;
    end

endmodule
