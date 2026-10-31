//-------------------------------------------------------------------
// xk264_axi_top - AXI wrapper for the xk264 H.264 (baseline) encoder.
//
// * S_AXI_LITE: control/status register file (see README for map)
// * M_AXI (AXI4, 64-bit): DDR access
//     - read : input frame (MB-tiled 4:2:0 stream, 384 B/MB),
//              reference/reconstruction frame banks (512 B/MB tiled)
//     - write: reconstruction banks, H.264 byte stream
// * irq: frame-complete interrupt
//
// Input frame format (per macroblock, 384 B, MB raster order):
//   32 beats * 8 luma pixels (row-major, MSB = leftmost pixel),
//   16 beats * 8 bytes with U/V byte-interleaved (u0 v0 u1 v1 ...),
//   chroma beat b covers row b/2, columns 4*(b%2) .. +3.
//
// All buffers must be at least 4 kB aligned in DDR.
//-------------------------------------------------------------------
module xk264_axi_top (
    input  wire         aclk,
    input  wire         aresetn,

    // AXI4-Lite slave
    input  wire [7:0]   s_axil_awaddr,
    input  wire [2:0]   s_axil_awprot,
    input  wire         s_axil_awvalid,
    output wire         s_axil_awready,
    input  wire [31:0]  s_axil_wdata,
    input  wire [3:0]   s_axil_wstrb,
    input  wire         s_axil_wvalid,
    output wire         s_axil_wready,
    output wire [1:0]   s_axil_bresp,
    output wire         s_axil_bvalid,
    input  wire         s_axil_bready,
    input  wire [7:0]   s_axil_araddr,
    input  wire [2:0]   s_axil_arprot,
    input  wire         s_axil_arvalid,
    output wire         s_axil_arready,
    output wire [31:0]  s_axil_rdata,
    output wire [1:0]   s_axil_rresp,
    output wire         s_axil_rvalid,
    input  wire         s_axil_rready,

    // AXI4 master, 64-bit
    output wire [0:0]   m_axi_arid,
    output wire [31:0]  m_axi_araddr,
    output wire [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output wire [0:0]   m_axi_arlock,
    output wire [3:0]   m_axi_arcache,
    output wire [2:0]   m_axi_arprot,
    output wire [3:0]   m_axi_arqos,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [0:0]   m_axi_rid,
    input  wire [63:0]  m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,
    output wire [0:0]   m_axi_awid,
    output wire [31:0]  m_axi_awaddr,
    output wire [7:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,
    output wire [0:0]   m_axi_awlock,
    output wire [3:0]   m_axi_awcache,
    output wire [2:0]   m_axi_awprot,
    output wire [3:0]   m_axi_awqos,
    output wire         m_axi_awvalid,
    input  wire         m_axi_awready,
    output wire [63:0]  m_axi_wdata,
    output wire [7:0]   m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,
    input  wire [0:0]   m_axi_bid,
    input  wire [1:0]   m_axi_bresp,
    input  wire         m_axi_bvalid,
    output wire         m_axi_bready,

    output wire         irq
);

    // --------------------------------------------------------------
    // register map
    // --------------------------------------------------------------
    localparam [7:0] REG_IDR    = 8'h00,  // ro 0x02640001
                     REG_CTRL   = 8'h04,  // [0] start (wo), [1] irq_en
                     REG_STATUS = 8'h08,  // [0] busy [1] done [2] rd_err [3] wr_err
                     REG_ISTAT  = 8'h0C,  // rw1c [0] frame_done
                     REG_CFG    = 8'h10,  // [5:0] qp [6] intra_flag [7] sys_mode
                     REG_PIC    = 8'h14,  // [15:0] mb_x_total [31:16] mb_y_total
                     REG_YUV    = 8'h18,
                     REG_BANK0  = 8'h1C,
                     REG_BANK1  = 8'h20,
                     REG_BSADDR = 8'h24,
                     REG_BSLEN  = 8'h28,  // ro
                     REG_FRMCNT = 8'h2C;  // ro

    reg [31:0] ctrl_r, cfg_r, pic_r;
    reg [31:0] yuv_addr_r, bank0_addr_r, bank1_addr_r, bs_addr_r;
    reg [31:0] bs_len_r, frm_cnt_r, istat_r;
    reg        done_sticky_r, start_req_r;

    wire [16:0] mb_total_w = pic_r[15:0] * pic_r[31:16];
    wire [23:0] in_total_beats = mb_total_w * 24'd48;

    // frame FSM
    localparam [1:0] FR_IDLE = 2'd0, FR_RUN = 2'd1, FR_FLUSH = 2'd2, FR_DONE = 2'd3;
    reg [1:0]  fstate;
    reg        sys_start_r, sys_done_q;
    reg        cur_bank_r;
    reg [31:0] in_addr_r, bs_wr_base_r, bs_byt_cnt_r;
    reg [23:0] in_left_r;
    reg        in_active_r;
    reg        inf_clr_r, bsf_clr_r, wf_clr_r;

    wire frame_start_go  = (fstate == FR_IDLE) && start_req_r;
    wire frame_done_pulse = (fstate == FR_DONE);

    // encoder core wires
    wire       sys_done;
    wire       core_rinc;
    wire [7:0] core_bs_byte;
    wire       core_bs_winc;
    wire       ext_start;
    wire [2:0] ext_mode;
    wire [7:0] ext_mbx, ext_mby;
    wire [127:0] ext_rdata;
    wire       ext_done, ext_wen, ext_ren;
    wire [3:0] ext_addr;
    wire [127:0] ext_wdata;

    // reset (synchronized)
    reg rstn_q;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) rstn_q <= 1'b0;
        else          rstn_q <= 1'b1;
    end

    // --------------------------------------------------------------
    // AXI4-Lite slave
    // --------------------------------------------------------------
    reg        aw_hs, w_hs, bvalid_q;
    reg [7:0]  awaddr_q;
    reg [31:0] wdata_q;
    reg [3:0]  wstrb_q;

    assign s_axil_awready = ~aw_hs;
    assign s_axil_wready  = ~w_hs;
    assign s_axil_bresp   = 2'b00;
    assign s_axil_bvalid  = bvalid_q;

    wire [31:0] wmask = {{8{wstrb_q[3]}}, {8{wstrb_q[2]}},
                         {8{wstrb_q[1]}}, {8{wstrb_q[0]}}};
    wire axil_wr_fire = aw_hs && w_hs && !bvalid_q;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            aw_hs <= 1'b0;
            w_hs  <= 1'b0;
            bvalid_q <= 1'b0;
            awaddr_q <= 8'b0;
            wdata_q  <= 32'b0;
            wstrb_q  <= 4'b0;
        end else begin
            if (!aw_hs && s_axil_awvalid) begin
                aw_hs <= 1'b1;
                awaddr_q <= s_axil_awaddr;
            end
            if (!w_hs && s_axil_wvalid) begin
                w_hs <= 1'b1;
                wdata_q <= s_axil_wdata;
                wstrb_q <= s_axil_wstrb;
            end
            if (axil_wr_fire) begin
                case (awaddr_q)
                    REG_CTRL:   ctrl_r      <= (ctrl_r      & ~wmask) | (wdata_q & wmask);
                    REG_CFG:    cfg_r       <= (cfg_r       & ~wmask) | (wdata_q & wmask);
                    REG_PIC:    pic_r       <= (pic_r       & ~wmask) | (wdata_q & wmask);
                    REG_YUV:    yuv_addr_r  <= (yuv_addr_r  & ~wmask) | (wdata_q & wmask);
                    REG_BANK0:  bank0_addr_r<= (bank0_addr_r& ~wmask) | (wdata_q & wmask);
                    REG_BANK1:  bank1_addr_r<= (bank1_addr_r& ~wmask) | (wdata_q & wmask);
                    REG_BSADDR: bs_addr_r   <= (bs_addr_r   & ~wmask) | (wdata_q & wmask);
                    default: ;
                endcase
                aw_hs    <= 1'b0;
                w_hs     <= 1'b0;
                bvalid_q <= 1'b1;
            end else if (bvalid_q && s_axil_bready) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // frame start request (set by CTRL write, consumed by frame FSM)
    wire start_set_w = axil_wr_fire && (awaddr_q == REG_CTRL)
                       && wmask[0] && wdata_q[0] && (fstate == FR_IDLE);

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)           start_req_r <= 1'b0;
        else if (start_set_w)   start_req_r <= 1'b1;
        else if (frame_start_go) start_req_r <= 1'b0;
    end

    // interrupt status (rw1c + frame done set)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            istat_r <= 32'b0;
        end else begin
            if (axil_wr_fire && (awaddr_q == REG_ISTAT))
                istat_r <= istat_r & ~(wdata_q & wmask);
            if (frame_done_pulse)
                istat_r[0] <= 1'b1;
        end
    end

    reg        arvalid_q;
    reg [7:0]  araddr_q;
    reg [31:0] rdata_q;
    assign s_axil_arready = ~arvalid_q;
    assign s_axil_rvalid  = arvalid_q;
    assign s_axil_rdata   = rdata_q;
    assign s_axil_rresp   = 2'b00;

    wire [31:0] status_w = {28'b0, wr_err, rd_err, done_sticky_r, (fstate != FR_IDLE)};

    always @(*) begin
        case (araddr_q)
            REG_IDR:    rdata_q = 32'h0264_0001;
            REG_CTRL:   rdata_q = ctrl_r;
            REG_STATUS: rdata_q = status_w;
            REG_ISTAT:  rdata_q = istat_r;
            REG_CFG:    rdata_q = cfg_r;
            REG_PIC:    rdata_q = pic_r;
            REG_YUV:    rdata_q = yuv_addr_r;
            REG_BANK0:  rdata_q = bank0_addr_r;
            REG_BANK1:  rdata_q = bank1_addr_r;
            REG_BSADDR: rdata_q = bs_addr_r;
            REG_BSLEN:  rdata_q = bs_len_r;
            REG_FRMCNT: rdata_q = frm_cnt_r;
            default:    rdata_q = 32'b0;
        endcase
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            arvalid_q <= 1'b0;
            araddr_q  <= 8'b0;
        end else begin
            if (!arvalid_q && s_axil_arvalid) begin
                arvalid_q <= 1'b1;
                araddr_q  <= s_axil_araddr;
            end else if (arvalid_q && s_axil_rready)
                arvalid_q <= 1'b0;
        end
    end

    // --------------------------------------------------------------
    // encoder core
    // --------------------------------------------------------------
    top u_core (
        .clk            ( aclk       ),
        .rst_n          ( rstn_q     ),
        .sys_start      ( sys_start_r),
        .sys_done       ( sys_done   ),
        .sys_intra_flag ( cfg_r[6]   ),
        .sys_qp         ( cfg_r[5:0] ),
        .sys_mode       ( cfg_r[7]   ),
        .sys_x_total    ( pic_r[7:0]   - 8'd1 ),
        .sys_y_total    ( pic_r[23:16] - 8'd1 ),
        .enc_ld_start   (            ),
        .enc_ld_x       (            ),
        .enc_ld_y       (            ),
        .rdata_i        ( in_dout    ),
        .rvalid_i       ( in_rvalid  ),
        .rinc_o         ( core_rinc  ),
        .wdata_o        ( core_bs_byte ),
        .wfull_i        ( 1'b0       ),
        .winc_o         ( core_bs_winc ),
        .ext_mb_x_o     ( ext_mbx    ),
        .ext_mb_y_o     ( ext_mby    ),
        .ext_start_o    ( ext_start  ),
        .ext_done_i     ( ext_done   ),
        .ext_mode_o     ( ext_mode   ),
        .ext_wen_i      ( ext_wen    ),
        .ext_ren_i      ( ext_ren    ),
        .ext_addr_i     ( ext_addr   ),
        .ext_data_i     ( ext_wdata  ),
        .ext_data_o     ( ext_rdata  )
    );

    // --------------------------------------------------------------
    // input pixel stream: prefetch FIFO + linear DDR reader
    // --------------------------------------------------------------
    wire [63:0] in_dout;
    wire        in_empty, in_full;
    wire [8:0]  in_count;
    wire        in_rvalid = core_rinc && !in_empty;

    xk264_sync_fifo #(.DW(64), .AW(8)) u_in_fifo (
        .clk     ( aclk ),
        .rst_n   ( rstn_q ),
        .clr     ( inf_clr_r ),
        .wr_en   ( inrd_data_valid ),
        .wr_data ( inrd_data ),
        .full    ( in_full ),
        .rd_en   ( in_rvalid ),
        .rd_data ( in_dout ),
        .empty   ( in_empty ),
        .count   ( in_count )
    );

    // --------------------------------------------------------------
    // bitstream capture: byte FIFO -> word packer -> word FIFO -> AXI
    // --------------------------------------------------------------
    localparam [1:0] BW_RUN = 2'd0, BW_TAIL = 2'd1, BW_DRAIN = 2'd2, BW_DONE = 2'd3;
    reg [1:0]  bw_state;
    reg        bw_active_r, bs_wr_out_r, bs_flushed_r, tail_valid_r;
    reg [7:0]  tail_strb_r;
    reg [4:0]  bw_beats_q;
    reg [63:0] bs_word_r;
    reg [3:0]  bs_nb_r;
    reg        wf_tail_push_r;
    reg [63:0] wf_tail_data_r;

    wire       bsf_empty, bsf_full;
    wire [7:0] bsf_dout;
    wire [9:0] bsf_count;
    wire       wf_full, wf_empty;
    wire [63:0] wf_dout;
    wire [5:0]  wf_count;

    wire bw_flush_req = (fstate == FR_FLUSH);
    wire can_take = ~bsf_empty && ((bs_nb_r != 4'd7) || ~wf_full);
    wire bw_pull  = (bw_state == BW_RUN) && can_take;

    // keeps the TOP (8-nb) bytes of the word: those are stale bytes shifted
    // up from the previous full word; the valid partial-word bytes accumulate
    // in the LOW nb lanes (byte packer shifts new bytes in at [7:0]).
    // Shift by nb*8 (NOT 64-nb*8): for nb >= 5 that wrong form masked out
    // valid lanes, zeroing the bytes at stream offsets len-nb+2 .. len-3
    // (frames with bslen%8 = 5..7 had a corrupted last macroblock).
    wire [63:0] tail_mask = 64'hFFFFFFFFFFFFFFFF << ({3'b0, bs_nb_r} * 7'd8);

    // command is latched when first eligible and held stable until the
    // arbiter accepts it (arbiter contract); wf_count can change while
    // the AW handshake is pending
    reg        bw_cmd_pend_r;
    reg [4:0]  bw_cmd_beats_r;
    reg        bw_cmd_last_r;
    wire bw_issue_now = ((bw_state == BW_RUN)   && !bw_active_r && !bw_cmd_pend_r && (wf_count >= 6'd16)) ||
                        ((bw_state == BW_DRAIN) && !bw_active_r && !bw_cmd_pend_r && (wf_count  != 6'd0));
    wire bw_issue = bw_cmd_pend_r;
    wire [4:0] bw_issue_beats = bw_cmd_beats_r;
    wire bw_issue_last = bw_cmd_last_r;

    wire bw_c1_wvalid = bw_active_r && !wf_empty;

    xk264_sync_fifo #(.DW(8), .AW(9)) u_bs_byte_fifo (
        .clk     ( aclk ),
        .rst_n   ( rstn_q ),
        .clr     ( bsf_clr_r ),
        .wr_en   ( core_bs_winc ),
        .wr_data ( core_bs_byte ),
        .full    ( bsf_full ),
        .rd_en   ( bw_pull ),
        .rd_data ( bsf_dout ),
        .empty   ( bsf_empty ),
        .count   ( bsf_count )
    );

    xk264_sync_fifo #(.DW(64), .AW(5)) u_bs_word_fifo (
        .clk     ( aclk ),
        .rst_n   ( rstn_q ),
        .clr     ( wf_clr_r ),
        .wr_en   ( bw_word_wr ),
        .wr_data ( bw_word_data ),
        .full    ( wf_full ),
        .rd_en   ( bsw_wfire ),
        .rd_data ( wf_dout ),
        .empty   ( wf_empty ),
        .count   ( wf_count )
    );

    wire        bw_word_wr   = bw_pull ? (bs_nb_r == 4'd7) : wf_tail_push_r;
    wire [63:0] bw_word_data = bw_pull ? {bs_word_r[55:0], bsf_dout} : wf_tail_data_r;

    // --------------------------------------------------------------
    // ext memory controller
    // --------------------------------------------------------------
    wire        extrd_cmd_valid, extrd_cmd_accept, extrd_data_valid, extrd_data_last;
    wire [31:0] extrd_cmd_addr;
    wire [4:0]  extrd_cmd_beats;
    wire [63:0] extrd_data;
    wire        extrd_err_unused;

    wire        extwr_cmd_valid, extwr_cmd_accept, extwr_done, extwr_wfire, extwr_wvalid;
    wire [31:0] extwr_cmd_addr;
    wire [4:0]  extwr_cmd_beats;
    wire [7:0]  extwr_cmd_wstrb;
    wire [63:0] extwr_wdata;

    xk264_ext_ctrl u_ext_ctrl (
        .clk           ( aclk ),
        .rst_n         ( rstn_q ),
        .ext_start_i   ( ext_start ),
        .ext_mode_i    ( ext_mode ),
        .ext_mb_x_i    ( ext_mbx ),
        .ext_mb_y_i    ( ext_mby ),
        .ext_done_o    ( ext_done ),
        .ext_wen_o     ( ext_wen ),
        .ext_wdata_o   ( ext_wdata ),
        .ext_ren_o     ( ext_ren ),
        .ext_addr_o    ( ext_addr ),
        .ext_rdata_i   ( ext_rdata ),
        .bank0_addr_i  ( bank0_addr_r ),
        .bank1_addr_i  ( bank1_addr_r ),
        .mb_x_total_i  ( pic_r[8:0] ),
        .cur_bank_i    ( cur_bank_r ),
        .rd_cmd_valid  ( extrd_cmd_valid ),
        .rd_cmd_addr   ( extrd_cmd_addr ),
        .rd_cmd_beats  ( extrd_cmd_beats ),
        .rd_cmd_accept ( extrd_cmd_accept ),
        .rd_data_valid ( extrd_data_valid ),
        .rd_data       ( extrd_data ),
        .rd_data_last  ( extrd_data_last ),
        .wr_cmd_valid  ( extwr_cmd_valid ),
        .wr_cmd_addr   ( extwr_cmd_addr ),
        .wr_cmd_beats  ( extwr_cmd_beats ),
        .wr_cmd_wstrb  ( extwr_cmd_wstrb ),
        .wr_cmd_accept ( extwr_cmd_accept ),
        .wr_done       ( extwr_done ),
        .wr_wfire      ( extwr_wfire ),
        .wr_wvalid     ( extwr_wvalid ),
        .wr_wdata      ( extwr_wdata )
    );

    // --------------------------------------------------------------
    // AXI read arbitration: ext (priority) / input prefetch
    // --------------------------------------------------------------
    wire        inrd_cmd_valid;
    wire [31:0] inrd_cmd_addr = in_addr_r;
    wire [4:0]  inrd_cmd_beats = (in_left_r >= 24'd16) ? 5'd16 : in_left_r[4:0];
    assign inrd_cmd_valid = (fstate == FR_RUN) && !in_active_r
                            && (in_left_r != 24'd0) && (in_count <= 9'd240);
    wire        inrd_cmd_accept, inrd_data_valid, inrd_data_last;
    wire [63:0] inrd_data;
    wire        rd_err, wr_err;

    xk264_axi_rd_arb u_rd_arb (
        .clk            ( aclk ),
        .rst_n          ( rstn_q ),
        .c0_cmd_valid   ( extrd_cmd_valid ),
        .c0_cmd_addr    ( extrd_cmd_addr ),
        .c0_cmd_beats   ( extrd_cmd_beats ),
        .c0_cmd_accept  ( extrd_cmd_accept ),
        .c0_data_valid  ( extrd_data_valid ),
        .c0_data_last   ( extrd_data_last ),
        .c0_data        ( extrd_data ),
        .c1_cmd_valid   ( inrd_cmd_valid ),
        .c1_cmd_addr    ( inrd_cmd_addr ),
        .c1_cmd_beats   ( inrd_cmd_beats ),
        .c1_cmd_accept  ( inrd_cmd_accept ),
        .c1_data_valid  ( inrd_data_valid ),
        .c1_data_last   ( inrd_data_last ),
        .c1_data        ( inrd_data ),
        .rd_err         ( rd_err ),
        .m_axi_arvalid  ( m_axi_arvalid ),
        .m_axi_arready  ( m_axi_arready ),
        .m_axi_araddr   ( m_axi_araddr ),
        .m_axi_arlen    ( m_axi_arlen ),
        .m_axi_arsize   ( m_axi_arsize ),
        .m_axi_arburst  ( m_axi_arburst ),
        .m_axi_arlock   ( m_axi_arlock ),
        .m_axi_arcache  ( m_axi_arcache ),
        .m_axi_arprot   ( m_axi_arprot ),
        .m_axi_arqos    ( m_axi_arqos ),
        .m_axi_rvalid   ( m_axi_rvalid ),
        .m_axi_rready   ( m_axi_rready ),
        .m_axi_rdata    ( m_axi_rdata ),
        .m_axi_rlast    ( m_axi_rlast ),
        .m_axi_rresp    ( m_axi_rresp )
    );

    // --------------------------------------------------------------
    // AXI write arbitration: ext (priority) / bitstream
    // --------------------------------------------------------------
    wire        bsw_cmd_valid = bw_issue;
    wire [31:0] bsw_cmd_addr  = bs_wr_base_r;
    wire [4:0]  bsw_cmd_beats = bw_issue_beats;
    wire [7:0]  bsw_cmd_wstrb = (bw_issue_last && tail_valid_r) ? tail_strb_r : 8'hFF;
    wire        bsw_cmd_accept, bsw_done, bsw_wfire;
    wire [63:0] bsw_wdata = wf_dout;

    xk264_axi_wr_arb u_wr_arb (
        .clk            ( aclk ),
        .rst_n          ( rstn_q ),
        .c0_cmd_valid   ( extwr_cmd_valid ),
        .c0_cmd_addr    ( extwr_cmd_addr ),
        .c0_cmd_beats   ( extwr_cmd_beats ),
        .c0_cmd_wstrb   ( extwr_cmd_wstrb ),
        .c0_cmd_accept  ( extwr_cmd_accept ),
        .c0_done        ( extwr_done ),
        .c0_wfire       ( extwr_wfire ),
        .c0_wvalid      ( extwr_wvalid ),
        .c0_wdata       ( extwr_wdata ),
        .c1_cmd_valid   ( bsw_cmd_valid ),
        .c1_cmd_addr    ( bsw_cmd_addr ),
        .c1_cmd_beats   ( bsw_cmd_beats ),
        .c1_cmd_wstrb   ( bsw_cmd_wstrb ),
        .c1_cmd_accept  ( bsw_cmd_accept ),
        .c1_done        ( bsw_done ),
        .c1_wfire       ( bsw_wfire ),
        .c1_wvalid      ( bw_c1_wvalid ),
        .c1_wdata       ( bsw_wdata ),
        .wr_err         ( wr_err ),
        .m_axi_awvalid  ( m_axi_awvalid ),
        .m_axi_awready  ( m_axi_awready ),
        .m_axi_awaddr   ( m_axi_awaddr ),
        .m_axi_awlen    ( m_axi_awlen ),
        .m_axi_awsize   ( m_axi_awsize ),
        .m_axi_awburst  ( m_axi_awburst ),
        .m_axi_awlock   ( m_axi_awlock ),
        .m_axi_awcache  ( m_axi_awcache ),
        .m_axi_awprot   ( m_axi_awprot ),
        .m_axi_awqos    ( m_axi_awqos ),
        .m_axi_wvalid   ( m_axi_wvalid ),
        .m_axi_wdata    ( m_axi_wdata ),
        .m_axi_wstrb    ( m_axi_wstrb ),
        .m_axi_wlast    ( m_axi_wlast ),
        .m_axi_wready   ( m_axi_wready ),
        .m_axi_bvalid   ( m_axi_bvalid ),
        .m_axi_bready   ( m_axi_bready ),
        .m_axi_bresp    ( m_axi_bresp )
    );

    // AXI ID
    assign m_axi_arid = 1'b0;
    assign m_axi_awid = 1'b0;

    // --------------------------------------------------------------
    // input reader bookkeeping
    // --------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            in_active_r <= 1'b0;
            in_addr_r   <= 32'b0;
            in_left_r   <= 24'b0;
            inf_clr_r   <= 1'b0;
        end else if (frame_start_go) begin
            in_active_r <= 1'b0;
            in_addr_r   <= yuv_addr_r;
            in_left_r   <= in_total_beats;
            inf_clr_r   <= 1'b1;
        end else begin
            inf_clr_r <= 1'b0;
            if (inrd_cmd_accept) begin
                in_active_r <= 1'b1;
                in_addr_r   <= in_addr_r + {inrd_cmd_beats, 3'b0};
                in_left_r   <= in_left_r - {19'b0, inrd_cmd_beats};
            end else if (inrd_data_last) begin
                in_active_r <= 1'b0;
            end
        end
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            bw_cmd_pend_r    <= 1'b0;
            bw_cmd_beats_r   <= 5'b0;
            bw_cmd_last_r    <= 1'b0;
        end else if (frame_start_go) begin
            bw_cmd_pend_r    <= 1'b0;
            bw_cmd_beats_r   <= 5'b0;
            bw_cmd_last_r    <= 1'b0;
        end else begin
            if (bsw_cmd_accept) begin
                bw_cmd_pend_r <= 1'b0;
            end else if (bw_issue_now) begin
                bw_cmd_pend_r  <= 1'b1;
                bw_cmd_beats_r <= (bw_state == BW_DRAIN)
                                  ? ((wf_count > 6'd16) ? 5'd16 : wf_count[4:0])
                                  : 5'd16;
                bw_cmd_last_r  <= (bw_state == BW_DRAIN) && (wf_count <= 6'd16);
            end
        end
    end

    // --------------------------------------------------------------
    // bitstream writer FSM
    // --------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            bw_state         <= BW_RUN;
            bw_active_r      <= 1'b0;
            bs_wr_out_r      <= 1'b0;
            bs_flushed_r     <= 1'b0;
            tail_valid_r     <= 1'b0;
            tail_strb_r      <= 8'hFF;
            bw_beats_q       <= 5'b0;
            bs_word_r        <= 64'b0;
            bs_nb_r          <= 4'b0;
            bs_wr_base_r     <= 32'b0;
            bsf_clr_r        <= 1'b0;
            wf_clr_r         <= 1'b0;
            bs_byt_cnt_r     <= 32'b0;
            wf_tail_push_r   <= 1'b0;
            wf_tail_data_r   <= 64'b0;
        end else if (frame_start_go) begin
            bw_state         <= BW_RUN;
            bw_active_r      <= 1'b0;
            bs_wr_out_r      <= 1'b0;
            bs_flushed_r     <= 1'b0;
            tail_valid_r     <= 1'b0;
            tail_strb_r      <= 8'hFF;
            bw_beats_q       <= 5'b0;
            bs_word_r        <= 64'b0;
            bs_nb_r          <= 4'b0;
            bs_wr_base_r     <= bs_addr_r;
            bsf_clr_r        <= 1'b1;
            wf_clr_r         <= 1'b1;
            bs_byt_cnt_r     <= 32'b0;
            wf_tail_push_r   <= 1'b0;
        end else begin
            bsf_clr_r <= 1'b0;
            wf_clr_r  <= 1'b0;
            wf_tail_push_r <= 1'b0;
            if (core_bs_winc)
                bs_byt_cnt_r <= bs_byt_cnt_r + 32'd1;

            // byte packer
            if (bw_pull) begin
                if (bs_nb_r == 4'd7)
                    bs_nb_r <= 4'd0;
                else
                    bs_nb_r <= bs_nb_r + 4'd1;
                bs_word_r <= {bs_word_r[55:0], bsf_dout};
            end

            // burst bookkeeping
            if (bsw_cmd_accept) begin
                bw_active_r <= 1'b1;
                bw_beats_q  <= bsw_cmd_beats;
                bs_wr_base_r <= bs_wr_base_r + {bsw_cmd_beats, 3'b0};
                bs_wr_out_r  <= 1'b1;
            end
            if (bsw_wfire) begin
                bw_beats_q <= bw_beats_q - 5'd1;
                if (bw_beats_q == 5'd1)
                    bw_active_r <= 1'b0;
            end
            if (bsw_done)
                bs_wr_out_r <= 1'b0;

            case (bw_state)
                BW_RUN: begin
                    if (bw_flush_req && bsf_empty && !bw_active_r && !bs_wr_out_r)
                        bw_state <= BW_TAIL;
                end
                BW_TAIL: begin
                    if (bs_nb_r == 4'd0) begin
                        bw_state <= BW_DRAIN;
                    end else if (!wf_full) begin
                        // push padded partial word (valid bytes in LSBs);
                        // stay in BW_TAIL this cycle so the push lands in
                        // the FIFO before BW_DRAIN samples wf_count
                        tail_valid_r   <= 1'b1;
                        tail_strb_r    <= 8'hFF >> (4'd8 - bs_nb_r);
                        wf_tail_data_r <= bs_word_r & ~tail_mask;
                        wf_tail_push_r <= 1'b1;
                        bs_nb_r        <= 4'd0;
                    end
                end
                BW_DRAIN: begin
                    if ((wf_count == 6'd0) && !bw_active_r && !bs_wr_out_r) begin
                        bs_flushed_r <= 1'b1;
                        bw_state     <= BW_DONE;
                    end
                end
                BW_DONE: begin
                    // sticky until next frame start
                end
                default: bw_state <= BW_RUN;
            endcase
        end
    end

    // --------------------------------------------------------------
    // frame control FSM
    // --------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            fstate        <= FR_IDLE;
            sys_start_r   <= 1'b0;
            sys_done_q    <= 1'b0;
            cur_bank_r    <= 1'b0;
            done_sticky_r <= 1'b0;
            bs_len_r      <= 32'b0;
            frm_cnt_r     <= 32'b0;
        end else begin
            sys_done_q <= sys_done;
            case (fstate)
                FR_IDLE: begin
                    if (start_req_r) begin
                        done_sticky_r <= 1'b0;
                        sys_start_r   <= 1'b1;
                        fstate        <= FR_RUN;
                    end
                end
                FR_RUN: begin
                    sys_start_r <= 1'b0;
                    if (sys_done && !sys_done_q)
                        fstate <= FR_FLUSH;
                end
                FR_FLUSH: begin
                    if (bs_flushed_r)
                        fstate <= FR_DONE;
                end
                FR_DONE: begin
                    bs_len_r      <= bs_byt_cnt_r;
                    frm_cnt_r     <= frm_cnt_r + 32'd1;
                    cur_bank_r    <= ~cur_bank_r;
                    done_sticky_r <= 1'b1;
                    fstate        <= FR_IDLE;
                end
                default: fstate <= FR_IDLE;
            endcase
        end
    end

    assign irq = istat_r[0] & ctrl_r[1];

endmodule
