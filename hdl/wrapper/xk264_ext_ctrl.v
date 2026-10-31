//-------------------------------------------------------------------
// xk264_ext_ctrl - External reference/reconstruction frame memory
// controller for the xk264 H.264 encoder core.
//
// Bridges the encoder core's per-macroblock ext_mem interface
// (ext_start/ext_mode/ext_done, ext_wen/ext_data, ext_ren/ext_addr)
// to DDR through the shared AXI4 master.
//
// DDR layout (per reconstruction/reference bank):
//   bank_base + mb_index*512 + (chroma ? 256 : 0) + word*8
//   luma   region: 32 words (256 B), word 2r = row r px[7:0],
//                                   word 2r+1 = row r px[15:8]
//   chroma region: 16 words (128 B), same row/word mapping
//
// ext_mode:
//   [2] 0:load 1:store  [1] 0:luma 1:chroma  [0] 0:bottom 4 rows 1:full MB
// Full-MB loads read the reference bank (~cur_bank_i), deblock
// loads/stores and all stores use the current bank (cur_bank_i),
// matching the golden testbench (tb_top.v) banking.
//
// Load replay / store capture beat orders replicate tb_top.v exactly.
//-------------------------------------------------------------------
module xk264_ext_ctrl (
    input  wire         clk,
    input  wire         rst_n,

    // encoder core ext-mem interface
    input  wire         ext_start_i,
    input  wire [2:0]   ext_mode_i,
    input  wire [7:0]   ext_mb_x_i,
    input  wire [7:0]   ext_mb_y_i,
    output reg          ext_done_o,
    output reg          ext_wen_o,
    output reg  [127:0] ext_wdata_o,
    output reg          ext_ren_o,
    output reg  [3:0]   ext_addr_o,
    input  wire [127:0] ext_rdata_i,

    // configuration
    input  wire [31:0]  bank0_addr_i,
    input  wire [31:0]  bank1_addr_i,
    input  wire [8:0]   mb_x_total_i,
    input  wire         cur_bank_i,

    // DDR read client
    output reg          rd_cmd_valid,
    output reg  [31:0]  rd_cmd_addr,
    output reg  [4:0]   rd_cmd_beats,
    input  wire         rd_cmd_accept,
    input  wire         rd_data_valid,
    input  wire [63:0]  rd_data,
    input  wire         rd_data_last,

    // DDR write client
    output reg          wr_cmd_valid,
    output reg  [31:0]  wr_cmd_addr,
    output reg  [4:0]   wr_cmd_beats,
    output reg  [7:0]   wr_cmd_wstrb,
    input  wire         wr_cmd_accept,
    input  wire         wr_done,
    input  wire         wr_wfire,
    output wire         wr_wvalid,
    output wire [63:0]  wr_wdata
);

    localparam E_IDLE    = 4'd0,
               E_AR      = 4'd1,
               E_FILL    = 4'd2,
               E_LPLAY   = 4'd3,
               E_LDONE   = 4'd4,
               E_CAP     = 4'd5,
               E_CAPEND  = 4'd6,
               E_SDONE   = 4'd7,
               E_AW      = 4'd8,
               E_WDATA   = 4'd9,
               E_WFIN    = 4'd10;

    reg [3:0]  state;

    reg        load_q, uv_q, full_q;
    reg [31:0] region_base_q;

    // 32 x 64b macroblock staging buffer
    // 同一拍需要从多个不同地址读出（重放时拼装 64b/128b 数据），
    // 无法映射到 Block-RAM，强制用寄存器实现，避免 BRAM 模板推断报错。
    (* ram_style = "registers" *)
    reg [63:0] mbuf [0:31];

    // load/read path
    reg [5:0]  wcur;          // current absolute word index
    reg [6:0]  wrem;          // words remaining
    reg [4:0]  burst_beats_q; // beats of current read burst
    reg [4:0]  fill_cnt;      // beats received in current burst
    reg [5:0]  play_cnt;      // replay beat counter
    reg [5:0]  nbeats_q;      // replay beat count
    reg [5:0]  wstart_q;      // first word index of region access

    // store capture path
    reg [4:0]  cap_idx;       // next store address index
    reg [4:0]  cap_cnt_q;     // total store addresses
    reg [3:0]  cap_addr_q;    // address presented in previous cycle
    reg        cap_vld_q;     // capture data valid
    reg [3:0]  addr_start_q;  // first store address

    // write-out path
    reg [4:0]  wbeat_cnt;

    // replay beat generation (combinational)
    wire [5:0] play_i = play_cnt[5:2];
    wire [1:0] play_j = play_cnt[1:0];
    wire [5:0] w_a    = {play_cnt[5:2], 2'b00};
    reg [63:0] play_d64;
    always @(*) begin
        case (play_j)
            2'd0: play_d64 = {mbuf[w_a + 6'd2][31:0],  mbuf[w_a][31:0]};
            2'd1: play_d64 = {mbuf[w_a + 6'd2][63:32], mbuf[w_a][63:32]};
            2'd2: play_d64 = {mbuf[w_a + 6'd3][31:0],  mbuf[w_a + 6'd1][31:0]};
            default: play_d64 = {mbuf[w_a + 6'd3][63:32], mbuf[w_a + 6'd1][63:32]};
        endcase
    end
    wire [5:0] db_w0 = wstart_q + {play_cnt[1:0], 1'b0};
    wire [127:0] play_d128 = {mbuf[db_w0 + 6'd1], mbuf[db_w0]};

    // staging buffer writes: store capture (two words per beat) and DDR fill.
    // 内容不需要复位，因此写端口用纯同步逻辑（不能挂在异步复位上，
    // 否则 Vivado 无法按 BRAM 模板推断并报 8-2913）。
    wire fill_wen = rd_data_valid && (state == E_FILL);
    always @(posedge clk) begin
        if (cap_vld_q) begin
            mbuf[{cap_addr_q, 1'b0}] <= ext_rdata_i[63:0];
            mbuf[{cap_addr_q, 1'b1}] <= ext_rdata_i[127:64];
        end
        if (fill_wen) begin
            mbuf[wcur] <= rd_data;
        end
    end

    // DDR address helpers
    wire [16:0] mb_idx  = ext_mb_y_i * mb_x_total_i + ext_mb_x_i;
    wire [31:0] bank0_mb = bank0_addr_i + {mb_idx, 9'b0};
    wire [31:0] bank1_mb = bank1_addr_i + {mb_idx, 9'b0};
    // must use ext_mode_i[0] (current txn), not full_q which is
    // latched this same cycle and still holds the previous txn's value
    wire        ld_bank  = ext_mode_i[0] ? ~cur_bank_i : cur_bank_i;
    wire [31:0] ld_base  = ld_bank ? (bank1_mb + (ext_mode_i[1] ? 32'd256 : 32'd0))
                                   : (bank0_mb + (ext_mode_i[1] ? 32'd256 : 32'd0));
    wire [31:0] st_base  = cur_bank_i ? (bank1_mb + (ext_mode_i[1] ? 32'd256 : 32'd0))
                                      : (bank0_mb + (ext_mode_i[1] ? 32'd256 : 32'd0));

    assign wr_wvalid = (state == E_WDATA);
    assign wr_wdata  = mbuf[wcur];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= E_IDLE;
            load_q        <= 1'b0;
            uv_q          <= 1'b0;
            full_q        <= 1'b0;
            region_base_q <= 32'b0;
            ext_done_o    <= 1'b0;
            ext_wen_o     <= 1'b0;
            ext_wdata_o   <= 128'b0;
            ext_ren_o     <= 1'b0;
            ext_addr_o    <= 4'b0;
            rd_cmd_valid  <= 1'b0;
            rd_cmd_addr   <= 32'b0;
            rd_cmd_beats  <= 5'b0;
            wr_cmd_valid  <= 1'b0;
            wr_cmd_addr   <= 32'b0;
            wr_cmd_beats  <= 5'b0;
            wr_cmd_wstrb  <= 8'hFF;
            wcur          <= 6'b0;
            wrem          <= 7'b0;
            wstart_q      <= 6'b0;
            burst_beats_q <= 5'b0;
            fill_cnt      <= 5'b0;
            play_cnt      <= 6'b0;
            nbeats_q      <= 6'b0;
            cap_idx       <= 4'b0;
            cap_cnt_q     <= 4'b0;
            cap_addr_q    <= 4'b0;
            cap_vld_q     <= 1'b0;
            addr_start_q  <= 4'b0;
            wbeat_cnt     <= 5'b0;
        end else begin
            // default pulses
            ext_done_o <= 1'b0;

            case (state)
                E_IDLE: begin
                    if (ext_start_i) begin
                        load_q <= ~ext_mode_i[2];
                        uv_q   <= ext_mode_i[1];
                        full_q <= ext_mode_i[0];
                        if (ext_mode_i[2]) begin
                            region_base_q <= st_base;
                            case (ext_mode_i)
                                3'b101: begin // store full luma MB
                                    addr_start_q <= 4'd0;
                                    cap_cnt_q    <= 5'd16;
                                    wcur         <= 6'd0;
                                    wrem         <= 7'd32;
                                end
                                3'b111: begin // store full chroma MB
                                    addr_start_q <= 4'd0;
                                    cap_cnt_q    <= 4'd8;
                                    wcur         <= 6'd0;
                                    wrem         <= 7'd16;
                                end
                                3'b100: begin // store db luma bottom rows
                                    addr_start_q <= 4'd12;
                                    cap_cnt_q    <= 4'd4;
                                    wcur         <= 6'd24;
                                    wrem         <= 7'd8;
                                end
                                default: begin // store db chroma bottom rows
                                    addr_start_q <= 4'd4;
                                    cap_cnt_q    <= 4'd4;
                                    wcur         <= 6'd8;
                                    wrem         <= 7'd8;
                                end
                            endcase
                            // first address presented when entering E_CAP
                            cap_idx    <= 4'd1;
                            cap_vld_q  <= 1'b0;
                            ext_ren_o  <= 1'b1;
                            ext_addr_o <= (ext_mode_i == 3'b101 || ext_mode_i == 3'b111) ? 4'd0 :
                                          (ext_mode_i == 3'b100) ? 4'd12 : 4'd4;
                            state <= E_CAP;
                        end else begin
                            region_base_q <= ld_base;
                            if (ext_mode_i[0]) begin // full MB load
                                wstart_q <= 6'd0;
                                wcur     <= 6'd0;
                                wrem     <= ext_mode_i[1] ? 7'd16 : 7'd32;
                                nbeats_q <= ext_mode_i[1] ? 6'd16 : 6'd32;
                            end else begin // db bottom rows load
                                wstart_q <= ext_mode_i[1] ? 6'd8 : 6'd24;
                                wcur     <= ext_mode_i[1] ? 6'd8 : 6'd24;
                                wrem     <= 7'd8;
                                nbeats_q <= 6'd4;
                            end
                            state <= E_AR;
                        end
                    end
                end

                // ---------------- load: read MB region from DDR ----------------
                E_AR: begin
                    rd_cmd_valid <= 1'b1;
                    rd_cmd_addr  <= region_base_q + {wcur, 3'b0};
                    rd_cmd_beats <= (wrem > 7'd16) ? 5'd16 : wrem[4:0];
                    if (rd_cmd_accept) begin
                        rd_cmd_valid  <= 1'b0;
                        burst_beats_q <= (wrem > 7'd16) ? 5'd16 : wrem[4:0];
                        fill_cnt      <= 5'd0;
                        state         <= E_FILL;
                    end
                end

                E_FILL: begin
                    if (rd_data_valid) begin
                        wcur      <= wcur + 6'd1;
                        wrem      <= wrem - 7'd1;
                        fill_cnt  <= fill_cnt + 5'd1;
                        if (fill_cnt == burst_beats_q - 5'd1) begin
                            if (wrem == 7'd1)
                                state <= E_LPLAY;
                            else
                                state <= E_AR;
                        end
                    end
                end

                // ---------------- load: replay beats to core ----------------
                E_LPLAY: begin
                    if (play_cnt < nbeats_q) begin
                        ext_wen_o   <= 1'b1;
                        ext_wdata_o <= full_q ? {64'b0, play_d64} : play_d128;
                        play_cnt    <= play_cnt + 6'd1;
                    end else begin
                        ext_wen_o <= 1'b0;
                        state     <= E_LDONE;
                    end
                end

                E_LDONE: begin
                    ext_done_o <= 1'b1;
                    play_cnt   <= 6'd0;
                    state      <= E_IDLE;
                end

                // ---------------- store: capture beats from core ----------------
                E_CAP: begin
                    if (cap_idx < cap_cnt_q) begin
                        ext_addr_o <= addr_start_q + cap_idx;
                        cap_idx    <= cap_idx + 4'd1;
                    end else begin
                        ext_ren_o <= 1'b0;
                        state     <= E_CAPEND;
                    end
                    cap_addr_q <= ext_addr_o;
                    cap_vld_q  <= 1'b1;
                end

                E_CAPEND: begin
                    cap_vld_q <= 1'b0;
                    state     <= E_AW;
                end

                // ---------------- store: write buffer to DDR ----------------
                E_AW: begin
                    wr_cmd_valid  <= 1'b1;
                    wr_cmd_addr   <= region_base_q + {wcur, 3'b0};
                    wr_cmd_beats  <= (wrem > 7'd16) ? 5'd16 : wrem[4:0];
                    wr_cmd_wstrb  <= 8'hFF;
                    if (wr_cmd_accept) begin
                        wr_cmd_valid  <= 1'b0;
                        burst_beats_q <= (wrem > 7'd16) ? 5'd16 : wrem[4:0];
                        wbeat_cnt     <= 5'd0;
                        state         <= E_WDATA;
                    end
                end

                E_WDATA: begin
                    if (wr_wfire) begin
                        wcur      <= wcur + 6'd1;
                        wrem      <= wrem - 7'd1;
                        wbeat_cnt <= wbeat_cnt + 5'd1;
                        if (wbeat_cnt == burst_beats_q - 5'd1) begin
                            if (wrem == 7'd1)
                                state <= E_WFIN;
                            else
                                state <= E_AW;
                        end
                    end
                end

                E_WFIN: begin
                    if (wr_done) begin
                        // done only after the DDR write completes, so the
                        // core's next ext_start pulse is never missed
                        ext_done_o <= 1'b1;
                        state      <= E_IDLE;
                    end
                end

                default: state <= E_IDLE;
            endcase
        end
    end

    // unused, kept for interface clarity
    wire unused_last = rd_data_last;

endmodule
