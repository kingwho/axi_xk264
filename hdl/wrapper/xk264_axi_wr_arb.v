//-------------------------------------------------------------------
// Two-client to one AXI4 write-channel arbiter.
// Client 0 has priority. One outstanding burst at a time.
// Clients must hold cmd signals stable until cmd_accept pulses and
// must keep wvalid/wdata valid during the whole W phase.
// cmd_wstrb applies to the last beat of the burst (other beats are
// full-width). cX_done pulses when the B response is received.
// cX_wfire pulses for every beat accepted on the AXI W channel.
//-------------------------------------------------------------------
module xk264_axi_wr_arb (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        c0_cmd_valid,
    input  wire [31:0] c0_cmd_addr,
    input  wire [4:0]  c0_cmd_beats,
    input  wire [7:0]  c0_cmd_wstrb,
    output reg         c0_cmd_accept,
    output reg         c0_done,
    output wire        c0_wfire,
    input  wire        c0_wvalid,
    input  wire [63:0] c0_wdata,

    input  wire        c1_cmd_valid,
    input  wire [31:0] c1_cmd_addr,
    input  wire [4:0]  c1_cmd_beats,
    input  wire [7:0]  c1_cmd_wstrb,
    output reg         c1_cmd_accept,
    output reg         c1_done,
    output wire        c1_wfire,
    input  wire        c1_wvalid,
    input  wire [63:0] c1_wdata,

    output reg         wr_err,

    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    output reg  [31:0] m_axi_awaddr,
    output reg  [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [0:0]  m_axi_awlock,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire [3:0]  m_axi_awqos,

    output wire        m_axi_wvalid,
    output wire [63:0] m_axi_wdata,
    output wire [7:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    input  wire        m_axi_wready,

    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    input  wire [1:0]  m_axi_bresp
);

    localparam S_IDLE = 2'd0, S_AW = 2'd1, S_W = 2'd2, S_B = 2'd3;

    reg [1:0] state;
    reg       owner;
    reg [7:0] beats_q;   // beats - 1
    reg [7:0] beat_cnt;
    reg [7:0] wstrb_q;

    assign m_axi_awsize  = 3'd3;
    assign m_axi_awburst = 2'd1;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'b0000;

    wire        c_wvalid = owner ? c1_wvalid : c0_wvalid;
    wire [63:0] c_wdata  = owner ? c1_wdata  : c0_wdata;

    wire wfire = m_axi_wvalid && m_axi_wready;

    assign m_axi_wvalid = (state == S_W) && c_wvalid;
    assign m_axi_wdata  = c_wdata;
    assign m_axi_wlast  = (state == S_W) && (beat_cnt == beats_q);
    assign m_axi_wstrb  = m_axi_wlast ? wstrb_q : 8'hFF;

    assign c0_wfire = wfire && (owner == 1'b0);
    assign c1_wfire = wfire && (owner == 1'b1);

    assign m_axi_bready = (state == S_B);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            owner         <= 1'b0;
            beats_q       <= 8'b0;
            beat_cnt      <= 8'b0;
            wstrb_q       <= 8'hFF;
            m_axi_awvalid <= 1'b0;
            m_axi_awaddr  <= 32'b0;
            m_axi_awlen   <= 8'b0;
            c0_cmd_accept <= 1'b0;
            c1_cmd_accept <= 1'b0;
            c0_done       <= 1'b0;
            c1_done       <= 1'b0;
            wr_err        <= 1'b0;
        end else begin
            c0_cmd_accept <= 1'b0;
            c1_cmd_accept <= 1'b0;
            c0_done       <= 1'b0;
            c1_done       <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (c0_cmd_valid) begin
                        m_axi_awaddr  <= c0_cmd_addr;
                        m_axi_awlen   <= {3'b0, c0_cmd_beats} - 8'd1;
                        wstrb_q       <= c0_cmd_wstrb;
                        beats_q       <= {3'b0, c0_cmd_beats} - 8'd1;
                        owner         <= 1'b0;
                        m_axi_awvalid <= 1'b1;
                        state         <= S_AW;
                    end else if (c1_cmd_valid) begin
                        m_axi_awaddr  <= c1_cmd_addr;
                        m_axi_awlen   <= {3'b0, c1_cmd_beats} - 8'd1;
                        wstrb_q       <= c1_cmd_wstrb;
                        beats_q       <= {3'b0, c1_cmd_beats} - 8'd1;
                        owner         <= 1'b1;
                        m_axi_awvalid <= 1'b1;
                        state         <= S_AW;
                    end
                end
                S_AW: begin
                    if (m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        beat_cnt      <= 8'b0;
                        if (owner == 1'b0)
                            c0_cmd_accept <= 1'b1;
                        else
                            c1_cmd_accept <= 1'b1;
                        state <= S_W;
                    end
                end
                S_W: begin
                    if (wfire) begin
                        beat_cnt <= beat_cnt + 8'd1;
                        if (beat_cnt == beats_q)
                            state <= S_B;
                    end
                end
                S_B: begin
                    if (m_axi_bvalid) begin
                        if (m_axi_bresp != 2'b00)
                            wr_err <= 1'b1;
                        if (owner == 1'b0)
                            c0_done <= 1'b1;
                        else
                            c1_done <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
