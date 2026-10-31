//-------------------------------------------------------------------
// Two-client to one AXI4 read-channel arbiter.
// Client 0 has priority. One outstanding burst at a time.
// Clients must hold cmd_valid/cmd_addr/cmd_beats stable until
// cmd_accept pulses, and must always be able to sink returned data
// (data_valid is asserted without backpressure).
//-------------------------------------------------------------------
module xk264_axi_rd_arb (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        c0_cmd_valid,
    input  wire [31:0] c0_cmd_addr,
    input  wire [4:0]  c0_cmd_beats,
    output reg         c0_cmd_accept,
    output wire        c0_data_valid,
    output wire        c0_data_last,
    output wire [63:0] c0_data,

    input  wire        c1_cmd_valid,
    input  wire [31:0] c1_cmd_addr,
    input  wire [4:0]  c1_cmd_beats,
    output reg         c1_cmd_accept,
    output wire        c1_data_valid,
    output wire        c1_data_last,
    output wire [63:0] c1_data,

    output reg         rd_err,

    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    output reg  [31:0] m_axi_araddr,
    output reg  [7:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output wire [0:0]  m_axi_arlock,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output wire [3:0]  m_axi_arqos,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,
    input  wire [63:0] m_axi_rdata,
    input  wire        m_axi_rlast,
    input  wire [1:0]  m_axi_rresp
);

    localparam S_IDLE = 2'd0, S_AR = 2'd1, S_RD = 2'd2;

    reg [1:0] state;
    reg       owner; // 0: c0, 1: c1

    assign m_axi_arsize  = 3'd3;    // 8 bytes
    assign m_axi_arburst = 2'd1;    // INCR
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_rready  = 1'b1;    // clients always sink data

    assign c0_data_valid = m_axi_rvalid && (owner == 1'b0);
    assign c1_data_valid = m_axi_rvalid && (owner == 1'b1);
    assign c0_data_last  = m_axi_rlast && (owner == 1'b0);
    assign c1_data_last  = m_axi_rlast && (owner == 1'b1);
    assign c0_data = m_axi_rdata;
    assign c1_data = m_axi_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            owner         <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_araddr  <= 32'b0;
            m_axi_arlen   <= 8'b0;
            c0_cmd_accept <= 1'b0;
            c1_cmd_accept <= 1'b0;
            rd_err        <= 1'b0;
        end else begin
            c0_cmd_accept <= 1'b0;
            c1_cmd_accept <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (c0_cmd_valid) begin
                        m_axi_araddr  <= c0_cmd_addr;
                        m_axi_arlen   <= {3'b0, c0_cmd_beats} - 8'd1;
                        owner         <= 1'b0;
                        m_axi_arvalid <= 1'b1;
                        state         <= S_AR;
                    end else if (c1_cmd_valid) begin
                        m_axi_araddr  <= c1_cmd_addr;
                        m_axi_arlen   <= {3'b0, c1_cmd_beats} - 8'd1;
                        owner         <= 1'b1;
                        m_axi_arvalid <= 1'b1;
                        state         <= S_AR;
                    end
                end
                S_AR: begin
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        if (owner == 1'b0)
                            c0_cmd_accept <= 1'b1;
                        else
                            c1_cmd_accept <= 1'b1;
                        state <= S_RD;
                    end
                end
                S_RD: begin
                    if (m_axi_rvalid && m_axi_rlast) begin
                        if (m_axi_rresp != 2'b00)
                            rd_err <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
