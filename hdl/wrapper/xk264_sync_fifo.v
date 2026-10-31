//-------------------------------------------------------------------
// First-word-fall-through synchronous FIFO.
//-------------------------------------------------------------------
module xk264_sync_fifo #(
    parameter DW = 64,
    parameter AW = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clr,
    input  wire         wr_en,
    input  wire [DW-1:0] wr_data,
    output wire         full,
    input  wire         rd_en,
    output wire [DW-1:0] rd_data,
    output wire         empty,
    output wire [AW:0]  count
);

    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0] wp, rp;

    assign full    = (wp[AW] != rp[AW]) && (wp[AW-1:0] == rp[AW-1:0]);
    assign empty   = (wp == rp);
    assign rd_data = mem[rp[AW-1:0]];
    assign count   = wp - rp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= {AW+1{1'b0}};
            rp <= {AW+1{1'b0}};
        end else if (clr) begin
            wp <= {AW+1{1'b0}};
            rp <= {AW+1{1'b0}};
        end else begin
            if (wr_en && !full) begin
                mem[wp[AW-1:0]] <= wr_data;
                wp <= wp + 1'b1;
            end
            if (rd_en && !empty)
                rp <= rp + 1'b1;
        end
    end

endmodule
