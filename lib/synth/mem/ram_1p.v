//-------------------------------------------------------------------
// Synthesizable drop-in replacement for lib/behave/mem/ram_1p.v.
// Single-port RAM: write when !cen && !wen, read when !cen && wen.
//-------------------------------------------------------------------
module ram_1p (
    clk,
    cen_i,
    oen_i,
    wen_i,
    addr_i,
    data_i,
    data_o
);

parameter  Word_Width = 32;
parameter  Addr_Width = 8;

input                     clk;
input                     cen_i;
input                     oen_i;   // unused: output always driven
input                     wen_i;   // low active write
input  [Addr_Width-1:0]   addr_i;
input  [Word_Width-1:0]   data_i;
output [Word_Width-1:0]   data_o;

reg [Word_Width-1:0] mem_array [(1<<Addr_Width)-1:0];
reg [Word_Width-1:0] data_r;

always @(posedge clk) begin
    if (!cen_i) begin
        if (!wen_i)
            mem_array[addr_i] <= data_i;
        else
            data_r <= mem_array[addr_i];
    end
end

// power-up zero (maps to BRAM INIT on FPGA, matches the behavioral
// model's zero-init in simulation)
integer zi;
initial begin
    for (zi = 0; zi < (1<<Addr_Width); zi = zi + 1)
        mem_array[zi] = {Word_Width{1'b0}};
end

assign data_o = data_r;

endmodule
