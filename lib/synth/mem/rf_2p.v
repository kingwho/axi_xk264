//-------------------------------------------------------------------
// Synthesizable drop-in replacement for lib/behave/mem/rf_2p.v.
// Simple dual-port RAM: port A read-only, port B write-only.
//-------------------------------------------------------------------
module rf_2p (
    clka,
    cena_i,
    addra_i,
    dataa_o,
    clkb,
    cenb_i,
    wenb_i,
    addrb_i,
    datab_i
);

parameter  Word_Width = 32;
parameter  Addr_Width = 8;

input                     clka;
input                     cena_i;   // low active read enable
input  [Addr_Width-1:0]   addra_i;
output [Word_Width-1:0]   dataa_o;

input                     clkb;
input                     cenb_i;   // low active chip enable
input                     wenb_i;   // low active write
input  [Addr_Width-1:0]   addrb_i;
input  [Word_Width-1:0]   datab_i;

reg [Word_Width-1:0] mem_array [(1<<Addr_Width)-1:0];
reg [Word_Width-1:0] dataa_r;

always @(posedge clka) begin
    if (!cena_i)
        dataa_r <= mem_array[addra_i];
end

always @(posedge clkb) begin
    if (!cenb_i && !wenb_i)
        mem_array[addrb_i] <= datab_i;
end

// power-up zero (maps to BRAM INIT on FPGA, matches the behavioral
// model's zero-init in simulation)
integer zi;
initial begin
    for (zi = 0; zi < (1<<Addr_Width); zi = zi + 1)
        mem_array[zi] = {Word_Width{1'b0}};
end

assign dataa_o = dataa_r;

endmodule
