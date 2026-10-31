//-------------------------------------------------------------------
// Synthesizable drop-in replacement for lib/behave/mem/ram_2p.v.
//
// Same ports, parameters and cycle semantics as the behavioral model
// (write: !cen && !wen; read: !cen && wen, synchronous), but without
// X/Z assignments so Vivado infers block RAM. Output holds its last
// value when the port is disabled - consumers in this design gate
// reads with rden, so the difference to the 'bx of the behavioral
// model is not observable.
//-------------------------------------------------------------------
module ram_2p (
    clka,
    cena_i,
    oena_i,
    wena_i,
    addra_i,
    dataa_o,
    dataa_i,
    clkb,
    cenb_i,
    oenb_i,
    wenb_i,
    addrb_i,
    datab_o,
    datab_i
);

parameter  Word_Width = 32;
parameter  Addr_Width = 8;

input                     clka;
input                     cena_i;
input                     oena_i;   // unused: output always driven
input                     wena_i;   // low active write
input  [Addr_Width-1:0]   addra_i;
output [Word_Width-1:0]   dataa_o;
input  [Word_Width-1:0]   dataa_i;

input                     clkb;
input                     cenb_i;
input                     oenb_i;   // unused
input                     wenb_i;   // low active write
input  [Addr_Width-1:0]   addrb_i;
output [Word_Width-1:0]   datab_o;
input  [Word_Width-1:0]   datab_i;

reg [Word_Width-1:0] mem_array [(1<<Addr_Width)-1:0];
reg [Word_Width-1:0] dataa_r, datab_r;

always @(posedge clka) begin
    if (!cena_i) begin
        if (!wena_i)
            mem_array[addra_i] <= dataa_i;
        else
            dataa_r <= mem_array[addra_i];
    end
end

always @(posedge clkb) begin
    if (!cenb_i) begin
        if (!wenb_i)
            mem_array[addrb_i] <= datab_i;
        else
            datab_r <= mem_array[addrb_i];
    end
end

// power-up zero (maps to BRAM INIT on FPGA, matches the behavioral
// model's zero-init in simulation)
integer zi;
initial begin
    for (zi = 0; zi < (1<<Addr_Width); zi = zi + 1)
        mem_array[zi] = {Word_Width{1'b0}};
end

assign dataa_o = dataa_r;
assign datab_o = datab_r;

endmodule
