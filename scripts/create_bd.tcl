#--------------------------------------------------------------------
# create_bd.tcl - Build the xk264 encoder block design for a
# Zynq-7000 (xc7z010clg400-1 class, "zynq7100") board.
#
# Usage (Vivado 2020.1+):
#   vivado -mode batch -source scripts/create_bd.tcl
#
# Result: xk264_proj/ project with BD xk264_bd
#   PS7 M_AXI_GP0 -> AXI interconnect -> xk264 S_AXI_LITE  (0x43C0_0000)
#   xk264 M_AXI (64-bit) -> AXI interconnect -> PS7 S_AXI_HP0 (DDR)
#   FCLK_CLK0 100 MHz system clock, FCLK_RESET0_N + proc_sys_reset
#   xk264 irq -> PS7 IRQ_F2P[0]
#--------------------------------------------------------------------

set proj_name  xk264_proj
set bd_name    xk264_bd
set part_name  xc7z010clg400-1   ;# change for 7z015/7z020 etc.
set repo_root  [file normalize [file join [file dirname [info script]] ..]]

#--------------------------------------------------------------------
# Project
#--------------------------------------------------------------------
create_project $proj_name [file join $repo_root $proj_name] -part $part_name -force

# all encoder + AXI wrapper RTL
add_files -norecurse [glob -directory [file join $repo_root hdl wrapper] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core top]  *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core cavlc] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core db] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core fetch] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core fme] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core ime] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core intra] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core mc] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core mem] *.v]
add_files -norecurse [glob -directory [file join $repo_root hdl core tq] *.v]
add_files -norecurse [file join $repo_root hdl core enc_defines.vh]
add_files -norecurse [glob -directory [file join $repo_root lib synth mem] *.v]

# include path for `include "enc_defines.vh"
set_property include_dirs [list [file join $repo_root hdl core]] [current_fileset]
# synthesizable RAM models (ram_2p/rf_2p/ram_1p) instantiate from RTL_MODEL
set_property verilog_define {RTL_MODEL=1 BIT_DEPTH=8} [current_fileset]

update_compile_order -fileset sources_1

#--------------------------------------------------------------------
# Block design
#--------------------------------------------------------------------
create_bd_design $bd_name

# ---------------- PS7 ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7

# 100 MHz FCLK, HP0 64-bit slave, no unused peripherals
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0       {1} \
    CONFIG.PCW_USE_S_AXI_HP0       {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {0} \
    CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
    CONFIG.PCW_TTC0_PERIPHERAL_ENABLE {0} \
    CONFIG.PCW_EN_CLK0_PORT        {1} \
    CONFIG.PCW_EN_RST0_PORT        {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_IRQ_F2P_INTR        {1} \
    CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3} \
    CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8} \
] [get_bd_cells ps7]

# ---------------- encoder wrapper (module reference) ----------------
create_bd_cell -type module -reference xk264_axi_top xk264
set_property CONFIG.FREQ_HZ 100000000 [get_bd_pins /xk264/aclk]

# ---------------- AXI interconnects ----------------
# GP0 -> Lite control
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_lite_ic
set_property -dict [list CONFIG.NUM_MI {1} CONFIG.NUM_SI {1}] [get_bd_cells axi_lite_ic]

# HP0 <- 64-bit DMA master
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hp_ic
set_property -dict [list CONFIG.NUM_MI {1} CONFIG.NUM_SI {1}] [get_bd_cells axi_hp_ic]

# ---------------- reset ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_sys

connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins ps7/M_AXI_GP0_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins ps7/S_AXI_HP0_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins xk264/aclk]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_lite_ic/ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_lite_ic/S00_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_lite_ic/M00_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_hp_ic/ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_hp_ic/S00_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins axi_hp_ic/M00_ACLK]
connect_bd_net [get_bd_pins ps7/FCLK_CLK0]      [get_bd_pins rst_sys/slowest_sync_clk]

connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N]  [get_bd_pins rst_sys/ext_reset_in]
connect_bd_net [get_bd_pins rst_sys/interconnect_aresetn] [get_bd_pins axi_lite_ic/ARESETN]
connect_bd_net [get_bd_pins rst_sys/interconnect_aresetn] [get_bd_pins axi_hp_ic/ARESETN]
connect_bd_net [get_bd_pins rst_sys/peripheral_aresetn]   [get_bd_pins axi_lite_ic/S00_ARESETN]
connect_bd_net [get_bd_pins rst_sys/peripheral_aresetn]   [get_bd_pins axi_lite_ic/M00_ARESETN]
connect_bd_net [get_bd_pins rst_sys/peripheral_aresetn]   [get_bd_pins axi_hp_ic/S00_ARESETN]
connect_bd_net [get_bd_pins rst_sys/peripheral_aresetn]   [get_bd_pins axi_hp_ic/M00_ARESETN]
connect_bd_net [get_bd_pins rst_sys/peripheral_aresetn]   [get_bd_pins xk264/aresetn]

# ---------------- AXI-Lite control path ----------------
connect_bd_intf_net [get_bd_intf_pins ps7/M_AXI_GP0]      [get_bd_intf_pins axi_lite_ic/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_lite_ic/M00_AXI] [get_bd_intf_pins xk264/S_AXI_LITE]

# ---------------- AXI4 64-bit DMA path ----------------
connect_bd_intf_net [get_bd_intf_pins xk264/M_AXI]        [get_bd_intf_pins axi_hp_ic/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_hp_ic/M00_AXI]  [get_bd_intf_pins ps7/S_AXI_HP0]

# ---------------- interrupt ----------------
connect_bd_net [get_bd_pins xk264/irq] [get_bd_pins ps7/IRQ_F2P]

# ---------------- addresses ----------------
# control registers @ 0x43C0_0000 (64 kB)
assign_bd_address -offset 0x43C00000 -range 0x10000 \
    [get_bd_addr_segs xk264/S_AXI_LITE/reg0]
# DMA window: full DDR through HP0
assign_bd_address -offset 0x00000000 -range 0x20000000 \
    [get_bd_addr_segs ps7/S_AXI_HP0/HP0_DDR_LOWOCM]

set_property range 512M [get_bd_addr_segs xk264/M_AXI/Reg]
set_property offset 0x00000000 [get_bd_addr_segs xk264/M_AXI/Reg]

validate_bd_design
save_bd_design

# ---------------- top-level wrapper ----------------
set wrapper [make_wrapper -files [get_files ${bd_name}.bd] -top]
add_files -norecurse $wrapper
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "BD '$bd_name' created. Next:"
puts "  launch_runs synth_1 impl_1 -to_step write_bitstream"
puts "  (export hardware: write_hw_platform -fixed ... )"
