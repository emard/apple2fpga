# ******* project, board and chip name *******
PROJECT = apple2
BOARD = ffmlfe5
# 12 25 45 85
FPGA_SIZE = 85

FPGA_PACKAGE = 6bg554c
# config flash: 1:SPI (standard), 4:QSPI (quad)
FLASH_SPI = 4
# chip: is25lp032d is25lp128f s25fl164k
FLASH_CHIP = is25lp128f

# ******* if programming with OpenOCD *******
# using local latest openocd until in linux distribution
OPENOCD=openocd
# default onboard usb-jtag
OPENOCD_INTERFACE=$(SCRIPTS)/ft4232.ocd

# ******* design files *******
CONSTRAINTS = ../constraints/FFM-LFE5U-V0r0_mit_FFC-CA7-V2r0.lpf
#TOP_MODULE = ulx3s_apple2
TOP_MODULE = ffmlfe5_apple2
TOP_MODULE_FILE = ../../../rtl_emard/lattice/top/$(TOP_MODULE).vhd

CLK3_NAME = clk_sys
CLK3_FILE_NAME = ../../../rtl_emard/lattice/ecp5/clocks/$(CLK3_NAME).v
CLK3_OPTIONS = \
  --module=$(CLK3_NAME) \
  --clkin=100 \
  --clkout0_name=clk_25          --clkout0=25

VERILOG_FILES = $(CLK3_FILE_NAME)

VHDL_FILES = \
  $(TOP_MODULE_FILE) \
  ../../../rtl_emard/lattice/ecp5/clocks/clk_sys_vhdl.vhd \
  ../../../rtl_emard/lattice/top/ulx3s_apple2.vhd \
  ../../../rtl_emard/lattice/ecp5/clocks/clk_25_140_28_14.vhd \
  ../../../rtl_emard/oled/oled_hex_decoder.vhd \
  ../../../rtl_emard/oled/oled_init_pack.vhd \
  ../../../rtl_emard/oled/oled_font_pack.vhd \
  ../../../rtl_emard/bram/bram_true2p_1clk.vhd \
  ../../../rtl_emard/dvi/vga2dvid.vhd \
  ../../../rtl_emard/dvi/tmds_encoder.vhd \
  ../../../apple2.vhd \
  ../../../main_roms.vhd \
  ../../../character_rom.vhd \
  ../../../cpu6502.vhd \
  ../../../keyboard.vhd \
  ../../../PS2_Ctrl.vhd \
  ../../../timing_generator.vhd \
  ../../../vga_controller.vhd \
  ../../../video_generator.vhd \
  ../../../disk_ii.vhd \
  ../../../disk_ii_rom.vhd \
  ../../../spi_controller.vhd \
  ../../../disk2_spi_slave.vhd \

SCRIPTS = ../scripts
include $(SCRIPTS)/trellis_path.mk
include $(SCRIPTS)/diamond_path.mk
include $(SCRIPTS)/diamond_main.mk
