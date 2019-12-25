# ******* project, board and chip name *******
PROJECT = apple2
BOARD = ulx3s
# 12 25 45 85
FPGA_SIZE = 25

FPGA_PACKAGE = 6bg381c
# config flash: 1:SPI (standard), 4:QSPI (quad)
FLASH_SPI = 4
# chip: is25lp032d is25lp128f s25fl164k
FLASH_CHIP = is25lp128f

# ******* if programming with OpenOCD *******
# using local latest openocd until in linux distribution
OPENOCD=openocd_ft232r
# default onboard usb-jtag
OPENOCD_INTERFACE=$(SCRIPTS)/ft231x.ocd
# ulx3s-jtag-passthru
#OPENOCD_INTERFACE=$(SCRIPTS)/ft231x2.ocd
# ulx2s
#OPENOCD_INTERFACE=$(SCRIPTS)/ft232r.ocd
# external jtag
#OPENOCD_INTERFACE=$(SCRIPTS)/ft2232.ocd

# ******* design files *******
CONSTRAINTS = ../constraints/ulx3s_v20_segpdi.lpf
#TOP_MODULE = ulx3s_apple2
TOP_MODULE = ulx3s_apple2
TOP_MODULE_FILE = ../../../rtl_emard/lattice/top/$(TOP_MODULE).vhd

VERILOG_FILES =

VHDL_FILES = \
  $(TOP_MODULE_FILE) \
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
