# ******* project, board and chip name *******
PROJECT = apple2
BOARD = ulx3s
# 12 25 45 85
FPGA_SIZE = 85

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
CONSTRAINTS = ../constraints/ulx3s_v20_difgpdi.lpf
TOP_MODULE = ulx3s_v20_apple2
#CONSTRAINTS = ../constraints/ulx3s_v31.lpf
#TOP_MODULE = ulx3s_v31_apple2
TOP_MODULE_FILE = ../../../rtl_emard/lattice/top/$(TOP_MODULE).vhd

VHDL_FILES = \
$(TOP_MODULE_FILE) \
../../../rtl_emard/lattice/ecp5/clocks/ecp5pll.vhd \
../../../rtl_emard/oled/oled_hex_decoder.vhd \
../../../rtl_emard/oled/oled_init_pack.vhd \
../../../rtl_emard/oled/oled_font_pack.vhd \
../../../rtl_emard/osd/osd_vhd.vhd \
../../../rtl_emard/osd/spi_ram_btn.vhd \
../../../rtl_emard/spi_display/spi_display.vhd \
../../../rtl_emard/spi_display/spi_display_init_pack.vhd \
../../../rtl_emard/spi_display/st7789_init_pack.vhd \
../../../rtl_emard/bram/bram_true2p_1clk.vhd \
../../../rtl_emard/dvi/vga2dvid.vhd \
../../../rtl_emard/dvi/tmds_encoder.vhd \
../../../rtl_emard/usb/usb11_phy_vhdl/usb_phy.vhd \
../../../rtl_emard/usb/usb11_phy_vhdl/usb_rx_phy.vhd \
../../../rtl_emard/usb/usb11_phy_vhdl/usb_tx_phy.vhd \
../../../rtl_emard/usb/usbhost/usbh_setup_pack.vhd \
../../../rtl_emard/usb/usbhost/usbh_host_hid.vhd \
../../../rtl_emard/usb/usbhost/usbh_sie_vhdl.vhd \
../../../rtl_emard/usb/usbhid/report_decoded_pack_generic.vhd \
../../../rtl_emard/usb/usbhid/usbhid_report_decoder_darfon_joystick.vhd \
../../../apple2.vhd \
../../../timing_generator.vhd \
../../../main_roms.vhd \
../../../character_rom.vhd \
../../../cpu6502.vhd \
../../../keyboard.vhd \
../../../PS2_Ctrl.vhd \
../../../paddle.vhd \
../../../vga_controller.vhd \
../../../video_generator.vhd \
../../../disk_ii.vhd \
../../../disk_ii_rom.vhd \
../../../spi_controller.vhd \
../../../rtl_emard/osd/spi_osd.vhd \
../../../rtl_emard/osd/spirw_slave.vhd \

#  clocks/clk_25_shift_pixel.vhd \

VERILOG_FILES = \
../../../rtl_emard/osd/spi_osd_v.v \
../../../rtl_emard/osd/osd.v \
../../../rtl_emard/osd/spi_ram_btn_v.v \
../../../rtl_emard/osd/spirw_slave_v.v \
../../../rtl_emard/usb/usbhost/usbh_sie.v \
../../../rtl_emard/usb/usbhost/usbh_crc5.v \
../../../rtl_emard/usb/usbhost/usbh_crc16.v \

#  $(CLK0_FILE_NAME) \

SCRIPTS = ../scripts
include $(SCRIPTS)/trellis_path.mk
include $(SCRIPTS)/diamond_path.mk
include $(SCRIPTS)/diamond_main.mk

osd.mem: osd.txt
	hexdump -n 1280 -v -e '1/1 "%02x " "\n"' $< > $@
