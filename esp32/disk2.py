# micropython ESP32
# DISK ][ NIB disk image server, read only

# AUTHOR=EMARD
# LICENSE=BSD

# this code is SPI master to FPGA SPI slave
# FPGA sends pulse to GPIO after it changes track number
# on GPIO pin interrupt from FPGA:
# track_number = SPI_read_byte
# track_len = 6656 bytes
# seek(track_number*track_len)
# buffer = read(track_len)
# SPI_write(buffer)
# FPGA SPI slave reads track in BRAM buffer

from machine import SPI, Pin
from micropython import const, alloc_emergency_exception_buf

class disk2:
  def __init__(self, file_nib):
    self.diskfilename = file_nib
    print("DISK ][ %s" % self.diskfilename)
    self.tracklen = const(6656)
    self.trackbuf = bytearray(self.tracklen)
    self.spi_read_track_irq = bytearray([1,0,0,0,0])
    self.spi_result_track_irq = bytearray(5)
    self.spi_write_track = bytearray([0,0,0])
    self.led = Pin(5, Pin.OUT)
    self.led.off()
    self.diskfile = open(self.diskfilename, "rb")
    self.spi_channel = const(1)
    self.init_pinout_sd()
    self.spi_freq = const(2000000)
    self.hwspi=SPI(self.spi_channel, baudrate=self.spi_freq, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=Pin(self.gpio_sck), mosi=Pin(self.gpio_mosi), miso=Pin(self.gpio_miso))
    self.count = 0
    self.count_prev = 0
    self.track_change = Pin(0, Pin.IN, Pin.PULL_UP)
    alloc_emergency_exception_buf(100)
    self.irq_handler_ref = self.irq_handler # allocation happens here
    self.track_change.irq(trigger=Pin.IRQ_FALLING, handler=self.irq_handler_ref)

  @micropython.viper
  def init_pinout_sd(self):
    self.gpio_sck  = const(16)
    self.gpio_mosi = const(4)
    self.gpio_miso = const(12)

  @micropython.viper
  def irq_handler(self, pin):
    self.led.on()
    self.hwspi.write_readinto(self.spi_read_track_irq, self.spi_result_track_irq)
    track = self.spi_result_track_irq[4]
    self.led.off()
    self.diskfile.seek(self.tracklen * track)
    self.diskfile.readinto(self.trackbuf)
    self.led.on()
    self.hwspi.write(self.spi_write_track)
    self.hwspi.write(self.trackbuf)
    self.led.off()

  def osd(self, a):
    if len(a) > 0:
      enable = 1
    else:
      enable = 0
    self.led.on()
    self.hwspi.write(bytearray([0,0xFE,0,enable])) # enable OSD
    self.led.off()
    if enable:
      self.led.on()
      self.hwspi.write(bytearray([0,0xF0,0])) # write content
      self.hwspi.write(bytearray(a)) # write content
      self.led.off()

# debug to manually write data and
# check them with *C0EC
#d.led.on(); d.hwspi.write(bytearray([0xd0,0xb2,0x02,0x59,0x17,0x69,0x91,0x9f]));d.led.off()

#import ecp5
#ecp5.prog("apple2.bit.gz")
d=disk2("disk2.nib")
#d.run()
