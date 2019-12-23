# micropython ESP32
# DISK ][ server, read only

# AUTHOR=EMARD
# LICENSE=BSD

# TODO:
# this code is SPI master to FPGA SPI slave
# FPGA sends pulse to GPIO after it changes track number
# track_len = 6656 bytes
# on GPIO pin interrupt from FPGA:
# track_number = SPI_read_byte
# seek(track_number*track_len)
# buffer = read(track_len)
# SPI_write(buffer)
# FPGA SPI slave reads track in BRAM buffer

from time import ticks_ms, sleep_ms
from machine import SPI, Pin, SDCard, enable_irq, disable_irq
from micropython import const

class disk2:

  def __init__(self):
    self.diskfilename = "disk2.ndo"
    print("DISK ][ %s" % self.diskfilename)
    self.track = 0
    self.tracklen = const(6656)
    self.trackbuf = bytearray(self.tracklen)
    self.byte = 0
    self.count = 0
    self.led = Pin(5, Pin.OUT)
    self.led.off()
    self.gpio0 = Pin(0, Pin.IN)
    self.gpio0.irq(trigger=Pin.IRQ_FALLING, handler=self.irq_handler)
    self.diskfile = open(self.diskfilename, "r")
    self.spi_channel = const(1)
    self.init_pinout_sd()
    self.spi_freq = const(2000000)
    self.csn = Pin(self.gpio_csn, Pin.OUT)
    self.csn.on() # detach SD card from SPI bus
    self.hwspi=SPI(self.spi_channel, baudrate=self.spi_freq, polarity=1, phase=1, bits=8, firstbit=SPI.MSB, sck=Pin(self.gpio_sck), mosi=Pin(self.gpio_mosi), miso=Pin(self.gpio_miso))

  def init_pinout_sd(self):
    self.gpio_csn  = const(17) # FIXME
    self.gpio_sck  = const(14)
    self.gpio_mosi = const(13)
    self.gpio_miso = const(12)

  @micropython.viper
  def irq_handler(self, pin):
    self.led.on()
    self.track = self.hwspi.read(1)[0]
    self.diskfile.seek(self.tracklen * self.track)
    self.diskfile.readinto(self.trackbuf)
    self.hwspi.write(self.trackbuf)
    self.byte = self.trackbuf[256]
    self.led.off()

  def run(self, n=10):
    for i in range(n):
      sleep_ms(1000)
      print("%d, track=%d, byte=%02X" % (i, self.track, self.byte))
