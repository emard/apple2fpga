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

#from time import ticks_ms, sleep_ms
from machine import SPI, Pin, SDCard # , enable_irq, disable_irq
from micropython import const

class disk2:
  def __init__(self):
    self.diskfilename = "disk2.ndo"
    print("DISK ][ %s" % self.diskfilename)
    self.tracklen = const(6656)
    self.trackbuf = bytearray(self.tracklen)
    self.led = Pin(5, Pin.OUT)
    self.led.off()
    self.diskfile = open(self.diskfilename, "rb")
    self.spi_channel = const(1)
    self.init_pinout_sd()
    self.spi_freq = const(2000000)
    self.csn = Pin(self.gpio_csn, Pin.OUT)
    self.csn.on()
    self.hwspi=SPI(self.spi_channel, baudrate=self.spi_freq, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=Pin(self.gpio_sck), mosi=Pin(self.gpio_mosi), miso=Pin(self.gpio_miso))
    self.count = 0
    self.count_prev = 0
    self.track_change = Pin(17, Pin.IN, Pin.PULL_UP)
    self.track_change.irq(trigger=Pin.IRQ_RISING, handler=self.irq_handler)

  @micropython.viper
  def init_pinout_sd(self):
    self.gpio_csn  = const(16)
    self.gpio_sck  = const(14)
    self.gpio_mosi = const(15)
    self.gpio_miso = const(2)

  @micropython.viper
  def irq_handler(self, pin):
    self.count += const(1)

  #@micropython.viper
  def run(self):
    while(True):
      if self.count != self.count_prev:
        self.led.on()
        track = self.hwspi.read(1)[0]
        self.led.off()
        self.diskfile.seek(self.tracklen * track)
        self.diskfile.readinto(self.trackbuf)
        self.led.on()
        self.hwspi.write(self.trackbuf)
        self.led.off()
        self.count_prev = self.count
        #for i in range(16):
        #  print("%02X " % self.trackbuf[i], end="")
        #print("(track %d)" % track)

d=disk2()
d.run()
#d.led.on(); d.hwspi.write(bytearray([0xd0,0xb2,0x02,0x59,0x17,0x69,0x91,0x9f]));d.led.off()
