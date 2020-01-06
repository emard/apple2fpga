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
from micropython import const

class disk2:
  def __init__(self, file_nib):
    self.diskfilename = file_nib
    print("DISK ][ %s" % self.diskfilename)
    self.tracklen = const(6656)
    self.trackbuf = bytearray(self.tracklen)
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
    self.track_change.irq(trigger=Pin.IRQ_FALLING, handler=self.irq_handler)

  @micropython.viper
  def init_pinout_sd(self):
    self.gpio_sck  = const(16)
    self.gpio_mosi = const(4)
    self.gpio_miso = const(12)

  @micropython.viper
  def irq_handler(self, pin):
#    self.count += const(1)
        self.led.on()
        track = self.hwspi.read(1)[0]
        self.led.off()
        self.diskfile.seek(self.tracklen * track)
        self.diskfile.readinto(self.trackbuf)
        self.led.on()
        self.hwspi.write(self.trackbuf)
        self.led.off()

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

# debug to manually write data and
# check them with *C0EC
#d.led.on(); d.hwspi.write(bytearray([0xd0,0xb2,0x02,0x59,0x17,0x69,0x91,0x9f]));d.led.off()

#import ecp5
#ecp5.prog("apple2.bit.gz")
d=disk2("disk2.nib")
#d.run()
