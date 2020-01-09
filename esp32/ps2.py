# micropython ESP32
# PS/2 protocol emulator (keyboard and mouse, transmit-only)

# AUTHOR=EMARD
# LICENSE=BSD

from time import sleep_us
from machine import Pin
from micropython import const
from uctypes import addressof

class ps2:
  def __init__(self, clk=26, data=25, qbit_us=16, byte_us=150, f0_us=50000, n=4, n_us=1000):
    self.gpio_ps2_clk  = clk
    self.gpio_ps2_data = data
    self.init_pins()
    self.qdelay  = qbit_us # quarter-bit delay
    self.cdelay  = byte_us # byte-to-byte delay
    self.f0delay = f0_us   # additional delay before 0xF0 and a after a byte after 0xF0
    self.ncount  = n       # after each N bytes:
    self.ndelay  = n_us    # delay after each N bytes (for mouse)

  def init_pins(self):
    #self.ps2_clk  = Pin(self.gpio_ps2_clk,  Pin.OPEN_DRAIN, Pin.PULL_UP)
    #self.ps2_data = Pin(self.gpio_ps2_data, Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_clk  = Pin(self.gpio_ps2_clk,  Pin.OUT)
    self.ps2_data = Pin(self.gpio_ps2_data, Pin.OUT)
    self.ps2_clk.on()
    self.ps2_data.on()

  @micropython.viper
  def write(self, data):
    qdelay = int(self.qdelay)
    f0delay = int(self.f0delay)
    ndelay = int(self.ndelay)
    p = ptr8(addressof(data))
    l = int(len(data))
    f0c = 0
    nc = int(self.ncount)
    for i in range(l):
      val = p[i]
      if val == 0xF0 and f0delay > 0:
        sleep_us(f0delay)
        f0c = 2
      parity = 1
      self.ps2_data.off()
      sleep_us(qdelay)
      self.ps2_clk.off()
      sleep_us(qdelay+qdelay)
      self.ps2_clk.on()
      sleep_us(qdelay)
      for nf in range(8):
        if val & 1:
          self.ps2_data.on()
          parity ^= 1
        else:
          self.ps2_data.off()
          parity ^= 0 # keep timing the same as above
        sleep_us(qdelay)
        self.ps2_clk.off()
        val >>= 1
        sleep_us(qdelay+qdelay)
        self.ps2_clk.on()
        sleep_us(qdelay)
      if parity:
        self.ps2_data.on()
      else:
        self.ps2_data.off()
      sleep_us(qdelay)
      self.ps2_clk.off()
      sleep_us(qdelay+qdelay)
      self.ps2_clk.on()
      sleep_us(qdelay)
      self.ps2_data.on()
      sleep_us(qdelay)
      self.ps2_clk.off()
      sleep_us(qdelay+qdelay)
      self.ps2_clk.on()
      sleep_us(self.cdelay)
      if f0c > 0:
        f0c -= 1
        if f0c == 0:
          sleep_us(f0delay)
      if nc > 0:
        nc -= 1
        if nc == 0:
          nc = int(self.ncount)
          sleep_us(ndelay)
