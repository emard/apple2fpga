# micropython ESP32
# PS/2 wire protocol emulator (keyboard and mouse)

# AUTHOR=EMARD
# LICENSE=BSD

from time import sleep_us
from machine import Pin
from micropython import const
from uctypes import addressof

class ps2:
  def __init__(self):
    self.init_pinout() # communicate using SD card pins when SD is inactive
    self.init_pins()
    self.qdelay = const(14) # quarter-bit delay

  @micropython.viper
  def init_pinout(self):
    self.gpio_ps2_clk  = const(16)
    self.gpio_ps2_data = const(17)
    #self.gpio_ps2_clk  = const(14) # sd_clk
    #self.gpio_ps2_data = const(15) # sd_cmd

  def init_pins(self):
    self.ps2_clk  = Pin(self.gpio_ps2_clk,  Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_data = Pin(self.gpio_ps2_data, Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_clk.on()
    self.ps2_data.on()

  @micropython.viper
  def write(self, data):
    p = ptr8(addressof(data))
    l = int(len(data))
    for i in range(l):
      val = p[i]
      parity = 1
      self.ps2_data.off()
      sleep_us(self.qdelay)
      self.ps2_clk.off()
      sleep_us(self.qdelay+self.qdelay)
      self.ps2_clk.on()
      sleep_us(self.qdelay)
      for nf in range(8):
        if val & 1:
          self.ps2_data.on()
          parity ^= 1
        else:
          self.ps2_data.off()
          parity ^= 0 # keep timing the same as above
        sleep_us(self.qdelay)
        self.ps2_clk.off()
        val >>= 1
        sleep_us(self.qdelay+self.qdelay)
        self.ps2_clk.on()
        sleep_us(self.qdelay)
      if parity:
        self.ps2_data.on()
      else:
        self.ps2_data.off()
      sleep_us(self.qdelay)
      self.ps2_clk.off()
      sleep_us(self.qdelay+self.qdelay)
      self.ps2_clk.on()
      sleep_us(self.qdelay)
      self.ps2_data.on()
      sleep_us(self.qdelay)
      self.ps2_clk.off()
      sleep_us(self.qdelay+self.qdelay)
      self.ps2_clk.on()
      sleep_us(self.qdelay)
