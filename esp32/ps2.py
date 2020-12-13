# micropython ESP32
# PS/2 protocol emulator (keyboard and mouse, transmit-only)

# AUTHOR=EMARD
# LICENSE=BSD

from time import sleep_us
from machine import Pin
from micropython import const
from uctypes import addressof

class ps2:
  def __init__(self, kbd_clk=26, kbd_data=25, mouse_clk=17, mouse_data=16, qbit_us=16, byte_us=150):
    self.gpio_kbd_clk    = kbd_clk
    self.gpio_kbd_data   = kbd_data
    self.gpio_mouse_clk  = mouse_clk
    self.gpio_mouse_data = mouse_data
    self.keyboard()
    self.qbit_us = qbit_us # quarter-bit delay
    self.byte_us = byte_us # byte-to-byte delay

  def keyboard(self):
    self.ps2_clk  = Pin(self.gpio_kbd_clk,  Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_data = Pin(self.gpio_kbd_data, Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_clk.on()
    self.ps2_data.on()

  def mouse(self):
    self.ps2_clk  = Pin(self.gpio_mouse_clk,  Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_data = Pin(self.gpio_mouse_data, Pin.OPEN_DRAIN, Pin.PULL_UP)
    self.ps2_clk.on()
    self.ps2_data.on()

  @micropython.viper
  def write(self, data):
    qbit_us = int(self.qbit_us)
    p = ptr8(addressof(data))
    l = int(len(data))
    for i in range(l):
      val = p[i]
      parity = 1
      self.ps2_data.off()
      sleep_us(qbit_us)
      self.ps2_clk.off()
      sleep_us(qbit_us+qbit_us)
      self.ps2_clk.on()
      sleep_us(qbit_us)
      for nf in range(8):
        if val & 1:
          self.ps2_data.on()
          parity ^= 1
        else:
          self.ps2_data.off()
          parity ^= 0 # keep timing the same as above
        sleep_us(qbit_us)
        self.ps2_clk.off()
        val >>= 1
        sleep_us(qbit_us+qbit_us)
        self.ps2_clk.on()
        sleep_us(qbit_us)
      if parity:
        self.ps2_data.on()
      else:
        self.ps2_data.off()
      sleep_us(qbit_us)
      self.ps2_clk.off()
      sleep_us(qbit_us+qbit_us)
      self.ps2_clk.on()
      sleep_us(qbit_us)
      self.ps2_data.on()
      sleep_us(qbit_us)
      self.ps2_clk.off()
      sleep_us(qbit_us+qbit_us)
      self.ps2_clk.on()
      sleep_us(self.byte_us)
