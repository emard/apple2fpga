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

class gpint:

  def __init__(self):
    print("gpio0 interrupt test")
    self.count = 0
    self.led = Pin(5, Pin.OUT)
    self.led.on()
    self.gpio0 = Pin(0, Pin.IN)
    self.gpio0.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)

  def callback(self, pin):
    self.count += 1

  def test(self, n=10):
    for i in range(n):
      sleep_ms(100)
      print("%d, int=%d" % (i, self.count))
