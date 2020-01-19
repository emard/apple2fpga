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
from uctypes import addressof
import os

class disk2:
  def __init__(self, file_nib):
    self.diskfilename = file_nib
    print("DISK ][ %s" % self.diskfilename)
    self.screen_x = const(64)
    self.screen_y = const(20)
    self.init_fb()
    self.exp_names = " KMGTE"
    self.highlight = bytearray([32,16,42]) # space, right triangle, asterisk
    self.read_dir()
    self.trackbuf = bytearray(6656)
    self.spi_read_track_irq = bytearray([1,0,0,0,0])
    self.spi_result = bytearray(5)
    self.spi_write_track = bytearray([0,0,0])
    self.spi_enable_osd = bytearray([0,0xFE,0,1])
    self.spi_write_osd = bytearray([0,0xF0,0])
    self.spi_read_btn = bytearray([1,0xFE,0,0,0])
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

# init file browser
  def init_fb(self):
    self.fb_topitem = 0
    self.fb_cursor = 0
    self.fb_selected = -1

  @micropython.viper
  def init_pinout_sd(self):
    self.gpio_sck  = const(16)
    self.gpio_mosi = const(4)
    self.gpio_miso = const(12)

  @micropython.viper
  def irq_handler(self, pin):
    p8result = ptr8(addressof(self.spi_result))
    self.led.on()
    self.hwspi.write_readinto(self.spi_read_track_irq, self.spi_result)
    self.led.off()
    track_irq = p8result[4]
    if track_irq & 0x40:
      track = track_irq & 0x3F
      self.diskfile.seek(6656 * track)
      self.diskfile.readinto(self.trackbuf)
      self.led.on()
      self.hwspi.write(self.spi_write_track)
      self.hwspi.write(self.trackbuf)
      self.led.off()
    if track_irq & 0x80:
      self.led.on()
      self.hwspi.write_readinto(self.spi_read_btn, self.spi_result)
      self.led.off()
      btn = p8result[4]
      self.osd_enable((btn & 2) >> 1)
      p8result[0] = 48 + ((btn &  4)>>2)
      p8result[1] = 48 + ((btn &  8)>>3)
      p8result[2] = 48 + ((btn & 16)>>4)
      p8result[3] = 48 + ((btn & 32)>>5)
      p8result[4] = 48 + ((btn & 64)>>6)
      self.osd_print(10,5,self.spi_result)
      if btn&4:
        #self.refresh_dir_line(0,0)
        #self.refresh_dir_line(1,0)
        #self.refresh_dir_line(7,0)
        self.show_directory()
      if btn&8: # cursor up
        self.move_dir_cursor(-1)
      if btn&16: # cursor down
        self.move_dir_cursor(1)
      if btn&64: # cursor right
        oldselected = self.fb_selected - self.fb_topitem
        self.fb_selected = self.fb_cursor
        self.show_dir_line(oldselected)
        self.show_dir_line(self.fb_cursor - self.fb_topitem)

  @micropython.viper
  def osd_enable(self, en:int):
    pena = ptr8(addressof(self.spi_enable_osd))
    pena[3] = en&1
    self.led.on()
    self.hwspi.write(self.spi_enable_osd)
    self.led.off()

  @micropython.viper
  def osd_print(self, x:int, y:int, text):
    p8msg=ptr8(addressof(self.spi_write_osd))
    a=0xF000+(x&63)+((y&31)<<6)
    p8msg[1]=a>>8
    p8msg[2]=a
    self.led.on()
    self.hwspi.write(self.spi_write_osd)
    self.hwspi.write(text)
    self.led.off()

  @micropython.viper
  def osd_cls(self):
    p8msg=ptr8(addressof(self.spi_write_osd))
    p8msg[1]=0xF0
    p8msg[2]=0
    self.led.on()
    self.hwspi.write(self.spi_write_osd)
    self.hwspi.read(1280,32)
    self.led.off()

  # moving cursor inside screen, no scrolling
  #def refresh_dir_line(self, cursor, highlight):
  #  screen_line = cursor - self.fb_topitem
  #  if screen_line < 0 or screen_line >= self.screen_y:
  #    return
  #  self.osd_print(0, screen_line, self.direntries[cursor][0])

  # y is actual line on the screen
  def show_dir_line(self, y):
    if y < 0 or y >= self.screen_y:
      return
    highlight = 0
    if y == self.fb_cursor - self.fb_topitem:
      highlight = 1
    if y == self.fb_selected - self.fb_topitem:
      highlight = 2
    i = y+self.fb_topitem
    if i >= len(self.direntries):
      self.osd_print(0,y,"%64s" % "")
      return
    if self.direntries[i][1]: # directory
      self.osd_print(0,y,"%c%-57s     D" % (self.highlight[highlight],self.direntries[i][0]))
    else: # file
      mantissa = self.direntries[i][2]
      exponent = 0
      while mantissa >= 1024:
        mantissa >>= 10
        exponent += 1
      self.osd_print(0,y,"%c%-57s %4d%c" % (self.highlight[highlight],self.direntries[i][0], mantissa, self.exp_names[exponent]))

  def show_directory(self):
    for i in range(self.screen_y):
      self.show_dir_line(i)

  def move_dir_cursor(self, step):
    oldcursor = self.fb_cursor
    if step == 1:
      if self.fb_cursor < len(self.direntries)-1:
        self.fb_cursor += 1
    if step == -1:
      if self.fb_cursor > 0:
        self.fb_cursor -= 1
    if oldcursor != self.fb_cursor:
      screen_line = self.fb_cursor - self.fb_topitem
      if screen_line >= 0 and screen_line < self.screen_y: # move cursor inside screen, no scroll
        self.show_dir_line(oldcursor - self.fb_topitem) # no highlight
        self.show_dir_line(screen_line) # highlight
      else: # scroll
        if screen_line < 0: # cursor going up
          screen_line = 0
          if self.fb_topitem > 0:
            self.fb_topitem -= 1
            self.show_directory()
        else: # cursor going down
          screen_line = self.screen_y-1
          if self.fb_topitem+self.screen_y < len(self.direntries):
            self.fb_topitem += 1
            self.show_directory()

  def read_dir(self):
    self.direntries = []
    self.cwd = os.getcwd()
    ls = sorted(os.listdir(self.cwd))
    for fname in ls:
      stat = os.stat(fname)
      if stat[0] & 0o170000 == 0o040000:
        self.direntries.append([fname,1,0]) # directory
      else:
        self.direntries.append([fname,0,stat[6]]) # file

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
