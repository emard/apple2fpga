#!/usr/bin/env python3

# AUTHOR=EMARD
# LICENSE=GPL

# use ps2recv.py on ESP32 (set pinout at ps2recv.py)
# edit "mouse_wheel" (below)
# False: send 3-byte reports as no-wheel mouse (legacy/uninitialized PS/2)
# True:  send 4-byte reports as wheel mouse    (modern PS/2)


import pygame
import struct
import socket

tcp_host = "192.168.48.132"
tcp_port = 3252
mouse_wheel = False

ps2_tcp=socket.create_connection((tcp_host, tcp_port))
print("Sending keyboard/mouse events to %s:%s" % (tcp_host,tcp_port))
#ps2_tcp.sendall(bytearray([0xAA, 0x00, 0xFA]))
# mouse sends 0xAA 0x00 after being plugged
# 0xFA is ACK what mouse sends after being configured

def mouse_wheel_report(dx,dy,dz,btn_left,btn_middle,btn_right):
  return struct.pack("<BBBBBBBBH",
    ord('M'), 4, # 4-byte mouse packet
     (btn_left   & 1)     +
    ((btn_right  & 1)<<1) +
    ((btn_middle & 1)<<2) +
    (               1  << 3) +
    ((((  dx  & 0x100) >> 8) & 1)<<4) +
    (((((-dy) & 0x100) >> 8) & 1)<<5),
    dx & 0xFF, 
    (-dy) & 0xFF, 
    (-dz) & 0x0F,
    ord('W'), 2, # 2-byte wait value (us, LSB first)
    1000 # us wait
    )

def mouse_nowheel_report(dx,dy,btn_left,btn_middle,btn_right):
  return struct.pack("<BBBBBBBH",
    ord('M'), 3, # 3-byte mouse packet
    (btn_left & 1) + ((btn_right & 1)<<1) + ((btn_middle & 1)<<2) +
    (               1  << 3) +
    ((((  dx  & 0x100) >> 8) & 1)<<4) +
    (((((-dy) & 0x100) >> 8) & 1)<<5),
    dx & 0xFF,
    (-dy) & 0xFF,
    ord('W'), 2, # 2-byte wait value (us, LSB first)
    1000 # us wait
  )


pygame.init()
(width, height) = (320, 200)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption(u'Press PAUSE to quit')
pygame.display.flip()
pygame.event.set_grab(True)
pygame.mouse.set_visible(False)

event2ps2 = {
      pygame.K_1            : 0x16,
      pygame.K_2            : 0x1E,
      pygame.K_3            : 0x26,
      pygame.K_4            : 0x25,
      pygame.K_5            : 0x2E,
      pygame.K_6            : 0x36,
      pygame.K_7            : 0x3D,
      pygame.K_8            : 0x3E,
      pygame.K_9            : 0x46,
      pygame.K_0            : 0x45,
      pygame.K_MINUS        : 0x4E,
      pygame.K_EQUALS       : 0x55,
      pygame.K_BACKSPACE    : 0x66,
      pygame.K_TAB          : 0x0D,
      pygame.K_q            : 0x15,
      pygame.K_w            : 0x1D,
      pygame.K_e            : 0x24,
      pygame.K_r            : 0x2D,
      pygame.K_t            : 0x2C,
      pygame.K_y            : 0x35,
      pygame.K_u            : 0x3C,
      pygame.K_i            : 0x43,
      pygame.K_o            : 0x44,
      pygame.K_p            : 0x4D,
      pygame.K_LEFTBRACKET  : 0x54,
      pygame.K_RIGHTBRACKET : 0x5B,
      pygame.K_CAPSLOCK     : 0x58,
      pygame.K_a            : 0x1C,
      pygame.K_s            : 0x1B,
      pygame.K_d            : 0x23,
      pygame.K_f            : 0x2B,
      pygame.K_g            : 0x34,
      pygame.K_h            : 0x33,
      pygame.K_j            : 0x3B,
      pygame.K_k            : 0x42,
      pygame.K_l            : 0x4B,
      pygame.K_SEMICOLON    : 0x4C,
      pygame.K_QUOTE        : 0x52,
      pygame.K_RETURN       : 0x5A,
      pygame.K_LSHIFT       : 0x12,
      pygame.K_z            : 0x1A,
      pygame.K_x            : 0x22,
      pygame.K_c            : 0x21,
      pygame.K_v            : 0x2A,
      pygame.K_b            : 0x32,
      pygame.K_n            : 0x31,
      pygame.K_m            : 0x3A,
      pygame.K_COMMA        : 0x41,
      pygame.K_PERIOD       : 0x49,
      pygame.K_SLASH        : 0x4A,
      pygame.K_RSHIFT       : 0x59,
      pygame.K_LCTRL        : 0x14,
      pygame.K_LALT         : 0x11,
      pygame.K_SPACE        : 0x29,
      pygame.K_RALT         :(0x11 | 0x80),
      pygame.K_RCTRL        :(0x14 | 0x80),
      pygame.K_INSERT       :(0x70 | 0x80),
      pygame.K_DELETE       :(0x71 | 0x80),
      pygame.K_HOME         :(0x6C | 0x80),
      pygame.K_END          :(0x69 | 0x80),
      pygame.K_PAGEUP       :(0x7D | 0x80),
      pygame.K_PAGEDOWN     :(0x7A | 0x80),
      pygame.K_UP           :(0x75 | 0x80),
      pygame.K_DOWN         :(0x72 | 0x80),
      pygame.K_LEFT         :(0x6B | 0x80),
      pygame.K_RIGHT        :(0x74 | 0x80),
      pygame.K_NUMLOCK      :(0x77 | 0x80),
      pygame.K_KP7          : 0x6C,
      pygame.K_KP4          : 0x6B,
      pygame.K_KP1          : 0x69,
      pygame.K_KP_DIVIDE    :(0x4A | 0x80),
      pygame.K_KP8          : 0x75,
      pygame.K_KP5          : 0x73,
      pygame.K_KP2          : 0x72,
      pygame.K_KP0          : 0x70,
      pygame.K_KP_MULTIPLY  : 0x7C,
      pygame.K_KP9          : 0x7D,
      pygame.K_KP6          : 0x74,
      pygame.K_KP3          : 0x7A,
      pygame.K_KP_PLUS      : 0x79,
      pygame.K_KP_ENTER     :(0x5A | 0x80),
      pygame.K_ESCAPE       : 0x76,
      pygame.K_F1           : 0x05,
      pygame.K_F2           : 0x06,
      pygame.K_F3           : 0x04,
      pygame.K_F4           : 0x0C,
      pygame.K_F5           : 0x03,
      pygame.K_F6           : 0x0B,
      pygame.K_F7           : 0x83,
      pygame.K_F8           : 0x0A,
      pygame.K_F9           : 0x01,
      pygame.K_F10          : 0x09,
      pygame.K_F11          : 0x78,
      pygame.K_F12          : 0x07,
      pygame.K_SCROLLOCK    : 0x7E,
      pygame.K_BACKSLASH    : 0x5D,
}

while(True):
  event = pygame.event.wait()
  if event.type == pygame.KEYDOWN:
    if event.key == pygame.K_PAUSE:
      print("QUIT")
      break
    if event.key in event2ps2:
                      code = event2ps2[event.key]
                      if code & 0x80:
                        packet = bytearray([ord('K'), 2, 0xE0, code & 0x7F])
                      else:
                        packet = bytearray([ord('K'), 1, code & 0x7F])
                      ps2_tcp.sendall(packet)
    continue
  if event.type == pygame.KEYUP:
    if event.key in event2ps2:
                      code = event2ps2[event.key]
                      if code & 0x80:
                        packet = bytearray([ord('K'), 3, 0xE0, 0xF0, code & 0x7F])
                      else:
                        packet = bytearray([ord('K'), 2, 0xF0, code & 0x7F])
                      ps2_tcp.sendall(packet)
    continue
  wheel = 0
  if event.type == pygame.MOUSEBUTTONDOWN: # for wheel events
    if event.button == 4: # wheel UP
      wheel = -1
    if event.button == 5: # wheel DOWN
      wheel = 1
  (dx, dy) = pygame.mouse.get_rel()
  dz = wheel
  (btn_left, btn_middle, btn_right) = pygame.mouse.get_pressed()

  if mouse_wheel:
    # mouse with wheel
    report = mouse_wheel_report(dx, dy, dz, btn_left, btn_middle, btn_right)
    ps2_tcp.sendall(bytearray(report))
    #print("0x%08X: X=%4d, Y=%4d, Z=%2d, L=%2d, M=%2d, R=%2d" % (struct.unpack("I",report)[0], dx, dy, dz, btn_left, btn_middle, btn_right))
  else:
    # mouse without wheel
    report = mouse_nowheel_report(dx, dy, btn_left, btn_middle, btn_right)
    ps2_tcp.sendall(bytearray(report))
    #print(report)
    #print("X=%4d, Y=%4d, L=%2d, M=%2d, R=%2d" % (dx, dy, btn_left, btn_middle, btn_right))
