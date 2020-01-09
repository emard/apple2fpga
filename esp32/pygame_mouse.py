#!/usr/bin/env python3

# AUTHOR=EMARD
# LICENSE=GPL

# use ps2recv.py on ESP32
# edit ps2recv.py and "mouse_wheel" (below)
# False: send reports as no-wheel mouse (old/legacy PS/2)
# True:  send reports as wheel mouse (modern PS/2)

import pygame
import struct
import socket

def mouse_wheel_report(dx,dy,dz,btn_left,btn_middle,btn_right):
  return struct.pack(">BBBB", (btn_left & 1) + ((btn_right & 1)<<1) + ((btn_middle & 1)<<2), dx & 0xFF, (-dy) & 0xFF, (-dz) & 0x0F)

def mouse_nowheel_report(dx,dy,btn_left,btn_middle,btn_right):
  return struct.pack(">BBB", (btn_left & 1) + ((btn_right & 1)<<1) + ((btn_middle & 1)<<2), dx & 0xFF, (-dy) & 0xFF)

tcp_host = "192.168.48.128"
tcp_port = 3252
mouse_wheel = True

ps2_tcp=socket.create_connection((tcp_host, tcp_port))
print("Sending mouse events to %s:%s" % (tcp_host,tcp_port))
ps2_tcp.sendall(bytearray([0xAA, 0x00, 0xFA]))
# mouse sends 0xAA 0x00 after being plugged
# 0xFA is ACK what mouse sends after being configured

pygame.init()
(width, height) = (320, 200)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption(u'Mouse events - any key to quit')
pygame.display.flip()
pygame.event.set_grab(True)
pygame.mouse.set_visible(False)



while(True):
  event = pygame.event.wait()
  if event.type == pygame.KEYDOWN:
    print("QUIT")
    break
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
    print("0x%08X: X=%4d, Y=%4d, Z=%2d, L=%2d, M=%2d, R=%2d" % (struct.unpack("I",report)[0], dx, dy, dz, btn_left, btn_middle, btn_right))
  else:
    # mouse without wheel
    report = mouse_nowheel_report(dx, dy, btn_left, btn_middle, btn_right)
    ps2_tcp.sendall(bytearray(report))
    print("X=%4d, Y=%4d, L=%2d, M=%2d, R=%2d" % (dx, dy, btn_left, btn_middle, btn_right))
