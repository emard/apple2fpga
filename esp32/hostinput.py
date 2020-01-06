#!/usr/bin/env python3

# AUTHOR=EMARD
# LICENSE=GPL

# Reads linux mouse input device (evdev).
# Converts mouse events to ps2 serial commands.
# Currently it can only move mouse pointer.
# Mouse clicks are received but not supported yet.

# lsinput
# /dev/input/event7
#    name    : "Logitech USB-PS/2 Optical Mouse"
# chmod a+rw /dev/input/event7

import evdev
import serial
import struct
import socket

# fix packet with header, escapes, trailer
def escape(p):
  retval = struct.pack("BB", 0, 0)
  for char in p:
    if char == 0 or char == 0x5C:
      retval += struct.pack("B", 0x5C)
    retval += struct.pack("B", char)
  retval += struct.pack("B", 0)
  return retval
  
def pointer(x,y):
  rgtr = (y & 0xFFF)*(2**12) + (x & 0xFFF)
  return struct.pack(">BBI", 0x15, 3, rgtr)

def mouse_report(dx,dy,dz,btn_left,btn_middle,btn_right):
  return struct.pack(">BBBBBB", 0x0F, 3, (-dz) & 0xFF, (-dy) & 0xFF, dx & 0xFF, (btn_left & 1) + (btn_right & 1)*(2**1) + (btn_middle & 1)*(2**2))

def print_packet(x):
  for c in x:
    print("%02X" % (c), end='');
  print("")

if __name__ == '__main__':
    # this string will search for mouse in list of evdev inputs
    # Usually this should match a part of USB mouse device name
    keyboard_input_name = "TypeM"
    # ps2 network host or ip
    tcp_host = "192.168.48.128"
    tcp_port = 3252 # use UDP, not serial port

    # from http://www.vetra.com/scancodes.html
    keymap_ps2_scan2 = {
      'KEY_GRAVE':0x0E,
      'KEY_1':0x16,
      'KEY_2':0x1E,
      'KEY_3':0x26,
      'KEY_4':0x25,
      'KEY_5':0x2E,
      'KEY_6':0x36,
      'KEY_7':0x3D,
      'KEY_8':0x3E,
      'KEY_9':0x46,
      'KEY_0':0x45,
      'KEY_MINUS':0x4E,
      'KEY_EQUAL':0x55,
      'KEY_BACKSPACE':0x66,
      'KEY_TAB':0x0D,
      'KEY_Q':0x15,
      'KEY_W':0x1D,
      'KEY_E':0x24,
      'KEY_R':0x2D,
      'KEY_T':0x2C,
      'KEY_Y':0x35,
      'KEY_U':0x3C,
      'KEY_I':0x43,
      'KEY_O':0x44,
      'KEY_P':0x4D,
      'KEY_LEFTBRACE':0x54,
      'KEY_RIGHTBRACE':0x5B,
      'KEY_CAPSLOCK':0x58,
      'KEY_A':0x1C,
      'KEY_S':0x1B,
      'KEY_D':0x23,
      'KEY_F':0x2B,
      'KEY_G':0x34,
      'KEY_H':0x33,
      'KEY_J':0x3B,
      'KEY_K':0x42,
      'KEY_L':0x4B,
      'KEY_SEMICOLON':0x4C,
      'KEY_APOSTROPHE':0x52,
      'KEY_ENTER':0x5A,
      'KEY_LEFTSHIFT':0x12,
      'KEY_Z':0x1A,
      'KEY_X':0x22,
      'KEY_C':0x21,
      'KEY_V':0x2A,
      'KEY_B':0x32,
      'KEY_N':0x31,
      'KEY_M':0x3A,
      'KEY_COMMA':0x41,
      'KEY_DOT':0x49,
      'KEY_SLASH':0x4A,
      'KEY_RIGHTSHIFT':0x59,
      'KEY_LEFTCTRL':0x14,
      'KEY_LEFTALT':0x11,
      'KEY_SPACE':0x29,
      'KEY_RIGHTALT':0x11, # TODO E0 11, E0 F0 11
      'KEY_RIGHTCTRL':0x14, # TODO E0 14, E0 F0 14
      'KEY_UP':0x75, # TODO E0 75/E0 F0 75
      'KEY_DOWN':0x72, # TODO E0 72/E0 F0 72
      'KEY_LEFT':0x6B, # TODO E0 6B/E0 F0 6B
      'KEY_RIGHT':0x74, # TODO E0 74/E0 F0 74
      'KEY_NUMLOCK':0x77,
      'KEY_KP7':0x6C,
      'KEY_KP4':0x6B,
      'KEY_KP1':0x69,
      'KEY_KPSLASH':0x4A, # TODO E0 4A/E0 F0 4A
      'KEY_KP8':0x75,
      'KEY_KP5':0x73,
      'KEY_KP2':0x72,
      'KEY_KP0':0x70,
      'KEY_KPASTERISK':0x7C,
      'KEY_KP9':0x7D,
      'KEY_KP6':0x74,
      'KEY_KP3':0x7A,
      'KEY_KPPLUS':0x79,
      'KEY_KPENTER':0x5A, # TODO E0 5A/E0 F0 5A
      'KEY_ESC':0x76,
      'KEY_F1':0x05,
      'KEY_F2':0x06,
      'KEY_F3':0x04,
      'KEY_F4':0x0C,
      'KEY_F5':0x03,
      'KEY_F6':0x0B,
      'KEY_F7':0x83,
      'KEY_F8':0x0A,
      'KEY_F9':0x01,
      'KEY_F10':0x09,
      'KEY_F11':0x78,
      'KEY_F12':0x07,
      'KEY_SCROLLLOCK':0x7E,
      'KEY_BACKSLASH':0x5D,
    }

    # convert keys to input events evdev.ecodes.ecodes[key]
    event2ps2 = { }
    for key in keymap_ps2_scan2:
      event2ps2[evdev.ecodes.ecodes[key]] = keymap_ps2_scan2[key]

    X = 0
    Y = 0
    Z = 0
    DX = 0
    DY = 0
    DZ = 0
    BTN_LEFT = 0
    BTN_RIGHT = 0
    BTN_MIDDLE = 0
    TOUCH = 0

    DEVICE = None

    DEVICES = [evdev.InputDevice(fn) for fn in evdev.list_devices()]

    for d in DEVICES:
        if keyboard_input_name in d.name:
            DEVICE = d
            print('Found %s at %s...' % (d.name, d.path))
            break

    if DEVICE:
        ps2_tcp=socket.create_connection((tcp_host, tcp_port))
        print("Sending keyboard events to %s:%s" % (tcp_host,tcp_port))
        ps2_tcp.sendall(bytearray([0xAA, 0xFA])) # keyboard sends 0xAA after being plugged
        for event in DEVICE.read_loop():
            if event.type == evdev.ecodes.EV_REL and False: # TODO support mouse properly
                if event.code == evdev.ecodes.REL_X:
                    DX = event.value
                    X += DX
                if event.code == evdev.ecodes.REL_Y:
                    DY = event.value
                    Y += DY
                if event.code == evdev.ecodes.REL_WHEEL:
                    DZ = event.value
                    Z += DZ
                #print('X=%d Y=%d Z=%d' % (X, Y, Z))
                #packet = pointer(X,Y)
                packet = mouse_report(DX,DY,DZ,BTN_LEFT,BTN_MIDDLE,BTN_RIGHT)
                DZ = 0
                DX = 0
                DY = 0
                ps2_tcp.sendall(packet)

            if event.type == evdev.ecodes.EV_KEY:
                if event.code in event2ps2:
                    packet = None
                    if event.value == 1: # key press
                      packet = bytearray([event2ps2[event.code]])
                    #if event.value == 2: # key autorepeat
                    #  packet = bytearray([event2ps2[event.code]])
                    if event.value == 0: # key release
                      packet = bytearray([0xF0,event2ps2[event.code]])
                    if packet:
                      ps2_tcp.sendall(packet)
