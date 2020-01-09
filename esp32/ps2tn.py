# AUTHOR=EMARD
# LICENSE=BSD

# telnet to ESP32 and type
# keystrokes should be converted to PS/2 signals

import socket
import network
import uos
import gc
from time import sleep_ms, localtime
from micropython import alloc_emergency_exception_buf
import ps2

# constant definitions
_SO_REGISTER_HANDLER = const(20)
_COMMAND_TIMEOUT = const(300)

# Global variables
ps2socket = None
client_list = []
verbose_l = 0
client_busy = False

# ASCII to PS/2 SET2 scancode conversion table
# from http://www.vetra.com/scancodes.html
asc2scan = {
'`'   : bytearray(b'\x0E\xF0\x0E'), '~'   : bytearray(b'\x12\x0E\xF0\x0E\xF0\x12'),
'1'   : bytearray(b'\x16\xF0\x16'), '!'   : bytearray(b'\x12\x16\xF0\x16\xF0\x12'),
'2'   : bytearray(b'\x1E\xF0\x1E'), '@'   : bytearray(b'\x12\x1E\xF0\x1E\xF0\x12'),
'3'   : bytearray(b'\x26\xF0\x26'), '#'   : bytearray(b'\x12\x26\xF0\x26\xF0\x12'),
'4'   : bytearray(b'\x25\xF0\x25'), '$'   : bytearray(b'\x12\x25\xF0\x25\xF0\x12'),
'5'   : bytearray(b'\x2E\xF0\x2E'), '%'   : bytearray(b'\x12\x2E\xF0\x2E\xF0\x12'),
'6'   : bytearray(b'\x36\xF0\x36'), '^'   : bytearray(b'\x12\x36\xF0\x36\xF0\x12'),
'7'   : bytearray(b'\x3D\xF0\x3D'), '&'   : bytearray(b'\x12\x3D\xF0\x3D\xF0\x12'),
'8'   : bytearray(b'\x3E\xF0\x3E'), '*'   : bytearray(b'\x12\x3E\xF0\x3E\xF0\x12'),
'9'   : bytearray(b'\x46\xF0\x46'), '('   : bytearray(b'\x12\x46\xF0\x46\xF0\x12'),
'0'   : bytearray(b'\x45\xF0\x45'), ')'   : bytearray(b'\x12\x45\xF0\x45\xF0\x12'),
'-'   : bytearray(b'\x4E\xF0\x4E'), '_'   : bytearray(b'\x12\x4E\xF0\x4E\xF0\x12'),
'='   : bytearray(b'\x55\xF0\x55'), '+'   : bytearray(b'\x12\x55\xF0\x55\xF0\x12'),
'\x7F': bytearray(b'\x66\xF0\x66'),# BACKSPACE
'\t'  : bytearray(b'\x0D\xF0\x0D'),# TAB
'q'   : bytearray(b'\x15\xF0\x15'),
'w'   : bytearray(b'\x1D\xF0\x1D'),
'e'   : bytearray(b'\x24\xF0\x24'),
'r'   : bytearray(b'\x2D\xF0\x2D'),
't'   : bytearray(b'\x2C\xF0\x2C'),
'y'   : bytearray(b'\x35\xF0\x35'),
'u'   : bytearray(b'\x3C\xF0\x3C'),
'i'   : bytearray(b'\x43\xF0\x43'),
'o'   : bytearray(b'\x44\xF0\x44'),
'p'   : bytearray(b'\x4D\xF0\x4D'),
'['   : bytearray(b'\x54\xF0\x54'), '{'   : bytearray(b'\x12\x54\xF0\x54\xF0\x12'),
']'   : bytearray(b'\x5B\xF0\x5B'), '}'   : bytearray(b'\x12\x5B\xF0\x5B\xF0\x12'),
#'CAPSLOCK'  : \x58,
'a'   : bytearray(b'\x1C\xF0\x1C'),
's'   : bytearray(b'\x1B\xF0\x1B'),
'd'   : bytearray(b'\x23\xF0\x23'),
'f'   : bytearray(b'\x2B\xF0\x2B'),
'g'   : bytearray(b'\x34\xF0\x34'),
'h'   : bytearray(b'\x33\xF0\x33'),
'j'   : bytearray(b'\x3B\xF0\x3B'),
'k'   : bytearray(b'\x42\xF0\x42'),
'l'   : bytearray(b'\x4B\xF0\x4B'),
';'   : bytearray(b'\x4C\xF0\x4C'), ':'   : bytearray(b'\x12\x4C\xF0\x4C\xF0\x12'),
'\''  : bytearray(b'\x52\xF0\x52'), '\"'  : bytearray(b'\x12\x52\xF0\x52\xF0\x12'),
'\r'  : bytearray(b'\x5A\xF0\x5A'),# ENTER
#'LEFTSHIFT' : \x12,
'z'   : bytearray(b'\x1A\xF0\x1A'),
'x'   : bytearray(b'\x22\xF0\x22'),
'c'   : bytearray(b'\x21\xF0\x21'),
'v'   : bytearray(b'\x2A\xF0\x2A'),
'b'   : bytearray(b'\x32\xF0\x32'),
'n'   : bytearray(b'\x31\xF0\x31'),
'm'   : bytearray(b'\x3A\xF0\x3A'),
','   : bytearray(b'\x41\xF0\x41'), '<'   : bytearray(b'\x12\x41\xF0\x41\xF0\x12'),
'.'   : bytearray(b'\x49\xF0\x49'), '>'   : bytearray(b'\x12\x49\xF0\x49\xF0\x12'),
'/'   : bytearray(b'\x4A\xF0\x4A'), '?'   : bytearray(b'\x12\x4A\xF0\x4A\xF0\x12'),
#'RIGHTSHIFT': \x59,
#'LEFTCTRL'  : \x14,
#'LEFTALT'   : \x11,
' '         : bytearray(b'\x29\xF0\x29'),
#'RIGHTALT'  :(\x11 | \x80),
#'RIGHTCTRL' :(\x14 | \x80),
#'INSERT'    :(\x70 | \x80),
#'DELETE'    :(\x71 | \x80),
#'HOME'      :(\x6C | \x80),
#'END'       :(\x69 | \x80),
#'PAGEUP'    :(\x7D | \x80),
#'PAGEDOWN'  :(\x7A | \x80),
#'UP'        :(\x75 | \x80),
#'DOWN'      :(\x72 | \x80),
#'LEFT'      :(\x6B | \x80),
#'RIGHT'     :(\x74 | \x80),
#'NUMLOCK'   :(\x77 | \x80),
#'KP7'       : \x6C,
#'KP4'       : \x6B,
#'KP1'       : \x69,
#'KPSLASH'   :(\x4A | \x80),
#'KP8'       : \x75,
#'KP5'       : \x73,
#'KP2'       : \x72,
#'KP0'       : \x70,
#'KPASTERISK': \x7C,
#'KP9'       : \x7D,
#'KP6'       : \x74,
#'KP3'       : \x7A,
#'KPPLUS'    : \x79,
#'KPENTER'   :(\x5A | \x80),
'\x1B': bytearray(b'\x76\xF0\x76'),# ESC
#'F1'        : \x05,
#'F2'        : \x06,
#'F3'        : \x04,
#'F4'        : \x0C,
#'F5'        : \x03,
#'F6'        : \x0B,
#'F7'        : \x83,
#'F8'        : \x0A,
#'F9'        : \x01,
#'F10'       : \x09,
#'F11'       : \x78,
#'F12'       : \x07,
#'SCROLLLOCK': \x7E,
'\\'  : bytearray(b'\x5D\xF0\x5D'), '|'   : bytearray(b'\x12\x5D\xF0\x5D\xF0\x12'),
'\x01': bytearray(b'\x14\x1C\xF0\x1C\xF0\x14'),# Ctrl-A
'\x02': bytearray(b'\x14\x32\xF0\x32\xF0\x14'),# Ctrl-B
'\x03': bytearray(b'\x14\x21\xF0\x21\xF0\x14'),# Ctrl-C
'\x04': bytearray(b'\x14\x23\xF0\x23\xF0\x14'),# Ctrl-D
'\x05': bytearray(b'\x14\x24\xF0\x24\xF0\x14'),# Ctrl-E
'\x06': bytearray(b'\x14\x2B\xF0\x2B\xF0\x14'),# Ctrl-F
'\x07': bytearray(b'\x14\x34\xF0\x34\xF0\x14'),# Ctrl-G
'\x08': bytearray(b'\x14\x33\xF0\x33\xF0\x14'),# Ctrl-H
'\x09': bytearray(b'\x14\x43\xF0\x43\xF0\x14'),# Ctrl-I
'\x0A': bytearray(b'\x14\x3B\xF0\x3B\xF0\x14'),# Ctrl-J
'\x0B': bytearray(b'\x14\x42\xF0\x42\xF0\x14'),# Ctrl-K
'\x0C': bytearray(b'\x14\x4B\xF0\x4B\xF0\x14'),# Ctrl-L
#'\x0D'      : bytearray(b'\x14\x3A\xF0\x3A\xF0\x14'),# Ctrl-M ENTER
'\x0E': bytearray(b'\x14\x31\xF0\x31\xF0\x14'),# Ctrl-N
'\x0F': bytearray(b'\x14\x44\xF0\x44\xF0\x14'),# Ctrl-O
'\x10': bytearray(b'\x14\x4D\xF0\x4D\xF0\x14'),# Ctrl-P
'\x11': bytearray(b'\x14\x15\xF0\x15\xF0\x14'),# Ctrl-Q
'\x12': bytearray(b'\x14\x2D\xF0\x2D\xF0\x14'),# Ctrl-R
'\x13': bytearray(b'\x14\x1B\xF0\x1B\xF0\x14'),# Ctrl-S
'\x14': bytearray(b'\x14\x2C\xF0\x2C\xF0\x14'),# Ctrl-T
'\x15': bytearray(b'\x14\x3C\xF0\x3C\xF0\x14'),# Ctrl-U
'\x16': bytearray(b'\x14\x2A\xF0\x2A\xF0\x14'),# Ctrl-V
'\x17': bytearray(b'\x14\x1D\xF0\x1D\xF0\x14'),# Ctrl-W
'\x18': bytearray(b'\x14\x22\xF0\x22\xF0\x14'),# Ctrl-X
'\x19': bytearray(b'\x14\x35\xF0\x35\xF0\x14'),# Ctrl-Y
'\x1A': bytearray(b'\x14\x1A\xF0\x1A\xF0\x14'),# Ctrl-Z
'A'   : bytearray(b'\x12\x1C\xF0\x1C\xF0\x12'),
'B'   : bytearray(b'\x12\x32\xF0\x32\xF0\x12'),
'C'   : bytearray(b'\x12\x21\xF0\x21\xF0\x12'),
'D'   : bytearray(b'\x12\x23\xF0\x23\xF0\x12'),
'E'   : bytearray(b'\x12\x24\xF0\x24\xF0\x12'),
'F'   : bytearray(b'\x12\x2B\xF0\x2B\xF0\x12'),
'G'   : bytearray(b'\x12\x34\xF0\x34\xF0\x12'),
'H'   : bytearray(b'\x12\x33\xF0\x33\xF0\x12'),
'I'   : bytearray(b'\x12\x43\xF0\x43\xF0\x12'),
'J'   : bytearray(b'\x12\x3B\xF0\x3B\xF0\x12'),
'K'   : bytearray(b'\x12\x42\xF0\x42\xF0\x12'),
'L'   : bytearray(b'\x12\x4B\xF0\x4B\xF0\x12'),
'M'   : bytearray(b'\x12\x3A\xF0\x3A\xF0\x12'),
'N'   : bytearray(b'\x12\x31\xF0\x31\xF0\x12'),
'O'   : bytearray(b'\x12\x44\xF0\x44\xF0\x12'),
'P'   : bytearray(b'\x12\x4D\xF0\x4D\xF0\x12'),
'Q'   : bytearray(b'\x12\x15\xF0\x15\xF0\x12'),
'R'   : bytearray(b'\x12\x2D\xF0\x2D\xF0\x12'),
'S'   : bytearray(b'\x12\x1B\xF0\x1B\xF0\x12'),
'T'   : bytearray(b'\x12\x2C\xF0\x2C\xF0\x12'),
'U'   : bytearray(b'\x12\x3C\xF0\x3C\xF0\x12'),
'V'   : bytearray(b'\x12\x2A\xF0\x2A\xF0\x12'),
'W'   : bytearray(b'\x12\x1D\xF0\x1D\xF0\x12'),
'X'   : bytearray(b'\x12\x22\xF0\x22\xF0\x12'),
'Y'   : bytearray(b'\x12\x35\xF0\x35\xF0\x12'),
'Z'   : bytearray(b'\x12\x1A\xF0\x1A\xF0\x12'),
}


class PS2_client:

    def __init__(self, ps2socket):
        self.command_client, self.remote_addr = ps2socket.accept()
        self.command_client.setblocking(False)
        self.command_client.sendall(bytes([255, 252, 34])) # dont allow line mode
        self.command_client.sendall(bytes([255, 251,  1])) # turn off local echo
        self.command_client.recv(32) # drain junk
        sleep_ms(20)
        self.command_client.recv(32) # drain junk
        self.remote_addr = self.remote_addr[0]
        #self.command_client.settimeout(_COMMAND_TIMEOUT)
        log_msg(1, "PS2 Command connection from:", self.remote_addr)
        self.command_client.setsockopt(socket.SOL_SOCKET,
                                       _SO_REGISTER_HANDLER,
                                       self.exec_ps2_command)
        self.act_data_addr = self.remote_addr
        self.active = True

    def exec_ps2_command(self, cl):
        global client_busy
        global my_ip_addr
        global ps2port

        #try:
        if True:
            gc.collect()

            data = cl.recv(32)

            if len(data) <= 0:
                # No data, close
                log_msg(1, "*** No data, assume QUIT")
                close_client(cl)
                return

            if client_busy:  # check if another client is busy
                #cl.sendall("400 Device busy.\r\n")  # tell so the remote client
                return  # and quit

            client_busy = True  # now it's my turn
            sdata = str(data, "utf-8")
            for cdata in sdata:
              if cdata in asc2scan:
                #code = asc2scan[cdata]
                ps2port.write(asc2scan[cdata])
                #for scancode in code:
                #  ps2port.write(bytearray([scancode]))
                #  sleep_ms(20)
            cl.sendall(data)
            client_busy = False
            return

        #except Exception as err:
        #    log_msg(1, "Exception in exec_ps2_command: {}".format(err))


def log_msg(level, *args):
    global verbose_l
    if verbose_l >= level:
        print(*args)


# close client and remove it from the list
def close_client(cl):
    cl.setsockopt(socket.SOL_SOCKET, _SO_REGISTER_HANDLER, None)
    cl.close()
    for i, client in enumerate(client_list):
        if client.command_client == cl:
            del client_list[i]
            break


def accept_ps2_connect(ps2socket):
    # Accept new calls for the server
    try:
        client_list.append(PS2_client(ps2socket))
    except:
        log_msg(1, "Attempt to connect failed")
        # try at least to reject
        try:
            temp_client, temp_addr = ps2socket.accept()
            temp_client.close()
        except:
            pass


def stop():
    global ps2socket
    global client_list
    global client_busy
    global ps2port

    for client in client_list:
        client.command_client.setsockopt(socket.SOL_SOCKET,
                                         _SO_REGISTER_HANDLER, None)
        client.command_client.close()
    del client_list
    client_list = []
    client_busy = False
    if ps2socket is not None:
        ps2socket.setsockopt(socket.SOL_SOCKET, _SO_REGISTER_HANDLER, None)
        ps2socket.close()
    del ps2port


# start listening for ftp connections on telnet default port 23
def start(port=23, verbose=0, splash=True):
    global ps2socket
    global verbose_l
    global client_list
    global client_busy
    global ps2port
    
    ps2port=ps2.ps2(clk=26,data=25,qbit_us=16,byte_us=150,f0_us=50000)

    alloc_emergency_exception_buf(100)
    verbose_l = verbose
    client_list = []
    client_busy = False

    ps2socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ps2socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ps2socket.bind(('0.0.0.0', port))
    ps2socket.listen(0)
    ps2socket.setsockopt(socket.SOL_SOCKET,
                         _SO_REGISTER_HANDLER, accept_ps2_connect)


def restart(port=23, verbose=0, splash=True):
    stop()
    sleep_ms(200)
    start(port, verbose, splash)


start(splash=True)
