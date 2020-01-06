import network
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("accesspoint", "password")
#from os import mount
#from machine import SDCard
#mount(SDCard(slot=3),"/sd")
import ecp5
ecp5.prog("apple2.bit.gz")
import disk2
import ps2server
