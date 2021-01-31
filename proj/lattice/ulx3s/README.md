# ULX3S APPLE ][

It boots and even supports read-only floopy images
in NIB format.

# compiling

Edit "Makefile" and select FPGA size 25/45/85

    make clean; make prog

# usage

Raw floppy images can be written to SD card with ESP32 uftpd.py

    ftp> put disk2.nib sd@0

Or ESP32 osd.py server and the image file file can be put
ESP32 FLASH filesystem:

    ftp> put osd.py
    ftp> put disk2.nib

and ESP32 DISK server started:

    screen /dev/ttyUSB0 115200
    >>> import osd
    import osd

press 4 direction buttons together (BTN3-6) or BTN 1
to open OSD menu, select file disk2.nib by pressing right direction (BTN6)
and it will "insert" disk2.nib floppy into emulated drive.

Some apple2 hints:

    ]CATALOG
    ]PR#6
    ]CALL -151
    *6<Ctrl-P><ENTER>
    *9DBFG
    ]

Most floppy disk images from [planet emulation](https://www.planetemu.net/machine/apple-ii)
are in DSK format.
[online disk image converter at kboohk](http://kboohk.com/dsk2woz/)
can convert them to NIB format.
