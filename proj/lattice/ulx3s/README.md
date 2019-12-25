# ULX3S APPLE ][

It boots and even supports read-only floopy images
in NIB format.

Raw floppy images can be written to SD card with ESP32 uftpd.py

    ftp> put disk2.nib sd@0

Or ESP32 disk2.py server and the image file file can be put
ESP32 FLASH filesystem:

    ftp> put disk2.py
    ftp> put disk2.nib

and ESP32 DISK server started:

    screen /dev/ttyUSB0 115200
    >>> import disk2
    import disk2
    DISK ][ disk2.nib

Some apple2 hints:

    ]CATALOG
    ]PR#6
    ]CALL -151
    *6<Ctrl-P><ENTER>
    *9DBFG
    ]
