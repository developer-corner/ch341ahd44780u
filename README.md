# ch341ahd44780u - Linux kernel driver, library, and tool for HD44780U/CH341A
The LCD display HITACHI 44780U (4 rows, 20 columns) is available as a hardware unit with the CH341A USB bridge controller, e.g. [buy a unit here](https://www.electronic-software-shop.com/lng/en/hardware/displays-usb/?language=en "www.electronic-software-shop.com").  
Each character is displayed as 5x8 dots matrix (5x10 is supported by the display, too).  
It comes with very good support for MS Windows but lacks support for Linux. This project closes the gap:
1. Linux kernel driver
2. userland library (shared object)
3. command line tool
4. full source code for 1., 2., and 3.
5. documentation (PDF document)

## For the impatient people
1. Install the source code;
2. Install the buildchain and DKMS - for Debian-like systems: **apt install build-essential dkms linux-headers-$(uname -r)**;
3. Enter the source code folder;
4. Execute **make**;
5. Execute **sudo make install** (or become 'root' before 'make install');
6. Attach the combined CH341A/HD44780U unit to the USB of your host;
7. Execute **usbhd44780u --dev=0 --test** (this executes the interactive testsuite).

## What this software provides
1. Linux kernel module (device driver);
2. userland library with software scrolling support;
3. command line tool for programming the LCD display.
4. Up to eight user-defined dot matrices can be programmed as user-defined characters.

## Documentation
Please read the accompanying PDF document and consult the source code. The kernel driver is documented as well as the userland library (Doxygen-style comments).

## Known limitations
* The current kernel driver was tested on Debian Buster (10), kernel 4.19.x and on Debian Bullseye (11), kernel 5.10.x. It does **not** read the BF (Busy Flag) of the HD44780U LCD display but waits 2 milliseconds after each write operation instead.
* A kernel interface could be added so that the display may be used for kernel debugging purposes.
* Only LTR (Left-To-Right) is currently supported, not RTL (Right-To-Left). Could be added, too.


