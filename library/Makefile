all: libusbhd44780u.so

libusbhd44780u.so:
	$(CC) -DNDEBUG -D_LINUX -I./inc -O3 -fomit-frame-pointer -pedantic -Wall -Wextra -Werror -c -fmessage-length=0 -fPIC -o usbhd44780u.o ./src/usbhd44780u.c
	$(CC) -fvisibility=hidden -Wl,--version-script=./def/libusbhd44780u.version -shared -o libusbhd44780u.so usbhd44780u.o
	
clean:
	rm -f libusbhd44780u.so usbhd44780u.o

.PHONY: all clean
