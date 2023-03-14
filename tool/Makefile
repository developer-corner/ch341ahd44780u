all: usbhd44780u

usbhd44780u:
	$(CC) -DNDEBUG -D_LINUX -I../library/inc -O3 -fomit-frame-pointer -pedantic -Wall -Wextra -Werror -c -fmessage-length=0 -fPIC -o main.o main.c
	$(CC) -L../library -o "usbhd44780u" main.o -lusbhd44780u
	
clean:
	rm -f main.o usbhd44780u

.PHONY: all clean
