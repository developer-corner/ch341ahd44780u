all: driver library tool

driver:
	$(MAKE) -C driver
	
library:
	$(MAKE) -C library
	
tool:
	$(MAKE) -C tool

clean:
	$(MAKE) -C driver clean
	$(MAKE) -C library clean
	$(MAKE) -C tool clean
	
install: driver library tool
	@(id | grep 'uid=0(root)' || (echo "You have to be root."; /bin/false))
	(rmmod -f ch341ahd44780u.ko >/dev/null 2>&1 || /bin/true)
	$(MAKE) -C driver install
	install -v -d /usr/bin
	install -v -d /usr/lib
	install -v -d /etc/udev/rules.d
	install -v -m 0775 ./library/libusbhd44780u.so /usr/lib
	install -v -m 0775 ./tool/usbhd44780u /usr/bin
	install -v -m 0664 ./udev/90-ch341a-hd44780u.rules /etc/udev/rules.d
	udevadm control --reload
	@(rmmod -f ch34x_pis.ko >/dev/null 2>&1 || /bin/true)
	@(rmmod -f spi-ch341-usb.ko >/dev/null 2>&1 || /bin/true)
	modprobe ch341ahd44780u
	
.PHONY: all driver library tool clean install
