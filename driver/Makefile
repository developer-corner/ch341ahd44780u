MODULE_NAME=ch341ahd44780u
MODULE_VERSION=0.1

DKMS       := $(shell which dkms)
PWD        := $(shell pwd) 
KVERSION   := $(shell uname -r)
KERNEL_DIR  = /lib/modules/$(KVERSION)/build
MODULE_DIR  = /lib/modules/$(KVERSION)

ifneq ($(DKMS),)
MODULE_INSTALLED := $(shell dkms status $(MODULE_NAME))
else
MODULE_INSTALLED =
endif

obj-m       := $(MODULE_NAME).o   

$(MODULE_NAME).ko: $(MODULE_NAME).c
	make -C $(KERNEL_DIR) M=$(PWD) modules

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean

ifeq ($(DKMS),)  # if DKMS is not installed

install: $(MODULE_NAME).ko
	cp $(MODULE_NAME).ko $(MODULE_DIR)/kernel/drivers/extra
	depmod
	
uninstall:
	rm -f $(MODULE_DIR)/kernel/drivers/extra/$(MODULE_NAME).ko
	depmod

else  # if DKMS is installed

install: $(MODULE_NAME).ko
ifneq ($(MODULE_INSTALLED),)
	@echo Module $(MODULE_NAME) is installed ... uninstall it first
	@make uninstall
endif
	@dkms install .
	
uninstall:
ifneq ($(MODULE_INSTALLED),)
	dkms remove -m $(MODULE_NAME) -v $(MODULE_VERSION) --all
	rm -rf /usr/src/$(MODULE_NAME)-$(MODULE_VERSION)
endif

endif  # ifeq ($(DKMS),)

indent:
	indent -nbad -bap -nbc -bbo -hnl -br -brs -c33 -cd33 -ncdb -ce -ci4 -cli0 -d0 -di1 -nfc1 -i8 -ip0 -l80 -lp -npcs -nprs -npsl -sai -saf -saw -ncs -nsc -sob -nfca -cp33 -ss -ts8 -il1 $(MODULE_NAME).c
