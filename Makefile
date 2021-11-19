
obj-m := mcp391x.o

PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) INSTALL_MOD_DIR=kernel/npt modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
