obj-m	:=	ib.o
ib-objs	:=	cbp_base.o client_block.o master_block.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
