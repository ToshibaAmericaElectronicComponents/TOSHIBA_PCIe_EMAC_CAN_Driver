export PTPSUPPORT CONFIG_PTPSUPPORT_OBJ DWC_ETH_QOS_CONFIG_PTP

#default values
PTPSUPPORT=y	#ptp is enabled

ifeq ($(RELEASE_PACKAGE),1)
EXTRA_CFLAGS+=-DRELEASE_PACKAGE
endif
LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
CCFLAGS += -m64
EXTRA_CFLAGS+=-DSYSTEM_IS_64
else
CCFLAGS += -m32
endif

ifeq "$(PTPSUPPORT)" "y"
CONFIG_PTPSUPPORT_OBJ=y
DWC_ETH_QOS_CONFIG_PTP=-DPTPSUPPORT
EXTRA_CFLAGS+=-DCONFIG_PTPSUPPORT_OBJ
else
CONFIG_PTPSUPPORT_OBJ=y
endif

obj-m := DWC_ETH_QOS.o

DWC_ETH_QOS-y += DWC_ETH_QOS_dev.o \
			DWC_ETH_QOS_drv.o \
			DWC_ETH_QOS_desc.o \
			DWC_ETH_QOS_ethtool.o \
			DWC_ETH_QOS_platform.o \
			DWC_ETH_QOS_mdio.o \
			DWC_ETH_QOS_eee.o

DWC_ETH_QOS-$(CONFIG_PTPSUPPORT_OBJ) += DWC_ETH_QOS_ptp.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	rm -f *.o
	rm -f .DWC*.cmd
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean


