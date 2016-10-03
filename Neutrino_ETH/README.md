# TOSHIBA_ETH_Driver
Release Date: 12 August 2016
Release Version: V_00-07


===============================================================================
TOSHIBA EMAC/Ethernet driver is based on "Fedora 20, kernel-3.19". Kernel needs to   
be recompiled with following options set into the ".config" file.
	- CONFIG_HAVE_GENERIC_DMA_COHERENT=y 
	- CONFIG_BLK_DEV_NVME is not set

Note:
	MAC ID can be to be configured in "config.ini" file. The current MAC ID is
	for test purpose only. Default MAC ID can be changed in "dev_addr" array
    in file "DWC_ETH_QOS_platform.c".
	
Compilation & Run: Need to be root user to execute the following steps.
===============================================================================
1.  Execute following commands:
    #make clean
    #make						
    
2.	Load PTP module
	#modprobe ptp
3.  Load the driver
	#insmod DWC_ETH_QOS.ko
4.  Remove the driver
	#rmmod DWC_ETH_QOS

	
Supported Features:
===============================================================================
1. Legacy and AVB traffic
2. Descriptors/Data buffer can be placed in SRAM/Host memory. Default is 
   Descriptors and Data buffer are placed in Host memory.
3. MAC ID is configurable.

