Release Date: 12 August 2016
Release Version: V_00-07
===============================================================================
Neutrino PCIe driver is the PCIe driver for Neutrino.  Driver is mainly 
served as a parent driver for Neutrino Ethernet Driver and Neutrino CAN
driver. 

===============================================================================
TOSHIBA EMAC/Ethernet driver is based on "Fedora 20, kernel-3.19". Kernel needs to
be recompiled with following options set into the ".config" file.
  - CONFIG_BLK_DEV_NVME is not set

	
Compilation & Run: Need to be root user to execute the follwoing steps.
===============================================================================
1.  Execute following commands:
    #make clean
    #make
2.  Load the driver
	#insmod neutrino_pcie.ko
3.  Remove the driver
	#rmmod neutrino_pcie


Changes from the last release
===============================================================================
1. This is the initial release. It is based on PCIe Ethernet and PCIe CAN driver
   release.	

