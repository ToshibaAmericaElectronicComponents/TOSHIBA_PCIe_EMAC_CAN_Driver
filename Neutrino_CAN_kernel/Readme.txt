# TOSHIBA_PCIe_CAN_Driver
Release Date: 12 August 2016
Release Version: V_00-07


===============================================================================
TOSHIBA PCIe CAN driver patch is based on "Fedora 20, kernel-3.19". Driver is 
mainly meant to showcase the demo application. Neutrino PCIe CAN driver is 
responsible to read/write the can frames from/to Neutrino can fd controller.

Apply driver as patch from kernel directory and follow below steps: 
	1) NVME needs to be disabled from the ".config" file.
		- CONFIG_BLK_DEV_NVME is not set

	2) CAN device support needs to be enabled as below.
		-*- Networking support  --->
		<*> CAN bus subsystem support  --->
			<*> Raw CAN Protocol (raw access with CAN-ID filtering)
			<*> Broadcast Manager CAN Protocol (with content filtering)
			<*> CAN Gateway/Router (with netlink configuration)
				CAN Device Drivers  --->
				<*> Platform CAN drivers with Netlink support
				[*]    CAN bit-timing calculation
				< >    Bosch C_CAN/D_CAN devices  ----
					   M_CAN device support (Toshiba Neutrino PCIe M_CAN devices)  --->
								( ) None
								( ) Bosch M_CAN devices
								(X) Toshiba Neutrino PCIe M_CAN devices

	3) Recompile the kernel.

	4) Copy "<Kernel Directory>/usr/include/linux/can.h"  
	   to   "<Kernel Directory>/usr/include/linux/"

	-------- Additional package for PCIe CAN demo ----------
	5) Download the IP Route Utility "iproute2-4.3.0" from
		https://www.kernel.org/pub/linux/utils/net/iproute2/
	6) #cd iproute2-4.3.0/
	7) #make && make install
	8) Download the CAN Utility from https://github.com/linux-can/can-utils
	9) #cd can-utils-master/
	10) #make && make install


Compilation & Run: Need to be root user to execute the following steps.
===============================================================================
1. Execute the following script in terminal
   #sh init_ntn_can.sh <loopback mode on/off>
   ex: #sh init_ntn_can.sh on
       #sh init_ntn_can.sh off

   Note: By enabling this loopback, transmitted can frames are received back 
         within the same can interface.

Known Limitations:
===============================================================================
- PCIe CAN driver is a builtin moudle.



