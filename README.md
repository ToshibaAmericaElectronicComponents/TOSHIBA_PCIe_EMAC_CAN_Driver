# TOSHIBA Neutrino Driver
Release Date: 12 August 2016
Release Version: V_00-07

Note: This release is not fully tested yet.

===============================================================================
TOSHIBA EMAC/Ethernet & CAN drivers are based on "Fedora 20, kernel-3.19".
	
1. Neutrino_PCIe

	This is the driver which will perform PCIe probe based on Neutrino PCIE device ID.
	- It will register a PCIe driver.
	- Enable MSI, setup PCIe DMA mask
	- PCI request regions
	- PCIe BARs setup (iomap)
	- Global Neutrino registers (clock, reset) setup
	- Add two platform devices: NTN_ETH and NTN_CAN
	
	Additional Details in Neutrino_PCIe/Readme

2. Neutrino_ETH & Neutrino_CAN_kernel

	These are the Platform Drivers.
	- NTN Ethernet Platform Driver (loadable module) will handle the Ethernet Interface
	- NTN CAN Driver (kernel built-in) will handle the CAN interface
	
	Additional Details in Neutrino_ETH/Readme & Neutrino_CAN_kernel/Readme
	
Compilation & Installation: Need to be root user to execute the following steps.
===============================================================================
1. Compile:

	a. Build kernel with Neutrino CAN patch  (See Neutrino_CAN_kernel/Readme) 
	b. Compile Neutrino PCIE Driver (See Neutrino_PCIe/Readme)
	c. Compile Neutrino Ethernet Driver (Neutrino_ETH/Readme)

2. Driver Installation:

	a.  Load the PCIe Driver
		#insmod neutrino_pcie.ko
	b.	Load the Ethernet Driver
		#insmod DWC_ETH_QOS.ko

3.	Interface Setup:

	a. See Neutrino_ETH/Readme
	b. See Neutrino_CAN_kernel/Readme
