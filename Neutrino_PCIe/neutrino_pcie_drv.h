/* ============================================================================
* COPYRIGHT © 2015
*
* Toshiba America Electronic Components
*
* PROJECT:   NEUTRINO
*
* Permission is hereby granted,
* free of charge, to any person obtaining a copy of this software annotated
* with this license and the Software, to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify,
* merge, publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
*
* EXAMPLE PROGRAMS ARE PROVIDED AS-IS WITH NO WARRANTY OF ANY KIND, 
* EITHER EXPRESS OR IMPLIED.
*
* TOSHIBA ASSUMES NO LIABILITY FOR CUSTOMERS' PRODUCT DESIGN OR APPLICATIONS.
* 
* THIS SOFTWARE IS PROVIDED AS-IS AND HAS NOT BEEN FULLY TESTED.  IT IS
* INTENDED FOR REFERENCE USE ONLY.
* 
* TOSHIBA DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES AND ALL LIABILITY OR
* ANY DAMAGES ASSOCIATED WITH YOUR USE OF THIS SOFTWARE.
*
* THIS SOFTWARE IS BEING DISTRIBUTED BY TOSHIBA SOLELY ON AN "AS IS" BASIS
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE HEREBY DISCLAIMED. IN NO EVENT SHALL TOSHIBA BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
* ========================================================================= */

/*! History:   
 *  01-August-2016	: Initial 
 */

/*!@file: neutrino_pcie_drv.h
 * @brief: Parent PCIe Driver
 */

#ifndef __Neutrino_PCIe__PCI_H__
#define __Neutrino_PCIe__PCI_H__

/**************************** Neutrino Defines Starts Here ***************************/

#define NTN_MAX_MSI         (1)

//Neutrino Debugging
#define NTN_DEBUG_L1	
//#define NTN_DEBUG_L2		
//#define NTN_DEBUG_TS1
//#define NTN_DEBUG_TS2

//#define NTN_LOAD_FW

#define VENDOR_ID 												0x1179
#define DEVICE_ID 												0x021a
#define DEVICE_NAME 											"TSB_NEUTRINO_PCIE"

#define DRV_NAME_NTN_PCI              	 	"neutrino_pcie"
#define DRV_NAME_NTN_PCI_ETH         			"DWC_ETH_QOS"
#define DRV_NAME_NTN_PCI_CAN          	  "ntn_pci_can"

#define NTN_PCIE_ETH                     	0
#define NTN_PCIE_CAN                     	1

struct ntn_mf_dev;
struct ntn_mfd_handle {
	struct ntn_mf_dev              *pmdev;
	struct device  									dev; 
};

struct ntn_mf_dev {
	struct pci_dev                  *pci;
  unsigned int                    id;

  /* pci resources */
  unsigned long                   addr;
  unsigned long 									remap_reg_addr;
  unsigned long                   remap_flash_addr;
  unsigned long                   remap_sram_addr;

  bool                            msi_en;
  bool                            msi_shared;
  int                             irq_eth;
  int                             irq_can;
};


void ntn_pci_write_register(struct ntn_mf_dev *pmdev, u32 addr, u32 data);
void ntn_pci_read_register(struct ntn_mf_dev *pmdev, u32 addr, u32 *data);


#endif


