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

/*!@file: neutrino_pcie_drv.c
 * @brief: Parent PCIe Driver
 */

/*!@file: neutrino_pci_drv.c
 * @brief: Parent PCIe Driver .
 */
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <asm/unaligned.h>

#include "neutrino_pcie_common.h"
#include "neutrino_pcie_drv.h"

#ifdef NTN_LOAD_FW
#include "fw_OSless_HWSeq_eMAC_TDM_CAN.h"
#endif


static struct mfd_cell ntn_mfd_cells[] = {
 	[NTN_PCIE_ETH] = {
 		.name = DRV_NAME_NTN_PCI_ETH,
 	},
 	[NTN_PCIE_CAN] = {
 		.name = DRV_NAME_NTN_PCI_CAN,
 	},
};


void ntn_pci_write_register(struct ntn_mf_dev *pmdev, u32 addr, u32 data)
{ 
	iowrite32(data, (void *) (pmdev)->remap_reg_addr + addr);
}
EXPORT_SYMBOL_GPL(ntn_pci_write_register);

void ntn_pci_read_register(struct ntn_mf_dev *pmdev, u32 addr, u32 *data)
{
	*data = ioread32((void *)(pmdev)->remap_reg_addr + addr);
}
EXPORT_SYMBOL_GPL(ntn_pci_read_register);


#ifdef NTN_LOAD_FW
static void ntn_load_fw(struct ntn_mf_dev *pmdev)
{
	unsigned int fw_size = sizeof(fw_data);
	unsigned long adrs, val;

	if(fw_size > 512*1024){
	    printk(KERN_ALERT"Error : FW size exceeds the memory size\n");	
	    return;
	}

	printk("FW Loading Start...\n");
	printk(" FW Size = %d\n", fw_size );

  adrs = 0;//SRAM Start Address
  do
  {
		val =  fw_data[adrs+0] << 0;
                val |= fw_data[adrs+1] << 8;
                val |= fw_data[adrs+2] << 16;
                val |= fw_data[adrs+3] << 24;

                //printk("\t%lx = %lx\n", adrs, val);
 		iowrite32(val, (void*)(pmdev->remap_sram_addr + adrs));
                adrs += 4;
  }while(adrs < fw_size);


	printk("FW Loading Finish.\n");

  /* De-assert M3 reset */
  adrs = 0x1008;
	val = ioread32( (void*)(pmdev->remap_reg_addr + adrs));
        printk("Reset Register value = %lx\n", val);

  val |= 0x1;
 	iowrite32(val, (void*)(pmdev->remap_reg_addr + adrs));

  val &= ~0x3;
 	iowrite32(val, (void*)(pmdev->remap_reg_addr + adrs));

  printk("Neutrino M3 started.\n");
}
#endif


/*!
* \brief API to initialize the device.
*
* \details This probing function gets called (during execution of
* pci_register_driver() for already existing devices or later if a
* new device gets inserted) for all PCI devices which match the ID table
* and are not "owned" by the other drivers yet. This function gets passed
* a "struct pci_dev *" for each device whose entry in the ID table matches
* the device. The probe function returns zero when the driver chooses to take
* "ownership" of the device or an error code (negative number) otherwise.
* The probe function always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
* \param[in] id   - pointer to table of device ID/ID's the driver is inerested.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

static int Neutrino_PCIe_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct ntn_mf_dev     *pmdev;
	struct ntn_mfd_handle *handle;
	int    i, ret = 0;
  ULONG  len;
	ULONG  base;    
  unsigned int rd_val;

	DBGPR("--> Neutrino_PCIe_probe in PCIe driver\n");

	ret = pci_enable_device(pdev);
	if (ret) {
		NMSGPR_ALERT( "%s:Unable to enable device\n", DEVICE_NAME);
		return -ENODEV;
	}

  /* Query and set the appropriate masks for DMA operations. */   
  if ( (ret = pci_set_dma_mask (pdev, DMA_BIT_MASK(64))) &&
  		 (ret = pci_set_consistent_dma_mask (pdev, DMA_BIT_MASK(64))))
  {
  	NMSGPR_ALERT( "%s: 64 bits DMA Configuration not supported, aborting\n", pci_name(pdev));
		ret = -ENODEV;
		goto err_out_dma_mask_failed;
  }
	
	if (pci_request_regions(pdev, DEVICE_NAME)) {
		NMSGPR_ALERT( "%s:Failed to get PCI regions\n", DEVICE_NAME);
		ret = -ENODEV;
		goto err_out_req_reg_failed;
	}

	pmdev = kzalloc(sizeof(*pmdev), GFP_KERNEL);
	if (!pmdev) {
		ret = -ENOMEM;
		goto err_out_alloc_mfd;
	}

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle) {
		ret = -ENOMEM;
		goto err_out_alloc_handle;
	}
	handle->pmdev = pmdev;

  /* Read BAR0 and map the Neutrino register base address 
     Read BAR1 and map the Neutrino SRAM memory address 
     Read BAR2 and map the Neutrino Flash memory address */
	len  = pci_resource_len(pdev, 0);
  base = pci_resource_start(pdev, 0);    
	pmdev->remap_reg_addr = (ULONG)ioremap_nocache(base, len);
	if (!pmdev->remap_reg_addr) {
  	NMSGPR_ALERT( "%s: cannot map Neutrino BAR0, aborting", pci_name(pdev));
    ret = -EIO;
    goto err_out_map_failed;
	}
  DBGPR("BAR0 virtual address = 0x%lx\n", pmdev->remap_reg_addr);

	len  = pci_resource_len(pdev, 2);
  base = pci_resource_start(pdev, 2);    
	pmdev->remap_sram_addr = (ULONG)ioremap_nocache(base, len);
	if (!pmdev->remap_sram_addr) {
  	NMSGPR_ALERT( "%s: cannot map Neutrino BAR2, aborting", pci_name(pdev));
		pci_iounmap(pdev, (void __iomem *)pmdev->remap_reg_addr);
    ret = -EIO;
    goto err_out_map_failed;
	}
  DBGPR("BAR2 virtual address = 0x%lx\n", pmdev->remap_sram_addr);

	len  = pci_resource_len(pdev, 4);
  base = pci_resource_start(pdev, 4);    
	pmdev->remap_flash_addr = (ULONG)ioremap_nocache(base, len);
	if (!pmdev->remap_flash_addr) {
  	NMSGPR_ALERT( "%s: cannot map Neutrino BAR4, aborting", pci_name(pdev));
		pci_iounmap(pdev, (void __iomem *)pmdev->remap_reg_addr);
		pci_iounmap(pdev, (void __iomem *)pmdev->remap_sram_addr);
    ret = -EIO;
    goto err_out_map_failed;
	}
  DBGPR("BAR4 virtual address = 0x%lx\n", pmdev->remap_flash_addr);
   
#ifdef NTN_LOAD_FW
  ntn_load_fw(pmdev);
#endif

	pmdev->pci = pdev;
	pci_set_drvdata(pdev, handle);


	/* Enable MSI support: following commneted code just enables the one MSI interrupt. This is only kept for debugging purpose.*/
#if 1 
	ret = pci_enable_msi(pdev);
  if(ret)
  {
		NMSGPR_ALERT( "%s:Enable MSI error\n", DEVICE_NAME);
		pmdev->msi_en = false;
		goto err_out_msi_failed;
  }
	else 
	{
		pmdev->msi_en = true;
		pmdev->msi_shared = true;
		/* !!!!!!!!!!!!!!!!! TBD: need to be able allocate multiple MSI and assign different IRQ to ETH and CAN */
		pmdev->irq_eth = pdev->irq;
		pmdev->irq_can = pdev->irq;
	}
  pci_write_config_word(pdev, pdev->msi_cap + PCI_MSI_MASK_64, 0);
#else
  //ret = pci_enable_msi_range(pdev, 1, 4);
  ret = pci_enable_msi_block(pdev, 2);
  if(ret < 0) //Failed to allocate
  {
  	NMSGPR_ALERT( "--------------->%s:Enable MSI error\n", DEVICE_NAME);
		pmdev->msi_en = false;
    goto err_out_msi_failed;
  } 
	else if (ret > 0) {   //This is the possible number that device can allocate
  	//pdata->max_irq = ret;
		pmdev->msi_en = true;
		if (ret < 2) {
			pmdev->msi_shared = true;
			/* !!!!!!!!!!!!!!!!! TBD: need to be able allocate multiple MSI and assign different IRQ to ETH and CAN */
			pmdev->irq_eth = pdev->irq;
			pmdev->irq_can = pdev->irq;
		}
		else { // NOT possible to be here!!
			pmdev->msi_shared = false;
			pmdev->irq_eth = pdev->irq;
			pmdev->irq_can = pdev->irq + 1;
		}
    NMSGPR_ALERT( "--------------->%s:Only %d MSI is allocated: NUM=%d\n", DEVICE_NAME, ret, pdev->irq ); 
  }
	else {
		pmdev->msi_en = true;
		pmdev->msi_shared = false;
		pmdev->irq_eth = pdev->irq;
		pmdev->irq_can = pdev->irq + 1;
  	NMSGPR_ALERT( "------------>%s:Successfully Allocated 2 MSI\n", DEVICE_NAME); 
	}
  pci_write_config_word(pdev, pdev->msi_cap + PCI_MSI_MASK_64, 0);
#endif

    
	pci_set_master(pdev);


	// Init Neutrino
	DBGPR("Replace me with APIs once driver is mature\n"); 
  rd_val = *(unsigned int*)(pmdev->remap_reg_addr + 0x1004);
  *(unsigned int*)(pmdev->remap_reg_addr + 0x1004) = (rd_val | 0x80 | 0x400 | 0x10); //Enable MAC, CAN, INTC Clock
       
  rd_val = *(unsigned int*)(pmdev->remap_reg_addr + 0x1008);
  *(unsigned int*)(pmdev->remap_reg_addr + 0x1008) = (rd_val & ~(0x80 | 0x400 | 0x10)); //Deassert MAC, CAN, INTC Reset

  /* Enable can0 & can1 pin mux */
  rd_val = *(unsigned int*)(pmdev->remap_reg_addr + 0x100C);
  *(unsigned int*)(pmdev->remap_reg_addr + 0x100C) =  (rd_val | 0x5000);

	for (i = 0; i < ARRAY_SIZE(ntn_mfd_cells); i++) {
		ntn_mfd_cells[i].platform_data = handle;
		ntn_mfd_cells[i].pdata_size = sizeof(*handle);
	}

#if 1
ret =  mfd_add_devices(&pdev->dev, -2, ntn_mfd_cells, ARRAY_SIZE(ntn_mfd_cells), NULL, 0, NULL);
	if (ret < 0)
		goto err_out_mfd_add;
#endif

	DBGPR("<-- Neutrino_PCIe_probe\n");

	return ret;

err_out_mfd_add:
	pci_disable_msi(pdev);

err_out_msi_failed:
	pci_iounmap(pdev, (void __iomem *)pmdev->remap_reg_addr);
	pci_iounmap(pdev, (void __iomem *)pmdev->remap_sram_addr);
	pci_iounmap(pdev, (void __iomem *)pmdev->remap_flash_addr);
	pci_set_drvdata(pdev, NULL);

err_out_map_failed:
	kfree(handle);

err_out_alloc_handle:
	kfree(pmdev);

err_out_alloc_mfd:
	pci_release_regions(pdev);

err_out_req_reg_failed:
err_out_dma_mask_failed:
	pci_disable_device(pdev);

	return ret;
}

/*!
* \brief API to release all the resources from the driver.
*
* \details The remove function gets called whenever a device being handled
* by this driver is removed (either during deregistration of the driver or
* when it is manually pulled out of a hot-pluggable slot). This function
* should reverse operations performed at probe time. The remove function
* always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to pci_dev structure.
*
* \return void
*/

static void Neutrino_PCIe_remove(struct pci_dev *pdev)
{
 	struct ntn_mfd_handle *handle = pci_get_drvdata(pdev);
	struct ntn_mf_dev     *pmdev = NULL;

	DBGPR("--> Neutrino_PCIe_remove\n");

 	if (handle) {
		pmdev = handle->pmdev;
  }

	mfd_remove_devices(&pdev->dev);

 	if ( pmdev != NULL )
  {
	  pmdev->irq_eth = 0;
	  pmdev->irq_can = 0;

	  DBGPR(" Neutrino_PCIe_remove: iounmap\n");
	  pci_iounmap(pdev, (void __iomem *)pmdev->remap_reg_addr);
	  pci_iounmap(pdev, (void __iomem *)pmdev->remap_sram_addr);
	  pci_iounmap(pdev, (void __iomem *)pmdev->remap_flash_addr);

		kfree(pmdev);
  } 

	pci_disable_msi(pdev);
	pci_set_drvdata(pdev, NULL);
       
	kfree(handle);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	DBGPR("<-- Neutrino_PCIe_remove\n");

	return;
}

static struct pci_device_id Neutrino_PCIe_id = {
	PCI_DEVICE(VENDOR_ID, DEVICE_ID)
};

struct pci_dev *Neutrino_pcidev;

static void Neutrino_PCIe_shutdown(struct pci_dev *pdev);
static INT Neutrino_PCIe_suspend_late(struct pci_dev *pdev, pm_message_t state);
static INT Neutrino_PCIe_resume_early(struct pci_dev *pdev);
#ifdef CONFIG_PM
static INT Neutrino_PCIe_suspend(struct pci_dev *pdev, pm_message_t state);
static INT Neutrino_PCIe_resume(struct pci_dev *pdev);
#endif


static struct pci_driver Neutrino_pci_driver = {
	.name 				= "Neutrino PCIe",
	.id_table 		= &Neutrino_PCIe_id,
	.probe 				= Neutrino_PCIe_probe,
	.remove 			= Neutrino_PCIe_remove,
  .shutdown 		= Neutrino_PCIe_shutdown,
  .suspend_late = Neutrino_PCIe_suspend_late,
  .resume_early = Neutrino_PCIe_resume_early,
#ifdef CONFIG_PM
  .suspend 			= Neutrino_PCIe_suspend,
  .resume 			= Neutrino_PCIe_resume,
#endif

	.driver 			= {
		   							.name = DEVICE_NAME,
		   							.owner = THIS_MODULE,
									},
};

static void Neutrino_PCIe_shutdown(struct pci_dev *pdev)
{
	NMSGPR_ALERT( "-->Neutrino_PCIe_shutdown\n");
	NMSGPR_ALERT( "Handle the shutdown\n");
	NMSGPR_ALERT( ">--Neutrino_PCIe_shutdown\n");

	return;
}

static INT Neutrino_PCIe_suspend_late(struct pci_dev *pdev, pm_message_t state)
{
	NMSGPR_ALERT( "-->Neutrino_PCIe_suspend_late\n");
	NMSGPR_ALERT( "Handle the suspend_late\n");
	NMSGPR_ALERT( "<--Neutrino_PCIe_suspend_late\n");

	return 0;
}

static INT Neutrino_PCIe_resume_early(struct pci_dev *pdev)
{
	NMSGPR_ALERT( "-->Neutrino_PCIe_resume_early\n");
	NMSGPR_ALERT( "Handle the resume_early\n");
	NMSGPR_ALERT( "<--Neutrino_PCIe_resume_early\n");

	return 0;
}

#ifdef CONFIG_PM
/*!
 * \brief Routine to put the device in suspend mode
 *
 * \details This function gets called by PCI core when the device is being
 * suspended. The suspended state is passed as input argument to it.
 * Following operations are performed in this function,
 * - stop the phy.
 * - detach the device from stack.
 * - stop the queue.
 * - Disable napi.
 * - Stop DMA TX and RX process.
 * - Enable power down mode using PMT module or disable MAC TX and RX process.
 * - Save the pci state.
 *
 * \param[in] pdev – pointer to pci device structure.
 * \param[in] state – suspend state of device.
 *
 * \return int
 *
 * \retval 0
 */

static INT Neutrino_PCIe_suspend(struct pci_dev *pdev, pm_message_t state)
{
	//struct net_device *dev = pci_get_drvdata(pdev);
	//struct DWE_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	//INT ret, pmt_flags = 0;

	DBGPR("-->Neutrino_PCIe_suspend\n");


	//ret = Neutrino_powerdown(dev, pmt_flags, Neutrino_DRIVER_CONTEXT);
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	DBGPR("<--Neutrino_PCIe_suspend\n");

	return 0;
}

/*!
 * \brief Routine to resume device operation
 *
 * \details This function gets called by PCI core when the device is being
 * resumed. It is always called after suspend has been called. These function
 * reverse operations performed at suspend time. Following operations are
 * performed in this function,
 * - restores the saved pci power state.
 * - Wakeup the device using PMT module if supported.
 * - Starts the phy.
 * - Enable MAC and DMA TX and RX process.
 * - Attach the device to stack.
 * - Enable napi.
 * - Starts the queue.
 *
 * \param[in] pdev – pointer to pci device structure.
 *
 * \return int
 *
 * \retval 0
 */

static INT Neutrino_PCIe_resume(struct pci_dev *pdev)
{
//	struct net_device *dev = pci_get_drvdata(pdev);
	INT ret = 0;

	DBGPR("-->Neutrino_PCIe_resume\n");

//	if (!dev ) {
//		DBGPR("<--Neutrino_dev_resume\n");
//		return -EINVAL;
//	}

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	//ret = Neutrino_powerup(dev, Neutrino_DRIVER_CONTEXT);

	DBGPR("<--Neutrino_PCIe_resume\n");

	return ret;
}

#endif	/* CONFIG_PM */

/*!
* \brief API to register the driver.
*
* \details This is the first function called when the driver is loaded.
* It register the driver with PCI sub-system
*
* \return void.
*/

static int __init Neutrino_init_module(void)
{
	INT ret = 0;

	DBGPR("\n\n-->Neutrino_init_module\n");

	ret = pci_register_driver(&Neutrino_pci_driver);
	if (ret < 0) {
		NMSGPR_ALERT( "Neutrino_init_module:driver registration failed");
		return ret;
	}

	DBGPR("<--Neutrino_init_module\n");

	return ret;
}

/*!
* \brief API to unregister the driver.
*
* \details This is the first function called when the driver is removed.
* It unregister the driver from PCI sub-system
*
* \return void.
*/

static void __exit Neutrino_exit_module(void)
{
	DBGPR("-->Neutrino_exit_module\n");

	pci_unregister_driver(&Neutrino_pci_driver);

	DBGPR("<--Neutrino_exit_module\n\n");
}

/*!
* \brief Macro to register the driver registration function.
*
* \details A module always begin with either the init_module or the function
* you specify with module_init call. This is the entry function for modules;
* it tells the kernel what functionality the module provides and sets up the
* kernel to run the module's functions when they're needed. Once it does this,
* entry function returns and the module does nothing until the kernel wants
* to do something with the code that the module provides.
*/
module_init(Neutrino_init_module);

/*!
* \brief Macro to register the driver un-registration function.
*
* \details All modules end by calling either cleanup_module or the function
* you specify with the module_exit call. This is the exit function for modules;
* it undoes whatever entry function did. It unregisters the functionality
* that the entry function registered.
*/
module_exit(Neutrino_exit_module);

/*!
* \brief Macro to declare the module author.
*
* \details This macro is used to declare the module's authore.
*/
MODULE_AUTHOR("Toshiba America Electronic Component");

/*!
* \brief Macro to describe what the module does.
*
* \details This macro is used to describe what the module does.
*/
MODULE_DESCRIPTION("Neutrino PCIe Driver");

/*!
* \brief Macro to describe the module license.
*
* \details This macro is used to describe the module license.
*/
MODULE_LICENSE("GPL");
