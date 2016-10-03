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

/* =========================================================================
* The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
* "Software") is an unsupported proprietary work of Synopsys, Inc. unless
* otherwise expressly agreed to in writing between Synopsys and you.
*
* The Software IS NOT an item of Licensed Software or Licensed Product under
* any End User Software License Agreement or Agreement for Licensed Product
* with Synopsys or any supplement thereto.  Permission is hereby granted,
* free of charge, to any person obtaining a copy of this software annotated
* with this license and the Software, to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify,
* merge, publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
* ========================================================================= */

/*! History:   
 *	2-March-2016	: Initial 
 *	21-March-2016	: Modified "pdev->msi_cap + PCI_MSI_MASK_64" register 
 *						read as "pci_write_config_dword"
 *	20-May-2016	: Changes made to Support for multiple Neutrino device 
 *			  connection
 *      3-Jun-2016	: Removed Extra TX/RX Queue/DMA Channel initialization.
 *     25-July-2016	: Updated DWC_ETH_QOS_remove() to explicitly release
 *     			  NAPI resources. This was causing a regression in the 
 *     			  driver hot-plug feature.
 */

/*!@file: DWC_ETH_QOS_platform.c
 * @brief: Driver functions.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/unaligned.h>

#include "DWC_ETH_QOS_yheader.h"
#include "neutrino_pcie_drv.h"

static UCHAR dev_addr[6] = { 0xE8, 0xE0, 0xB7, 0xB5, 0x7D, 0xF8};   
typedef struct
{
	char mdio_key[32];
	unsigned short mdio_key_len;
	char mac_key[32];
	unsigned short mac_key_len;
	unsigned short mac_str_len;
	char mac_str_def[20];
} config_param_list_t;

static const config_param_list_t config_param_list[] = {
{"MDIOBUSID",9,"MAC_ID", 6, 18, "00:00:00:00:00:00"},
};

#define CONFIG_PARAM_NUM (sizeof(config_param_list)/sizeof(config_param_list[0]))
/* Holds virtual address for BAR0 for register access,
   BAR1 for SRAM memory and BAR2 for Flash memory */
#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
ULONG cur_virt;    
#endif  

USHORT mdio_bus_id;

void DWC_ETH_QOS_init_all_fptrs(struct DWC_ETH_QOS_prv_data *pdata)
{
	DWC_ETH_QOS_init_function_ptrs_dev(&pdata->hw_if);
	DWC_ETH_QOS_init_function_ptrs_desc(&pdata->desc_if);
}

/*!
 * \brief API to kernel read from file
 *
 * \param[in] file   - pointer to file descriptor
 * \param[in] offset - Offset of file to start read
 * \param[in] size   - Size of the buffer to be read
 *
 * \param[out] data   - File data buffer
 *
 *
 * \return integer
 *
 * \retval 0 on success & -ve number on failure.
 */
static int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

/*!
 * \brief API to validate MAC ID
 *
 * \param[in] char *s - pointer to MAC ID string
 * 
 * \return boolean
 *
 * \retval true on success and false on failure.
 */
bool isMAC(char *s) {
    int i=0;
    if (s == NULL)
	return false;

    for(i = 0; i < 17; i++) {
        if(i % 3 != 2 && !isxdigit(s[i]))
            return false;
        if(i % 3 == 2 && s[i] != ':')
            return false;
    }
    return true;
}

/*!
 * \brief API to extract MAC ID from given string
 *
 * \param[in] char *string - pointer to MAC ID string
 * 
 * \return None
 */
void extract_macid(char *string)
{
	char *token_m = NULL;
	int j = 0;
        int mac_id = 0;

	/* Extract MAC ID byte by byte */
	token_m = strsep(&string, ":");
	while(token_m != NULL) {
		sscanf(token_m, "%x", &mac_id);
		dev_addr[j++] = mac_id;
		token_m = strsep(&string, ":");
	}
}

/*!
 * \brief API to parse and extract the user configured MAC ID
 *
 * \param[in] file_buf - Pointer to file data buffer
 *
 * \return boolean
 *
 * \return - True on Success and False in failure 
 */
static bool lookfor_macid(char *file_buf)
{
	char *string = NULL, *token_n = NULL, *token_s = NULL, *token_m = NULL;
	bool status = false;
	int ntn_device_no= 0;

	string = file_buf;
	/* Parse Line-0 */	
	token_n = strsep(&string, "\n");
	while (token_n != NULL) {

		/* Check if line is enabled */
		if (token_n[0] != '#') {
			/* Extract the token based space character */
			token_s = strsep(&token_n, " ");
			if (token_s != NULL) {
			if (strncmp (token_s, config_param_list[0].mdio_key, 9) == 0 ) {
					token_s = strsep(&token_n, " ");
					token_m = strsep(&token_s, ":");
					sscanf(token_m, "%d", &ntn_device_no);
					if (ntn_device_no != mdio_bus_id){
						if ((token_n = strsep(&string, "\n")) == NULL)
							break;
						continue;
					}
				}
			}
			
			/* Extract the token based space character */
			token_s = strsep(&token_n, " ");
			if (token_s != NULL) {
				/* Compare if parsed string matches with key listed in configuration table */
				if (strncmp (token_s, config_param_list[0].mac_key, 6) == 0 ) {

					NDBGPR_L1("MAC_ID Key is found\n");
					/* Read next word */
					token_s = strsep(&token_n, " \n");
					if (token_s != NULL) {

						/* Check if MAC ID length  and MAC ID is valid */
						if ((isMAC(token_s) == true) && (strlen(token_s) ==  config_param_list[0].mac_str_len)) {
							/* If user configured MAC ID is valid,  assign default MAC ID */
							extract_macid(token_s);
							status = true;
						} else {
							NMSGPR_ALERT( "Valid Mac ID not found\n");
						}
					}
				}
			}
		}
		/* Read next lile */
                if ((token_n = strsep(&string, "\n")) == NULL)
			break;
		
	}
	return status;
}

/*!
 * \brief Parse the user configuration file for various config
 *
 * \param[in] None
 *
 * \return None
 *
 */
static void parse_config_file(void)
{
	struct file *filep = NULL;
	char data[1000]={0};
	mm_segment_t oldfs;
	int ret, flags = O_RDONLY, i = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filep = filp_open("config.ini", flags, 0600);
	set_fs(oldfs);
	if(IS_ERR(filep)) {
		NMSGPR_ALERT( "Mac configuration file not found\n");
		NMSGPR_ALERT( "Using Default MAC Address\n");
		return;
	}
	else  {
		/* Parse the file */
		ret = file_read(filep, 0, data, 1000);
		for (i = 0; i < CONFIG_PARAM_NUM; i++) {
			if (strstr ((const char *)data, config_param_list[i].mdio_key)) {
				NDBGPR_L1("Pattern Match\n");
				if (strncmp(config_param_list[i].mdio_key, "MDIOBUSID", 9) == 0) {
					/* MAC ID Configuration */
					NDBGPR_L1("MAC_ID Configuration\n");
					if (lookfor_macid(data) == false) {
						//extract_macid ((char *)config_param_list[i].str_def);
					}
		}
			}
		}
	}

	filp_close(filep, NULL);

	return;
}


/*!
* \brief API to initialize the device.
*
* \details This probing function gets called (during execution of
* platform_driver_register() for already existing devices or later if a
* new device gets inserted). This function gets passed a "struct platform_device *" 
* for each device. The probe function returns zero when the driver chooses to take
* "ownership" of the device or an error code (negative number) otherwise.
* The probe function always gets called from process context, so it can sleep.
*
* \param[in] pfdev - pointer to platform_device structure.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

static int DWC_ETH_QOS_probe(struct platform_device *pfdev)
{
  struct ntn_mfd_handle       *handle = pfdev->dev.platform_data;
	struct ntn_mf_dev           *pmdev;
  struct pci_dev              *pdev;
  struct DWC_ETH_QOS_prv_data *pdata = NULL;
  struct net_device           *dev = NULL;
  struct hw_if_struct         *hw_if = NULL;
  struct desc_if_struct       *desc_if = NULL;
  int                          i, ret = 0;
	UCHAR                        tx_q_count = 0, rx_q_count = 0;
	unsigned int                 reg_val;
	ULONG dwc_eth_ntn_reg_pci_base_addr; 
	ULONG dwc_eth_ntn_FLASH_pci_base_addr;
	ULONG dwc_eth_ntn_SRAM_pci_base_addr_virt;
#ifdef NTN_DECLARE_MEM_FOR_DMAAPI
	ULONG                        phy_mem_adrs;
#endif

#ifdef NTN_ENABLE_PCIE_MEM_ACCESS
	ULONG_LONG                  adrs_to_be_replaced, adrs_for_replacement;
	UINT                        tmap_no, no_of_bits;
#endif

	NDBGPR_L1("Debug version\n");
	DBGPR("--> DWC_ETH_QOS_probe\n");

	if (!handle) {
    DBGPR("--> can not get the handle!!!!!!!!\n");
    return -ENXIO;
  }

  pmdev = handle->pmdev;
  if (!pmdev){
    DBGPR("--> can not get the pmdev!!!!!!!!\n");
    return -ENXIO;
  }

  pdev = pmdev->pci;

	/* Read BAR0 and map the Neutrino register base address 
     Read BAR1 and map the Neutrino SRAM memory address 
     Read BAR2 and map the Neutrino Flash memory address */
	dwc_eth_ntn_reg_pci_base_addr = pmdev->remap_reg_addr;
  dwc_eth_ntn_SRAM_pci_base_addr_virt = pmdev->remap_sram_addr;
#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
  cur_virt = dwc_eth_ntn_SRAM_pci_base_addr_virt + NTN_DMAAPI_MEM_OFFSET;
#endif
  dwc_eth_ntn_FLASH_pci_base_addr = pmdev->remap_flash_addr;


  NDBGPR_L1( "BAR0 virtual address = 0x%lx\n", dwc_eth_ntn_reg_pci_base_addr);
  NDBGPR_L1( "BAR2 virtual address = 0x%lx\n", dwc_eth_ntn_SRAM_pci_base_addr_virt);
  NDBGPR_L1( "BAR4 virtual address = 0x%lx\n", dwc_eth_ntn_FLASH_pci_base_addr);
   
  if (((void __iomem *)dwc_eth_ntn_reg_pci_base_addr == NULL)  ||
      ((void __iomem *)dwc_eth_ntn_SRAM_pci_base_addr_virt == NULL) ||
      ((void __iomem *)dwc_eth_ntn_FLASH_pci_base_addr == NULL) )
	{
		NMSGPR_ALERT( "%s: cannot map Neutrino BARs, aborting", pci_name(pdev));
    ret = -EIO;
    goto err_out_map_failed;
  }

	// Debug print
  NDBGPR_L1( "HFR0 Val = 0x%08x \n", *(unsigned int*)(dwc_eth_ntn_reg_pci_base_addr + 0xA11C));
  NDBGPR_L1( "HFR1 Val = 0x%08x \n", *(unsigned int*)(dwc_eth_ntn_reg_pci_base_addr + 0xA120));
  NDBGPR_L1( "HFR2 Val = 0x%08x \n", *(unsigned int*)(dwc_eth_ntn_reg_pci_base_addr + 0xA124));

	/* queue count */
	tx_q_count = get_tx_queue_count(dwc_eth_ntn_reg_pci_base_addr);
	rx_q_count = get_rx_queue_count(dwc_eth_ntn_reg_pci_base_addr);

  NDBGPR_L1("No of TX Queue = %d\n", tx_q_count);
  NDBGPR_L1("No of RX Queue = %d\n", rx_q_count);

	/* Need to add 2 to number of queue as this api corresponds to number of DMA channels */
	dev = alloc_etherdev_mqs(sizeof(struct DWC_ETH_QOS_prv_data), (tx_q_count+2), (rx_q_count+2));
	if (dev == NULL) {
		NMSGPR_ALERT( "%s:Unable to alloc new net device\n", DEV_NAME);
		ret = -ENOMEM;
		goto err_out_dev_failed;
	}
	++mdio_bus_id;
	/* Read mac address from mac.ini file */
	parse_config_file();
	if(!is_valid_ether_addr(dev_addr)) {
		NMSGPR_ALERT( "Not found valid mac address\n");
		NMSGPR_ALERT( "Using Default MAC Address\n");
	}
	NMSGPR_INFO( "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", 
		dev_addr[0], dev_addr[1], dev_addr[2], dev_addr[3], dev_addr[4], dev_addr[5]);
	dev->dev_addr[0] = dev_addr[0];
	dev->dev_addr[1] = dev_addr[1];
	dev->dev_addr[2] = dev_addr[2];
	dev->dev_addr[3] = dev_addr[3];
	dev->dev_addr[4] = dev_addr[4];
	dev->dev_addr[5] = dev_addr[5];

	dev->base_addr = dwc_eth_ntn_reg_pci_base_addr;
	SET_NETDEV_DEV(dev, &pfdev->dev);
	pdata = netdev_priv(dev);
	DWC_ETH_QOS_init_all_fptrs(pdata);
	hw_if = &(pdata->hw_if);
	desc_if = &(pdata->desc_if);

	platform_set_drvdata(pfdev, dev);
	pdata->pdev = pdev;
	pdata->pfdev = pfdev;

	pdata->dev = dev;
	pdata->tx_dma_ch_cnt = tx_q_count + 2;
	pdata->rx_dma_ch_cnt = rx_q_count + 2;
	pdata->dwc_eth_ntn_FLASH_pci_base_addr = dwc_eth_ntn_FLASH_pci_base_addr;
	pdata->dwc_eth_ntn_SRAM_pci_base_addr_virt = dwc_eth_ntn_SRAM_pci_base_addr_virt;
	
	/* 	Host: TX DMA CH : 0,2,3,4, 
		Host: RX DMA CH : 0,2,3,4,5 */	
	for(i=0; i<pdata->rx_dma_ch_cnt; i++)
		pdata->rx_dma_ch_for_host[i] = 1;
	for(i=0; i<pdata->tx_dma_ch_cnt; i++)
		pdata->tx_dma_ch_for_host[i] = 1;

	/* 	Host: TX Q : 0,1,2 
		Host: RX Q : 0,1,2,3 */	
	for(i=0; i<rx_q_count; i++)
		pdata->rx_q_for_host[i] = 1;
	for(i=0; i<tx_q_count; i++)
		pdata->tx_q_for_host[i] = 1;

	/* 	M3: TX DMA CH : 1
		M3: RX DMA CH : 1 */	
	pdata->tx_dma_ch_for_host[1] = 0;
	pdata->rx_dma_ch_for_host[1] = 0;

#ifdef NTN_DECLARE_MEM_FOR_DMAAPI
	phy_mem_adrs = pci_resource_start(pdev, 2);
	ret = dma_declare_coherent_memory(&pdata->pdev->dev, phy_mem_adrs + NTN_DMAAPI_MEM_OFFSET,
              NTN_DMAAPI_MEM_BASE + NTN_DMAAPI_MEM_OFFSET, 
              NTN_DMAAPI_MEM_LENGTH, 
              DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
  if(ret == 0){
  	NMSGPR_ALERT( "Coherent memory declaration error!!\n");
    ret = -ENXIO;
    goto err_coherent_mem_declaration;
	}
#endif

#ifdef NTN_TX_DATA_BUF_IN_SRAM
	pdata->tx_mem_pool = dma_pool_create("TX_MEMORY_POOL", &pdata->pdev->dev,
			NTN_TX_MEM_POOL_SIZE, NTN_TX_MEM_POOL_ALIGN, NTN_TX_MEM_POOL_ALLOC);
	if(pdata->tx_mem_pool == NULL)
	{
    NMSGPR_ALERT( "Cann't create TX Memory Pool\n");
    ret = -ENOMEM;
		goto err_tx_mem_pool_creation;
	}
#endif
#ifdef NTN_RX_DATA_BUF_IN_SRAM
	pdata->rx_mem_pool = dma_pool_create("RX_MEMORY_POOL", &pdata->pdev->dev,
			NTN_RX_MEM_POOL_SIZE, NTN_RX_MEM_POOL_ALIGN, NTN_RX_MEM_POOL_ALLOC);
	if(pdata->rx_mem_pool == NULL)
	{
    NMSGPR_ALERT( "Cann't create RX Memory Pool\n");
    ret = -ENOMEM;
		goto err_rx_mem_pool_creation;
	}
#endif

#ifdef NTN_ENABLE_PCIE_MEM_ACCESS
	/* Configure TMAP 0 to access full range of host memory */
	tmap_no = 0;
	adrs_to_be_replaced = ( (unsigned long long)0x00000010 << 32);
	adrs_for_replacement = ( (unsigned long long)0x00000000 << 32);
	no_of_bits = 28;
	hw_if->ntn_config_tamap(tmap_no, adrs_to_be_replaced, adrs_for_replacement, no_of_bits,pdata);
#endif

    /* issue clock enable to GMAC device */
    hw_if->ntn_mac_clock_config(0x1, pdata);
    /* issue software reset to GMAC device */
    hw_if->exit(pdata);
  	
	//Assert TDM reset, in case if it is on.	
	reg_val = hw_if->ntn_reg_rd(0x1008, 0, pdata);		
	reg_val |= (0x1<<6);
	hw_if->ntn_reg_wr(0x1008, reg_val, 0, pdata);		

  pdata->irq_number = pmdev->irq_eth;
	dev->irq = pmdev->irq_eth;
  NDBGPR_L1( "Allocated IRQ Number = %d\n", dev->irq); 

	DWC_ETH_QOS_get_all_hw_features(pdata);
	DWC_ETH_QOS_print_all_hw_features(pdata);

	ret = desc_if->alloc_queue_struct(pdata);
	if (ret < 0) {
		NMSGPR_ALERT( "ERROR: Unable to alloc Tx/Rx queue\n");
		goto err_out_q_alloc_failed;
	}

	dev->netdev_ops = DWC_ETH_QOS_get_netdev_ops();

	pdata->interface = DWC_ETH_QOS_get_phy_interface(pdata);
	/* Bypass PHYLIB for TBI, RTBI and SGMII interface */
	if (1 == pdata->hw_feat.sma_sel) {
		ret = DWC_ETH_QOS_mdio_register(dev);
		if (ret < 0) {
			NMSGPR_ALERT( "MDIO bus (id %d) registration failed\n", pdata->bus_id);
			goto err_out_mdio_reg;
		}
	} 
	else {
		NMSGPR_ALERT( "%s: MDIO is not present\n\n", DEV_NAME);
	}

	/* enabling and registration of irq with magic wakeup */
	if (1 == pdata->hw_feat.mgk_sel) {
		device_set_wakeup_capable(&pdev->dev, 1);
		pdata->wolopts = WAKE_MAGIC;
		enable_irq_wake(dev->irq);
	}

	for (i = 0; i < NTN_RX_DMA_CH_CNT; i++) {
		struct DWC_ETH_QOS_rx_dma_ch *rx_dma_ch = GET_RX_DMA_CH_PTR(i);
		if(!pdata->rx_dma_ch_for_host[i])
			continue;

		netif_napi_add(dev, &rx_dma_ch->napi, DWC_ETH_QOS_poll_mq, (64 * NTN_RX_DMA_CH_CNT));
	}

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0) )
	SET_ETHTOOL_OPS(dev, DWC_ETH_QOS_get_ethtool_ops());
#else	//3.16.0
	netdev_set_default_ethtool_ops(dev, DWC_ETH_QOS_get_ethtool_ops());
#endif	//3.16.0

	DWC_ETH_QOS_reset_ethtool_stats(pdata);

	if (pdata->hw_feat.tso_en) {
		dev->hw_features = NETIF_F_TSO;
		dev->hw_features |= NETIF_F_SG;
		dev->hw_features |= NETIF_F_IP_CSUM;
		dev->hw_features |= NETIF_F_IPV6_CSUM;
		NDBGPR_L2( "Supports TSO, SG and TX COE\n");
	}
	else if (pdata->hw_feat.tx_coe_sel) {
		dev->hw_features = NETIF_F_IP_CSUM ;
		dev->hw_features |= NETIF_F_IPV6_CSUM;
		NDBGPR_L2( "Supports TX COE\n");
	}

	if (pdata->hw_feat.rx_coe_sel) {
		dev->hw_features |= NETIF_F_RXCSUM;
		dev->hw_features |= NETIF_F_LRO;
		NDBGPR_L2( "Supports RX COE and LRO\n");
	}
#ifdef DWC_ETH_QOS_ENABLE_VLAN_TAG
	dev->vlan_features |= dev->hw_features;
	dev->hw_features |= NETIF_F_HW_VLAN_CTAG_RX;
	if (pdata->hw_feat.sa_vlan_ins) {
		dev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
		NDBGPR_L2( "VLAN Feature enabled\n");
	}
	if (pdata->hw_feat.vlan_hash_en) {
		dev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
		NDBGPR_L2( "VLAN HASH Filtering enabled\n");
	}
#endif /* end of DWC_ETH_QOS_ENABLE_VLAN_TAG */
	dev->features |= dev->hw_features;
	pdata->dev_state |= dev->features;

	DWC_ETH_QOS_init_rx_coalesce(pdata);

#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_init(pdata);
#endif	/* end of DWC_ETH_QOS_CONFIG_PTP */

	spin_lock_init(&pdata->lock);
	spin_lock_init(&pdata->tx_lock);
	spin_lock_init(&pdata->pmt_lock);

	ret = register_netdev(dev);
	if (ret) {
		NMSGPR_ALERT( "%s: Net device registration failed\n", DEV_NAME);
		goto err_out_netdev_failed;
	}

	if (pdata->hw_feat.pcs_sel) {
		netif_carrier_off(dev);
		NMSGPR_ALERT( "carrier off till LINK is up\n");
	}

	DBGPR("<-- DWC_ETH_QOS_probe\n");

	return 0;

err_out_netdev_failed:
#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_remove(pdata);
#endif	/* end of DWC_ETH_QOS_CONFIG_PTP */

	if (1 == pdata->hw_feat.sma_sel)
		DWC_ETH_QOS_mdio_unregister(dev);

err_out_mdio_reg:
	desc_if->free_queue_struct(pdata);

err_out_q_alloc_failed:
#ifdef NTN_RX_DATA_BUF_IN_SRAM
	dma_pool_destroy(pdata->rx_mem_pool);
err_rx_mem_pool_creation:
#endif
#ifdef NTN_TX_DATA_BUF_IN_SRAM
	dma_pool_destroy(pdata->tx_mem_pool);
err_tx_mem_pool_creation:
#endif
#ifdef NTN_DECLARE_MEM_FOR_DMAAPI
	dma_release_declared_memory(&pdata->pdev->dev);

err_coherent_mem_declaration:
#endif
    /* issue clock disable to GMAC device */
    hw_if->ntn_mac_clock_config(0x0, pdata);
    
	free_netdev(dev);
	platform_set_drvdata(pfdev, NULL);

err_out_dev_failed:
err_out_map_failed:

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
* \param[in] pfdev - pointer to platform_device structure.
*
* \return void
*/
static int DWC_ETH_QOS_remove(struct platform_device *pfdev)
{
  struct net_device           *dev = platform_get_drvdata(pfdev);
  struct DWC_ETH_QOS_prv_data *pdata;
  struct desc_if_struct       *desc_if;
  struct hw_if_struct         *hw_if;
  struct DWC_ETH_QOS_rx_dma_ch *rx_dma_ch = NULL;
  int i;

	DBGPR("--> DWC_ETH_QOS_remove\n");

  pdata = netdev_priv(dev);
  desc_if = &(pdata->desc_if);
  hw_if = &(pdata->hw_if);

	unregister_netdev(dev);

#ifdef DWC_ETH_QOS_CONFIG_PTP
	DWC_ETH_QOS_ptp_remove(pdata);
#endif /* end of DWC_ETH_QOS_CONFIG_PTP */

	if (1 == pdata->hw_feat.sma_sel)
		DWC_ETH_QOS_mdio_unregister(dev);

#ifdef NTN_RX_DATA_BUF_IN_SRAM
	dma_pool_destroy(pdata->rx_mem_pool);
#endif
#ifdef NTN_TX_DATA_BUF_IN_SRAM
	dma_pool_destroy(pdata->tx_mem_pool);
#endif
#ifdef NTN_DECLARE_MEM_FOR_DMAAPI
	dma_release_declared_memory(&pdata->pdev->dev);
#endif

	/* issue clock disable to GMAC device */
    hw_if->ntn_mac_clock_config(0x0, pdata);

	/* If NAPI is enabled, delete any references to the NAPI struct. */
	for (i = 0; i < NTN_RX_DMA_CH_CNT; i++) {
		rx_dma_ch = GET_RX_DMA_CH_PTR(i);
		if(!pdata->rx_dma_ch_for_host[i])
			continue;
		netif_napi_del(&rx_dma_ch->napi);
	}

	desc_if->free_queue_struct(pdata);

	free_netdev(dev);
	platform_set_drvdata(pfdev, NULL);

	DBGPR("<-- DWC_ETH_QOS_remove\n");

	return 0;
}

static void DWC_ETH_QOS_shutdown(struct platform_device *pfdev)
{
	NMSGPR_ALERT( "-->DWC_ETH_QOS_shutdown\n");
	NMSGPR_ALERT( "Handle the shutdown\n");
	NMSGPR_ALERT( ">--DWC_ETH_QOS_shutdown\n");

	return;
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
 * \param[in] pfdev – pointer to platform device structure.
 * \param[in] state – suspend state of device.
 *
 * \return int
 *
 * \retval 0
 */
static INT DWC_ETH_QOS_suspend(struct platform_device *pfdev, pm_message_t state)
{
  struct ntn_mfd_handle       *handle = pfdev->dev.platform_data;
	struct ntn_mf_dev           *pmdev;
  struct pci_dev              *pdev;
  struct net_device           *dev;
  struct DWC_ETH_QOS_prv_data *pdata;
  struct hw_if_struct         *hw_if;
	INT ret, pmt_flags = 0;
	unsigned int rwk_filter_values[] = {
		/* for filter 0 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 1 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 2 CRC is computed on 0 - 7 bytes from offset */
		0x000000ff,

		/* for filter 3 CRC is computed on 0 - 31 bytes from offset */
		0x000000ff,

		/* filter 0, 1 independently enabled and would apply for
		 * unicast packet only filter 3, 2 combined as,
		 * "Filter-3 pattern AND NOT Filter-2 pattern" */
		0x03050101,

		/* filter 3, 2, 1 and 0 offset is 50, 58, 66, 74 bytes
		 * from start */
		0x4a423a32,

		/* pattern for filter 1 and 0, "0x55", "11", repeated 8 times */
		0xe7b77eed,

		/* pattern for filter 3 and 4, "0x44", "33", repeated 8 times */
		0x9b8a5506,
	};

	DBGPR("-->DWC_ETH_QOS_suspend\n");

	if (!handle) {
    DBGPR("--> can not get the handle!!!!!!!!\n");
    return -ENXIO;
  }

  pmdev = handle->pmdev;
  if (!pmdev){
    DBGPR("--> can not get the pmdev !!!!!!!!\n");
    return -ENXIO;
  }

  pdev = pmdev->pci;

  //dev = dev_get_drvdata(&handle->dev);
  dev = dev_get_drvdata(&pdev->dev);
  pdata = netdev_priv(dev);
  hw_if = &(pdata->hw_if);
  
	if (!dev || !netif_running(dev) || (!pdata->hw_feat.mgk_sel &&
			!pdata->hw_feat.rwk_sel)) {
		DBGPR("<--DWC_ETH_QOS_dev_suspend\n");
		return -EINVAL;
	}

	if (pdata->hw_feat.rwk_sel && (pdata->wolopts & WAKE_UCAST)) {
		pmt_flags |= DWC_ETH_QOS_REMOTE_WAKEUP;
		hw_if->configure_rwk_filter(rwk_filter_values, 8, pdata);
	}

	if (pdata->hw_feat.mgk_sel && (pdata->wolopts & WAKE_MAGIC))
		pmt_flags |= DWC_ETH_QOS_MAGIC_WAKEUP;

	ret = DWC_ETH_QOS_powerdown(dev, pmt_flags, DWC_ETH_QOS_DRIVER_CONTEXT);
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	DBGPR("<--DWC_ETH_QOS_suspend\n");

	return ret;
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
static INT DWC_ETH_QOS_resume(struct platform_device *pfdev)
{
  struct ntn_mfd_handle *handle = pfdev->dev.platform_data;
  struct ntn_mf_dev     *pmdev;
  struct pci_dev        *pdev;
  //struct net_device   *dev;
	INT ret;

	DBGPR("-->DWC_ETH_QOS_resume\n");

  if (!handle) {
    DBGPR("--> can not get the handle!!!!!!!!\n");
    return -ENXIO;
  }

  pmdev = handle->pmdev;
  if (!pmdev){
    DBGPR("--> can not get the pmdev !!!!!!!!\n");
    return -ENXIO;
  }

  pdev = pmdev->pci;


	//!!!!!!!!!!!!!!! set up dev = ....

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	//????????????? ret = DWC_ETH_QOS_powerup(dev, DWC_ETH_QOS_DRIVER_CONTEXT);

	DBGPR("<--DWC_ETH_QOS_resume\n");

	return ret;
}

#endif	/* CONFIG_PM */


static struct platform_device_id ntn_pcie_eth_ids[] = {
  {
    .name = DRV_NAME_NTN_PCI_ETH,
  },
  {
  }
};

static struct platform_driver DWC_ETH_QOS_pci_driver = {
  .id_table = ntn_pcie_eth_ids,
  .probe = DWC_ETH_QOS_probe,
  .remove = DWC_ETH_QOS_remove,
  .shutdown = DWC_ETH_QOS_shutdown,
#ifdef CONFIG_PM
  .suspend = DWC_ETH_QOS_suspend,
  .resume = DWC_ETH_QOS_resume,
#endif
  .driver = {
  	.owner = THIS_MODULE,
    .name = DRV_NAME_NTN_PCI_ETH,
  },
};



/*!
* \brief API to register the driver.
*
* \details This is the first function called when the driver is loaded.
* It register the driver with PCI sub-system
*
* \return void.
*/

static int __init DWC_ETH_QOS_init_module(void)
{
	INT ret = 0;

	DBGPR("-->DWC_ETH_QOS_init_module\n");

	ret = platform_driver_register(&DWC_ETH_QOS_pci_driver);
	if (ret < 0) {
		NMSGPR_ALERT( "DWC_ETH_QOS:driver registration failed");
		return ret;
	}

	DBGPR("<--DWC_ETH_QOS_init_module\n");

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

static void __exit DWC_ETH_QOS_exit_module(void)
{
	DBGPR("-->DWC_ETH_QOS_exit_module\n");

	platform_driver_unregister(&DWC_ETH_QOS_pci_driver);

	DBGPR("<--DWC_ETH_QOS_exit_module\n");
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
module_init(DWC_ETH_QOS_init_module);

/*!
* \brief Macro to register the driver un-registration function.
*
* \details All modules end by calling either cleanup_module or the function
* you specify with the module_exit call. This is the exit function for modules;
* it undoes whatever entry function did. It unregisters the functionality
* that the entry function registered.
*/
module_exit(DWC_ETH_QOS_exit_module);

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
MODULE_DESCRIPTION("Neutrino Driver");

/*!
* \brief Macro to describe the module license.
*
* \details This macro is used to describe the module license.
*/
MODULE_LICENSE("GPL");
