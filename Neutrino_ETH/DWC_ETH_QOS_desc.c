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
 *      2-March-2016	: Initial 
 *	20-May-2016	: Changes made to Support for multiple Neutrino device
 *			  connection
 *	3-Jun-2016	: Some Clean up for unused legacy code.
 */

/*!@file: DWC_ETH_QOS_desc.c
 * @brief: Neutrino PCIe Driver.
 */
#include "DWC_ETH_QOS_yheader.h"
#include "DWC_ETH_QOS_desc.h"
#include "DWC_ETH_QOS_yregacc.h"

/*!
* \brief API to free the transmit descriptor memory.
*
* \details This function is used to free the transmit descriptor memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_tx_desc_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					 UINT tx_chCnt)
{
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data = NULL;
	UINT chInx;
	
#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
	return;
#endif

	DBGPR("-->DWC_ETH_QOS_tx_desc_free_mem: tx_chCnt = %d\n", tx_chCnt);

	for (chInx = 0; chInx < tx_chCnt; chInx++) {
		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;
		desc_data = GET_TX_WRAPPER_DESC(chInx);

		if (GET_TX_DESC_PTR(chInx, 0)) {
			dma_free_coherent(&pdata->pdev->dev,
					  (sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
					  GET_TX_DESC_PTR(chInx, 0),
					  GET_TX_DESC_DMA_ADDR(chInx, 0));
			GET_TX_DESC_PTR(chInx, 0) = NULL;
		}
	}

	DBGPR("<--DWC_ETH_QOS_tx_desc_free_mem\n");
}

/*!
* \brief API to free the receive descriptor memory.
*
* \details This function is used to free the receive descriptor memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_rx_desc_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					 UINT rx_chCnt)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data = NULL;
	UINT chInx = 0;

#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
	return;
#endif

	DBGPR("-->DWC_ETH_QOS_rx_desc_free_mem: rx_chCnt = %d\n", rx_chCnt);

	for (chInx = 0; chInx < rx_chCnt; chInx++) {
		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;
		desc_data = GET_RX_WRAPPER_DESC(chInx);

		if (GET_RX_DESC_PTR(chInx, 0)) {
			dma_free_coherent(&pdata->pdev->dev,
					  (sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT),
					  GET_RX_DESC_PTR(chInx, 0),
					  GET_RX_DESC_DMA_ADDR(chInx, 0));
			GET_RX_DESC_PTR(chInx, 0) = NULL;
		}
	}

	DBGPR("<--DWC_ETH_QOS_rx_desc_free_mem\n");
}

/*!
* \brief API to alloc the queue memory.
*
* \details This function allocates the queue structure memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

static int DWC_ETH_QOS_alloc_queue_struct(struct DWC_ETH_QOS_prv_data *pdata)
{
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_alloc_queue_struct: tx_dma_ch_cnt = %d,"\
		"rx_dma_ch_cnt = %d\n", pdata->tx_dma_ch_cnt, pdata->rx_dma_ch_cnt);

	pdata->tx_dma_ch =
		kzalloc(sizeof(struct DWC_ETH_QOS_tx_dma_ch) * pdata->tx_dma_ch_cnt,
		GFP_KERNEL);
	if (pdata->tx_dma_ch == NULL) {
		NMSGPR_ALERT( "ERROR: Unable to allocate Tx queue structure\n");
		ret = -ENOMEM;
		goto err_out_tx_q_alloc_failed;
	}

	pdata->rx_dma_ch =
		kzalloc(sizeof(struct DWC_ETH_QOS_rx_dma_ch) * pdata->rx_dma_ch_cnt,
		GFP_KERNEL);
	if (pdata->rx_dma_ch == NULL) {
		NMSGPR_ALERT( "ERROR: Unable to allocate Rx queue structure\n");
		ret = -ENOMEM;
		goto err_out_rx_q_alloc_failed;
	}

	DBGPR("<--DWC_ETH_QOS_alloc_queue_struct\n");

	return ret;

err_out_rx_q_alloc_failed:
	kfree(pdata->tx_dma_ch);

err_out_tx_q_alloc_failed:
	return ret;
}


/*!
* \brief API to free the queue memory.
*
* \details This function free the queue structure memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void
*/

static void DWC_ETH_QOS_free_queue_struct(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_free_queue_struct\n");

	if (pdata->tx_dma_ch != NULL) {
		kfree(pdata->tx_dma_ch);
		pdata->tx_dma_ch = NULL;
	}

	if (pdata->rx_dma_ch != NULL) {
		kfree(pdata->rx_dma_ch);
		pdata->rx_dma_ch = NULL;
	}

	DBGPR("<--DWC_ETH_QOS_free_queue_struct\n");
}



/*!
* \brief API to allocate the memory for descriptor & buffers.
*
* \details This function is used to allocate the memory for device
* descriptors & buffers
* which are used by device for data transmission & reception.
*
* \param[in] pdata - pointer to private data structure.
*
* \return integer
*
* \retval 0 on success & -ENOMEM number on failure.
*/
#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
extern ULONG cur_virt;
ULONG cur_sram_adr = NTN_DMAAPI_MEM_BASE + NTN_DMAAPI_MEM_OFFSET;
#endif

static INT allocate_buffer_and_desc(struct DWC_ETH_QOS_prv_data *pdata)
{
	INT ret = 0;
	UINT chInx;
#if !defined(NTN_DECLARE_MEM_FOR_DMAAPI) && !defined(DESC_HOSTMEM_BUF_HOSTMEM)
	UINT DESC_MEM_ALLOC_SZ = 4096;
#endif	

	NDBGPR_L1( "-->allocate_buffer_and_desc: TX_DMA_CH_CNT = %d, "\
		"RX_DMA_CH_CNT = %d\n", NTN_TX_DMA_CH_CNT,
		NTN_RX_DMA_CH_CNT);
#ifdef SYSTEM_IS_64
	NDBGPR_L1( "DWC_ETH_QOS :  TX size is %lu  RX size is %lu\n",
		(sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
		(sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT));
#else
	NDBGPR_L1( "DWC_ETH_QOS : TX size is %u  RX size is %u\n",
		(sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
		(sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT));
#endif
	/* Allocate descriptors and buffers memory for all TX queues */
	for (chInx = 0; chInx < NTN_TX_DMA_CH_CNT; chInx++) {

		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;
#if defined(NTN_DECLARE_MEM_FOR_DMAAPI) || defined(DESC_HOSTMEM_BUF_HOSTMEM)
		/* TX descriptors */
		GET_TX_DESC_PTR(chInx, 0) = dma_alloc_coherent(&pdata->pdev->dev,
						(sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
						&(GET_TX_DESC_DMA_ADDR(chInx, 0)),
						GFP_KERNEL);
#else
		if( (cur_sram_adr+DESC_MEM_ALLOC_SZ) < (NTN_DMAAPI_MEM_BASE + NTN_DMAAPI_MEM_OFFSET + NTN_DMAAPI_MEM_LENGTH) )
		{
			GET_TX_DESC_PTR(chInx, 0) = (struct s_TX_NORMAL_DESC *)cur_virt; 
			GET_TX_DESC_DMA_ADDR(chInx, 0) = cur_sram_adr;
	
			cur_virt = cur_virt + DESC_MEM_ALLOC_SZ;
		    cur_sram_adr = cur_sram_adr + DESC_MEM_ALLOC_SZ; 
		}
		else
			GET_TX_DESC_PTR(chInx, 0) = NULL; 
#endif
		NDBGPR_L1("TX_DESC_PTR(%d, 0) = 0x%lx : TX_DESC_DMA_ADDR(%d, 0) = 0x%lx\n" 
			, chInx, (long unsigned int)GET_TX_DESC_PTR(chInx, 0), chInx, (long unsigned int)GET_TX_DESC_DMA_ADDR(chInx, 0));
		if (GET_TX_DESC_PTR(chInx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_tx_desc;
		}

	}

	for (chInx = 0; chInx < NTN_TX_DMA_CH_CNT; chInx++) {

		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;

		/* TX wrapper buffer */
		GET_TX_BUF_PTR(chInx, 0) =
			kzalloc((sizeof(struct DWC_ETH_QOS_tx_buffer) * TX_DESC_CNT),
			GFP_KERNEL);
		if (GET_TX_BUF_PTR(chInx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_tx_buf;
		}
	}

	/* Allocate descriptors and buffers memory for all RX queues */
	for (chInx = 0; chInx < NTN_RX_DMA_CH_CNT; chInx++) {

		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;

#if defined(NTN_DECLARE_MEM_FOR_DMAAPI) || defined(DESC_HOSTMEM_BUF_HOSTMEM)
		/* RX descriptors */
		GET_RX_DESC_PTR(chInx, 0) = dma_alloc_coherent(&pdata->pdev->dev,
						(sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT),
						&(GET_RX_DESC_DMA_ADDR(chInx, 0)),
						GFP_KERNEL);
#else
		if( (cur_sram_adr+DESC_MEM_ALLOC_SZ) < (NTN_DMAAPI_MEM_BASE + NTN_DMAAPI_MEM_OFFSET + NTN_DMAAPI_MEM_LENGTH) )
		{
                	GET_RX_DESC_PTR(chInx, 0) = (struct s_RX_NORMAL_DESC *)cur_virt;
                	GET_RX_DESC_DMA_ADDR(chInx, 0) = cur_sram_adr;

			cur_virt = cur_virt + DESC_MEM_ALLOC_SZ;
		    cur_sram_adr = cur_sram_adr + DESC_MEM_ALLOC_SZ; 
		}
		else
			GET_RX_DESC_PTR(chInx, 0) = NULL; 
#endif
		NDBGPR_L1("RX_DESC_PTR(%d, 0) = 0x%lx : RX_DESC_DMA_ADDR(%d, 0) = 0x%lx\n" 
			, chInx, (long unsigned int)GET_RX_DESC_PTR(chInx, 0), chInx, (long unsigned int)GET_RX_DESC_DMA_ADDR(chInx, 0));
		if (GET_RX_DESC_PTR(chInx, 0) == NULL) {
			ret = -ENOMEM;
			goto rx_alloc_failure;
		}
	}

	for (chInx = 0; chInx < NTN_RX_DMA_CH_CNT; chInx++) {

		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;

		/* RX wrapper buffer */
		GET_RX_BUF_PTR(chInx, 0) =
			kzalloc((sizeof(struct DWC_ETH_QOS_rx_buffer) * RX_DESC_CNT),
			GFP_KERNEL);
		if (GET_RX_BUF_PTR(chInx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_rx_buf;
		}
	}

	DBGPR("<--allocate_buffer_and_desc\n");

	return ret;

 err_out_rx_buf:
	DWC_ETH_QOS_rx_buf_free_mem(pdata, chInx);
	chInx = NTN_RX_DMA_CH_CNT;

 rx_alloc_failure:
	DWC_ETH_QOS_rx_desc_free_mem(pdata, chInx);
	chInx = NTN_TX_DMA_CH_CNT;

 err_out_tx_buf:
	DWC_ETH_QOS_tx_buf_free_mem(pdata, chInx);
	chInx = NTN_TX_DMA_CH_CNT;

 err_out_tx_desc:
	DWC_ETH_QOS_tx_desc_free_mem(pdata, chInx);

	return ret;
}

/*!
* \brief API to initialize the transmit descriptors.
*
* \details This function is used to initialize transmit descriptors.
* Each descriptors are assigned a buffer. The base/starting address
* of the descriptors is updated in device register if required & all
* the private data structure variables related to transmit
* descriptor handling are updated in this function.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void DWC_ETH_QOS_wrapper_tx_descriptor_init_single_q(
			struct DWC_ETH_QOS_prv_data *pdata,
			UINT chInx)
{
	int i;
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(chInx);
	struct DWC_ETH_QOS_tx_buffer *buffer = GET_TX_BUF_PTR(chInx, 0);
	struct s_TX_NORMAL_DESC *desc = GET_TX_DESC_PTR(chInx, 0);
	dma_addr_t desc_dma = GET_TX_DESC_DMA_ADDR(chInx, 0);
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	DBGPR("-->DWC_ETH_QOS_wrapper_tx_descriptor_init_single_q: "\
		"chInx = %u\n", chInx);

	for (i = 0; i < TX_DESC_CNT; i++) {
		GET_TX_DESC_PTR(chInx, i) = &desc[i];
		GET_TX_DESC_DMA_ADDR(chInx, i) =
		    (desc_dma + sizeof(struct s_TX_NORMAL_DESC) * i);
		GET_TX_BUF_PTR(chInx, i) = &buffer[i];
	}

	desc_data->cur_tx = 0;
	desc_data->dirty_tx = 0;
	desc_data->queue_stopped = 0;
	desc_data->tx_pkt_queued = 0;
	desc_data->packet_count = 0;
	desc_data->free_desc_cnt = TX_DESC_CNT;

	hw_if->tx_desc_init(pdata, chInx);
	desc_data->cur_tx = 0;

	DBGPR("<--DWC_ETH_QOS_wrapper_tx_descriptor_init_single_q\n");
}

/*!
* \brief API to initialize the receive descriptors.
*
* \details This function is used to initialize receive descriptors.
* skb buffer is allocated & assigned for each descriptors. The base/starting
* address of the descriptors is updated in device register if required and
* all the private data structure variables related to receive descriptor
* handling are updated in this function.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void DWC_ETH_QOS_wrapper_rx_descriptor_init_single_q(
			struct DWC_ETH_QOS_prv_data *pdata,
			UINT chInx)
{
	int i;
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(chInx);
	struct DWC_ETH_QOS_rx_buffer *buffer = GET_RX_BUF_PTR(chInx, 0);
	struct s_RX_NORMAL_DESC *desc = GET_RX_DESC_PTR(chInx, 0);
	dma_addr_t desc_dma = GET_RX_DESC_DMA_ADDR(chInx, 0);
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	DBGPR("-->DWC_ETH_QOS_wrapper_rx_descriptor_init_single_q: "\
		"chInx = %u\n", chInx);

	memset(buffer, 0, (sizeof(struct DWC_ETH_QOS_rx_buffer) * RX_DESC_CNT));

	for (i = 0; i < RX_DESC_CNT; i++) {
		GET_RX_DESC_PTR(chInx, i) = &desc[i];
		GET_RX_DESC_DMA_ADDR(chInx, i) =
		    (desc_dma + sizeof(struct s_RX_NORMAL_DESC) * i);
		GET_RX_BUF_PTR(chInx, i) = &buffer[i];
		/* allocate skb & assign to each desc */
		if (pdata->alloc_rx_buf(chInx, pdata, GET_RX_BUF_PTR(chInx, i), GFP_KERNEL))
			break;

		wmb();
	}

	desc_data->cur_rx = 0;
	desc_data->dirty_rx = 0;
	desc_data->skb_realloc_idx = 0;
	desc_data->skb_realloc_threshold = MIN_RX_DESC_CNT;
	desc_data->pkt_received = 0;

	hw_if->rx_desc_init(pdata, chInx);
	desc_data->cur_rx = 0;

	DBGPR("<--DWC_ETH_QOS_wrapper_rx_descriptor_init_single_q\n");
}

static void DWC_ETH_QOS_wrapper_tx_descriptor_init(struct DWC_ETH_QOS_prv_data
						   *pdata)
{
	UINT chInx;

	DBGPR("-->DWC_ETH_QOS_wrapper_tx_descriptor_init\n");

	for (chInx = 0; chInx < NTN_TX_DMA_CH_CNT; chInx++) {

		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;
		
		DWC_ETH_QOS_wrapper_tx_descriptor_init_single_q(pdata, chInx);
	}

	DBGPR("<--DWC_ETH_QOS_wrapper_tx_descriptor_init\n");
}

static void DWC_ETH_QOS_wrapper_rx_descriptor_init(struct DWC_ETH_QOS_prv_data
						   *pdata)
{
	struct DWC_ETH_QOS_rx_dma_ch * rx_dma_ch = NULL;
	UINT chInx;

	DBGPR("-->DWC_ETH_QOS_wrapper_rx_descriptor_init\n");

	for (chInx = 0; chInx < NTN_RX_DMA_CH_CNT; chInx++) {

		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;
		
		rx_dma_ch = GET_RX_DMA_CH_PTR(chInx);
		rx_dma_ch->pdata = pdata;

		/* LRO configuration */
		rx_dma_ch->lro_mgr.dev = pdata->dev;
		memset(&rx_dma_ch->lro_mgr.stats, 0,
			sizeof(rx_dma_ch->lro_mgr.stats));
		rx_dma_ch->lro_mgr.features =
			LRO_F_NAPI | LRO_F_EXTRACT_VLAN_ID;
		rx_dma_ch->lro_mgr.ip_summed = CHECKSUM_UNNECESSARY;
		rx_dma_ch->lro_mgr.ip_summed_aggr = CHECKSUM_UNNECESSARY;
		rx_dma_ch->lro_mgr.max_desc = DWC_ETH_QOS_MAX_LRO_DESC;
		rx_dma_ch->lro_mgr.max_aggr = (0xffff/pdata->dev->mtu);
		rx_dma_ch->lro_mgr.lro_arr = rx_dma_ch->lro_arr;
		rx_dma_ch->lro_mgr.get_skb_header = DWC_ETH_QOS_get_skb_hdr;
		memset(&rx_dma_ch->lro_arr, 0, sizeof(rx_dma_ch->lro_arr));
		rx_dma_ch->lro_flush_needed = 0;

		DWC_ETH_QOS_wrapper_rx_descriptor_init_single_q(pdata, chInx);
	}

	DBGPR("<--DWC_ETH_QOS_wrapper_rx_descriptor_init\n");
}

/*!
* \brief API to free the receive descriptor & buffer memory.
*
* \details This function is used to free the receive descriptor & buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_rx_free_mem(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_rx_free_mem\n");

	/* free RX descriptor */
	DWC_ETH_QOS_rx_desc_free_mem(pdata, NTN_RX_DMA_CH_CNT);

	/* free RX skb's */
	DWC_ETH_QOS_rx_skb_free_mem(pdata, NTN_RX_DMA_CH_CNT);

	/* free RX wrapper buffer */
	DWC_ETH_QOS_rx_buf_free_mem(pdata, NTN_RX_DMA_CH_CNT);

	DBGPR("<--DWC_ETH_QOS_rx_free_mem\n");
}

/*!
* \brief API to free the transmit descriptor & buffer memory.
*
* \details This function is used to free the transmit descriptor
* & buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_tx_free_mem(struct DWC_ETH_QOS_prv_data *pdata)
{
	DBGPR("-->DWC_ETH_QOS_tx_free_mem\n");

	/* free TX descriptor */
	DWC_ETH_QOS_tx_desc_free_mem(pdata, NTN_TX_DMA_CH_CNT);

	/* free TX buffer */
	DWC_ETH_QOS_tx_buf_free_mem(pdata, NTN_TX_DMA_CH_CNT);

	DBGPR("<--DWC_ETH_QOS_tx_free_mem\n");
}

/*!
 * \details This function is invoked by other function to free
 * the tx socket buffers.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_tx_skb_free_mem_single_q(struct DWC_ETH_QOS_prv_data *pdata,
							UINT chInx)
{
	UINT i;

	DBGPR("-->DWC_ETH_QOS_tx_skb_free_mem_single_q: chInx = %u\n", chInx);

	for (i = 0; i < TX_DESC_CNT; i++)
		DWC_ETH_QOS_unmap_tx_skb(pdata, GET_TX_BUF_PTR(chInx, i));

	DBGPR("<--DWC_ETH_QOS_tx_skb_free_mem_single_q\n");
}

/*!
* \brief API to free the transmit descriptor skb memory.
*
* \details This function is used to free the transmit descriptor skb memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_tx_skb_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					UINT tx_chCnt)
{
	UINT chInx;

	DBGPR("-->DWC_ETH_QOS_tx_skb_free_mem: tx_chCnt = %d\n", tx_chCnt);

	for (chInx = 0; chInx < tx_chCnt; chInx++){
		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;
		DWC_ETH_QOS_tx_skb_free_mem_single_q(pdata, chInx);
	}
	
	DBGPR("<--DWC_ETH_QOS_tx_skb_free_mem\n");
}


/*!
 * \details This function is invoked by other function to free
 * the rx socket buffers.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_rx_skb_free_mem_single_q(struct DWC_ETH_QOS_prv_data *pdata,
							UINT chInx)
{
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(chInx);
	UINT i;

	DBGPR("-->DWC_ETH_QOS_rx_skb_free_mem_single_q: chInx = %u\n", chInx);

	for (i = 0; i < RX_DESC_CNT; i++) {
		DWC_ETH_QOS_unmap_rx_skb(pdata, GET_RX_BUF_PTR(chInx, i));
	}

	/* there are also some cached data from a chained rx */
	if (desc_data->skb_top)
		dev_kfree_skb_any(desc_data->skb_top);

	desc_data->skb_top = NULL;

	DBGPR("<--DWC_ETH_QOS_rx_skb_free_mem_single_q\n");
}

/*!
* \brief API to free the receive descriptor skb memory.
*
* \details This function is used to free the receive descriptor skb memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_rx_skb_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					UINT rx_chCnt)
{
	UINT chInx;

	DBGPR("-->DWC_ETH_QOS_rx_skb_free_mem: rx_chCnt = %d\n", rx_chCnt);

	for (chInx = 0; chInx < rx_chCnt; chInx++){
		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;
		DWC_ETH_QOS_rx_skb_free_mem_single_q(pdata, chInx);
	}
	
	DBGPR("<--DWC_ETH_QOS_rx_skb_free_mem\n");
}

/*!
* \brief API to free the transmit descriptor wrapper buffer memory.
*
* \details This function is used to free the transmit descriptor wrapper buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_tx_buf_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					UINT tx_chCnt)
{
	UINT chInx;

	DBGPR("-->DWC_ETH_QOS_tx_buf_free_mem: tx_chCnt = %d\n", tx_chCnt);

	for (chInx = 0; chInx < tx_chCnt; chInx++) {
		if(!pdata->tx_dma_ch_for_host[chInx])
			continue;
		/* free TX buffer */
		if (GET_TX_BUF_PTR(chInx, 0)) {
			kfree(GET_TX_BUF_PTR(chInx, 0));
			GET_TX_BUF_PTR(chInx, 0) = NULL;
		}
	}

	DBGPR("<--DWC_ETH_QOS_tx_buf_free_mem\n");
}

/*!
* \brief API to free the receive descriptor wrapper buffer memory.
*
* \details This function is used to free the receive descriptor wrapper buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void DWC_ETH_QOS_rx_buf_free_mem(struct DWC_ETH_QOS_prv_data *pdata,
					UINT rx_chCnt)
{
	UINT chInx = 0;

	DBGPR("-->DWC_ETH_QOS_rx_buf_free_mem: rx_chCnt = %d\n", rx_chCnt);

	for (chInx = 0; chInx < rx_chCnt; chInx++) {
		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;
		if (GET_RX_BUF_PTR(chInx, 0)) {
			kfree(GET_RX_BUF_PTR(chInx, 0));
			GET_RX_BUF_PTR(chInx, 0) = NULL;
		}
	}

	DBGPR("<--DWC_ETH_QOS_rx_buf_free_mem\n");
}

/*!
* \brief Assigns the network and tcp header pointers
*
* \details This function gets the ip and tcp header pointers of the packet
* in the skb and assigns them to the corresponding arguments passed to the
* function. It also sets some flags indicating that the packet to be receieved
* is an ipv4 packet and that the protocol is tcp.
*
* \param[in] *skb - pointer to the sk buffer,
* \param[in] **iph - pointer to be pointed to the ip header,
* \param[in] **tcph - pointer to be pointed to the tcp header,
* \param[in] *hdr_flags - flags to be set
* \param[in] *pdata - private data structure
*
* \return integer
*
* \retval -1 if the packet does not conform to ip protocol = TCP, else 0
*/

static int DWC_ETH_QOS_get_skb_hdr(struct sk_buff *skb, void **iph,
				   void **tcph, u64 *flags, void *ptr)
{
	struct DWC_ETH_QOS_prv_data *pdata = ptr;

	DBGPR("-->DWC_ETH_QOS_get_skb_hdr\n");

	if (!pdata->tcp_pkt)
		return -1;

	skb_reset_network_header(skb);
	skb_set_transport_header(skb, ip_hdrlen(skb));
	*iph = ip_hdr(skb);
	*tcph = tcp_hdr(skb);
	*flags = LRO_IPV4 | LRO_TCP;

	DBGPR("<--DWC_ETH_QOS_get_skb_hdr\n");

	return 0;
}


/*!
 * \brief api to handle tso
 *
 * \details This function is invoked by start_xmit functions. This function
 * will get all the tso details like MSS(Maximum Segment Size), packet header length,
 * packet pay load length and tcp header length etc if the given skb has tso
 * packet and store it in other wrapper tx structure for later usage.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] skb – pointer to socket buffer structure.
 *
 * \return integer
 *
 * \retval 1 on success, -ve no failure and 0 if not tso pkt
 * */

static int DWC_ETH_QOS_handle_tso(struct net_device *dev,
	struct sk_buff *skb)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR;
	int ret = 1;

	DBGPR("-->DWC_ETH_QOS_handle_tso\n");

	if (skb_is_gso(skb) == 0) {
		DBGPR("This is not a TSO/LSO/GSO packet\n");
        DBGPR("<--DWC_ETH_QOS_handle_tso\n");
		return 0;
	}

	DBGPR("Got TSO packet\n");

	if (skb_header_cloned(skb)) {
		ret = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (ret)
			return ret;
    }

	/* get TSO details */
	tx_pkt_features->mss = skb_shinfo(skb)->gso_size;
	tx_pkt_features->hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
	tx_pkt_features->pay_len = (skb->len - tx_pkt_features->hdr_len);
	tx_pkt_features->tcp_hdr_len = tcp_hdrlen(skb);

	DBGPR("mss         = %lu\n", tx_pkt_features->mss);
	DBGPR("hdr_len     = %lu\n", tx_pkt_features->hdr_len);
	DBGPR("pay_len     = %lu\n", tx_pkt_features->pay_len);
	DBGPR("tcp_hdr_len = %lu\n", tx_pkt_features->tcp_hdr_len);

	DBGPR("<--DWC_ETH_QOS_handle_tso\n");

	return ret;
}

/* returns 0 on success and -ve on failure */
static int DWC_ETH_QOS_map_non_page_buffs(struct DWC_ETH_QOS_prv_data *pdata,
				struct DWC_ETH_QOS_tx_buffer *buffer,
				struct DWC_ETH_QOS_tx_buffer *prev_buffer,
				struct sk_buff *skb,
				unsigned int offset,
				unsigned int size)
{
	DBGPR("-->DWC_ETH_QOS_map_non_page_buffs\n");

	if (size > DWC_ETH_QOS_MAX_DATA_PER_TX_BUF) {
		if (prev_buffer && !prev_buffer->dma2) {
			/* fill the first buffer pointer in prev_buffer->dma2 */
            NMSGPR_ERR("dma_map_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			prev_buffer->dma2 = dma_map_single((&pdata->pdev->dev),
							(skb->data + offset),
							DWC_ETH_QOS_MAX_DATA_PER_TX_BUF,
							DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), prev_buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}
			prev_buffer->len2 = DWC_ETH_QOS_MAX_DATA_PER_TX_BUF;
			prev_buffer->buf2_mapped_as_page = Y_FALSE;

			/* fill the second buffer pointer in buffer->dma */
            NMSGPR_ERR("dma_map_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma = dma_map_single((&pdata->pdev->dev),
						(skb->data + offset + DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						(size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}
			buffer->len = (size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF);
			buffer->buf1_mapped_as_page = Y_FALSE;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		} else {
			/* fill the first buffer pointer in buffer->dma */
            NMSGPR_ERR("dma_map_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma = dma_map_single((&pdata->pdev->dev),
					(skb->data + offset),
					DWC_ETH_QOS_MAX_DATA_PER_TX_BUF,
					DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}
			buffer->len = DWC_ETH_QOS_MAX_DATA_PER_TX_BUF;
			buffer->buf1_mapped_as_page = Y_FALSE;

			/* fill the second buffer pointer in buffer->dma2 */
            NMSGPR_ERR("dma_map_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma2 = dma_map_single((&pdata->pdev->dev),
					(skb->data + offset + DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
					(size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
					DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}
			buffer->len2 = (size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF);
			buffer->buf2_mapped_as_page = Y_FALSE;
		}
	} else {
		if (prev_buffer && !prev_buffer->dma2) {
			/* fill the first buffer pointer in prev_buffer->dma2 */
            NMSGPR_ERR("dma_map_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			prev_buffer->dma2 = dma_map_single((&pdata->pdev->dev),
						(skb->data + offset),
						size, DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), prev_buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}
			prev_buffer->len2 = size;
			prev_buffer->buf2_mapped_as_page = Y_FALSE;

			/* indicate current buffer struct is not used */
			buffer->dma = 0;
			buffer->len = 0;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		} else {
			/* fill the first buffer pointer in buffer->dma */
#ifdef NTN_TX_DATA_BUF_IN_SRAM
            		/* Allocate buffer in device memory */
    			buffer->dma_virt = dma_pool_alloc(pdata->tx_mem_pool, GFP_ATOMIC, &buffer->dma);
            		if (buffer->dma_virt == NULL) {
                		NMSGPR_ALERT( "failed to allocate dma memory\n");
                		return - ENOMEM;
            		}	

            		NDBGPR_L2("buffer->dma_virt = %#lx : buffer->dma = %#lx\n", (long unsigned int)buffer->dma_virt, (long unsigned int)buffer->dma);
            		memcpy(buffer->dma_virt, (skb->data + offset), size);
            		NDBGPR_L2("offset = %d, size = %d\n", offset, size);
   			
			buffer->dma_iommu = dma_map_single((&pdata->pdev->dev),
						buffer->dma_virt,
						size, DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), buffer->dma_iommu)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}

#else
			buffer->dma = dma_map_single((&pdata->pdev->dev),
						(skb->data + offset),
						size, DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev), buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return - ENOMEM;
			}

#endif 

			buffer->len = size;
			buffer->buf1_mapped_as_page = Y_FALSE;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		}
	}

	DBGPR("<--DWC_ETH_QOS_map_non_page_buffs\n");

	return 0;
}

/* returns 0 on success and -ve on failure */
static int DWC_ETH_QOS_map_page_buffs(struct DWC_ETH_QOS_prv_data *pdata,
			struct DWC_ETH_QOS_tx_buffer *buffer,
			struct DWC_ETH_QOS_tx_buffer *prev_buffer,
			struct skb_frag_struct *frag,
			unsigned int offset,
			unsigned int size)
{
	DBGPR("-->DWC_ETH_QOS_map_page_buffs\n");

	if (size > DWC_ETH_QOS_MAX_DATA_PER_TX_BUF) {
		if (!prev_buffer->dma2) {
			DBGPR("prev_buffer->dma2 is empty\n");
			/* fill the first buffer pointer in pre_buffer->dma2 */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			prev_buffer->dma2 =
				dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						(frag->page_offset + offset),
						DWC_ETH_QOS_MAX_DATA_PER_TX_BUF,
						DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						prev_buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			prev_buffer->len2 = DWC_ETH_QOS_MAX_DATA_PER_TX_BUF;
			prev_buffer->buf2_mapped_as_page = Y_TRUE;

			/* fill the second buffer pointer in buffer->dma */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma = dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						(frag->page_offset + offset + DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						(size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			buffer->len = (size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF);
			buffer->buf1_mapped_as_page = Y_TRUE;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		} else {
			/* fill the first buffer pointer in buffer->dma */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma = dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						(frag->page_offset + offset),
						DWC_ETH_QOS_MAX_DATA_PER_TX_BUF,
						DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			buffer->len = DWC_ETH_QOS_MAX_DATA_PER_TX_BUF;
			buffer->buf1_mapped_as_page = Y_TRUE;

			/* fill the second buffer pointer in buffer->dma2 */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma2 = dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						(frag->page_offset + offset + DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						(size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF),
						DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			buffer->len2 = (size - DWC_ETH_QOS_MAX_DATA_PER_TX_BUF);
			buffer->buf2_mapped_as_page = Y_TRUE;
		}
	} else {
		if (!prev_buffer->dma2) {
			DBGPR("prev_buffer->dma2 is empty\n");
			/* fill the first buffer pointer in pre_buffer->dma2 */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			prev_buffer->dma2 = dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						frag->page_offset,
						size, DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						prev_buffer->dma2)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			prev_buffer->len2 = size;
			prev_buffer->buf2_mapped_as_page = Y_TRUE;

			/* indicate current buffer struct is not used */
			buffer->dma = 0;
			buffer->len = 0;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		} else {
			/* fill the first buffer pointer in buffer->dma */
            NMSGPR_ERR("dma_map_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			buffer->dma = dma_map_page((&pdata->pdev->dev),
						frag->page.p,
						frag->page_offset,
						size, DMA_TO_DEVICE);
			if (dma_mapping_error((&pdata->pdev->dev),
						buffer->dma)) {
				NMSGPR_ALERT( "failed to do the dma map\n");
				return -ENOMEM;
			}
			buffer->len = size;
			buffer->buf1_mapped_as_page = Y_TRUE;
			buffer->dma2 = 0;
			buffer->len2 = 0;
		}
	}

	DBGPR("<--DWC_ETH_QOS_map_page_buffs\n");

	return 0;
}


/*!
 * \details This function is invoked by start_xmit functions. This function
 * will get the dma/physical address of the packet to be transmitted and
 * its length. All this information about the packet to be transmitted is
 * stored in private data structure and same is used later in the driver to
 * setup the descriptor for transmission.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] skb – pointer to socket buffer structure.
 *
 * \return unsigned int
 *
 * \retval count – number of packet to be programmed in the descriptor or
 * zero on failure.
 */

static unsigned int DWC_ETH_QOS_map_skb(struct net_device *dev,
					struct sk_buff *skb)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	UINT chInx = skb_get_queue_mapping(skb);
	struct DWC_ETH_QOS_tx_wrapper_descriptor *desc_data =
	    GET_TX_WRAPPER_DESC(chInx);
	struct DWC_ETH_QOS_tx_buffer *buffer =
	    GET_TX_BUF_PTR(chInx, desc_data->cur_tx);
	struct DWC_ETH_QOS_tx_buffer *prev_buffer = NULL;
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR;
	UINT varvlan_pkt;
	int index = (int)desc_data->cur_tx;
	unsigned int frag_cnt = skb_shinfo(skb)->nr_frags;
	unsigned int hdr_len = 0;
	unsigned int i;
	unsigned int count = 0, offset = 0, size;
	int len;
	int vartso_enable = 0;
	int ret;

	DBGPR("-->DWC_ETH_QOS_map_skb: cur_tx = %d, chInx = %u\n",
		desc_data->cur_tx, chInx);

	TX_PKT_FEATURES_PKT_ATTRIBUTES_TSO_ENABLE_Mlf_Rd(
		tx_pkt_features->pkt_attributes, vartso_enable);
	TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_Mlf_Rd(
			tx_pkt_features->pkt_attributes, varvlan_pkt);
	if (varvlan_pkt == 0x1) {
		DBGPR("Skipped preparing index %d "\
			"(VLAN Context descriptor)\n\n", index);
		INCR_TX_DESC_INDEX(index, 1);
		buffer = GET_TX_BUF_PTR(chInx, index);
	} else if ((vartso_enable == 0x1) && (desc_data->default_mss != tx_pkt_features->mss)) {
		/* keep space for CONTEXT descriptor in the RING */
		INCR_TX_DESC_INDEX(index, 1);
		buffer = GET_TX_BUF_PTR(chInx, index);
	}
#ifdef DWC_ETH_QOS_ENABLE_DVLAN
	if (pdata->via_reg_or_desc) {
		DBGPR("Skipped preparing index %d "\
			"(Double VLAN Context descriptor)\n\n", index);
		INCR_TX_DESC_INDEX(index, 1);
		buffer = GET_TX_BUF_PTR(chInx, index);
	}
#endif /* End of DWC_ETH_QOS_ENABLE_DVLAN */

	if (vartso_enable) {
		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		len = hdr_len;
	} else {
		len = (skb->len - skb->data_len);
	}

	DBGPR("skb->len - skb->data_len = %d, hdr_len = %d\n",
				len, hdr_len);
	while (len) {
		size = min(len, DWC_ETH_QOS_MAX_DATA_PER_TXD);

		buffer = GET_TX_BUF_PTR(chInx, index);
		ret = DWC_ETH_QOS_map_non_page_buffs(pdata, buffer,
						prev_buffer,
						skb, offset, size);
		if (ret < 0)
			goto err_out_dma_map_fail;

		len -= size;
		offset += size;
		prev_buffer = buffer;
		INCR_TX_DESC_INDEX(index, 1);
		count++;
	}

	/* Process remaining pay load in skb->data in case of TSO packet */
	if (vartso_enable) {
		len = ((skb->len - skb->data_len) - hdr_len);
		while (len > 0) {
			size = min(len, DWC_ETH_QOS_MAX_DATA_PER_TXD);

			buffer = GET_TX_BUF_PTR(chInx, index);
			ret = DWC_ETH_QOS_map_non_page_buffs(pdata, buffer,
							prev_buffer,
							skb, offset, size);
			if (ret < 0)
				goto err_out_dma_map_fail;

			len -= size;
			offset += size;
			if (buffer->dma != 0) {
				prev_buffer = buffer;
				INCR_TX_DESC_INDEX(index, 1);
				count++;
			}
		}
	}

	/* Process fragmented skb's */
	for (i = 0; i < frag_cnt; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];

		len = frag->size;
		offset = 0;
		while (len) {
			size = min(len, DWC_ETH_QOS_MAX_DATA_PER_TXD);

			buffer = GET_TX_BUF_PTR(chInx, index);
			ret = DWC_ETH_QOS_map_page_buffs(pdata, buffer,
							prev_buffer, frag,
							offset, size);
			if (ret < 0)
				goto err_out_dma_map_fail;

			len -= size;
			offset += size;
			if (buffer->dma != 0) {
				prev_buffer = buffer;
				INCR_TX_DESC_INDEX(index, 1);
				count++;
			}
		}
	}
	buffer->skb = skb;

	DBGPR("<--DWC_ETH_QOS_map_skb: buffer->dma = %#x\n",
	      (UINT) buffer->dma);

	return count;

 err_out_dma_map_fail:
	NMSGPR_ALERT( "Tx DMA map failed\n");

	for (; count > 0; count--) {
		DECR_TX_DESC_INDEX(index);
		buffer = GET_TX_BUF_PTR(chInx, index);
		DWC_ETH_QOS_unmap_tx_skb(pdata, buffer);
	}

	return 0;
}

/*!
* \brief API to release the skb.
*
* \details This function is called in *_tx_interrupt function to release
* the skb for the successfully transmited packets.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] buffer - pointer to *_tx_buffer structure
*
* \return void
*/

static void DWC_ETH_QOS_unmap_tx_skb(struct DWC_ETH_QOS_prv_data *pdata,
				     struct DWC_ETH_QOS_tx_buffer *buffer)
{
	DBGPR("-->DWC_ETH_QOS_unmap_tx_skb\n");

	if (buffer->dma) {
		if (buffer->buf1_mapped_as_page == Y_TRUE){
            		NMSGPR_ERR("dma_unmap_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			dma_unmap_page((&pdata->pdev->dev), buffer->dma,
				       buffer->len, DMA_TO_DEVICE);
		}else{
#ifdef NTN_TX_DATA_BUF_IN_SRAM
            		NDBGPR_L2("unmap dma coherent memory\n");
			dma_unmap_single((&pdata->pdev->dev), buffer->dma_iommu,
					 buffer->len, DMA_TO_DEVICE);
    	                NDBGPR_L2("virt_adrs = 0x%lx : buffer->dma = 0x%x\n", (long unsigned int)buffer->dma_virt, (unsigned int)buffer->dma);

                        dma_pool_free(pdata->tx_mem_pool, buffer->dma_virt, buffer->dma);
            		NDBGPR_L2("Free dma coherent memory done\n");

            		buffer->dma_iommu = 0;
            		buffer->dma_virt = 0;
#else 
			dma_unmap_single((&pdata->pdev->dev), buffer->dma,
					 buffer->len, DMA_TO_DEVICE);
#endif
        	}

		buffer->dma = 0;
		buffer->len = 0;
	}

	if (buffer->dma2) {
		if (buffer->buf2_mapped_as_page == Y_TRUE){
           		NMSGPR_ERR("dma_unmap_page: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			dma_unmap_page((&pdata->pdev->dev), buffer->dma2,
					buffer->len2, DMA_TO_DEVICE);
		}else{
            		NMSGPR_ERR("dma_unmap_single: %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
			dma_unmap_single((&pdata->pdev->dev), buffer->dma2,
					buffer->len2, DMA_TO_DEVICE);
		}
		buffer->dma2 = 0;
		buffer->len2 = 0;
	}


	if (buffer->skb != NULL) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}

	DBGPR("<--DWC_ETH_QOS_unmap_tx_skb\n");
}

/*!
 * \details This function is invoked by other function for releasing the socket
 * buffer which are received by device and passed to upper layer.
 *
 * \param[in] pdata – pointer to private device structure.
 * \param[in] buffer – pointer to rx wrapper buffer structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_unmap_rx_skb(struct DWC_ETH_QOS_prv_data *pdata,
				     struct DWC_ETH_QOS_rx_buffer *buffer)
{
	DBGPR("-->DWC_ETH_QOS_unmap_rx_skb\n");

	/* unmap the first buffer */
	if (buffer->dma) {
#ifdef NTN_RX_DATA_BUF_IN_SRAM
			dma_unmap_single((&pdata->pdev->dev), buffer->dma_iommu,
					 pdata->rx_buffer_len, DMA_FROM_DEVICE);
    	                NDBGPR_L2("2 virt_adrs = 0x%lx : buffer->dma = 0x%x\n", (long unsigned int)buffer->skb->data, (unsigned int)buffer->dma);

                        dma_pool_free(pdata->rx_mem_pool, buffer->skb->data, buffer->dma);
            		buffer->dma_iommu = 0;
            		buffer->skb->data = buffer->skb_temp_data_ptr;
#else
			dma_unmap_single(&pdata->pdev->dev, buffer->dma,
					 pdata->rx_buffer_len, DMA_FROM_DEVICE);
#endif
		buffer->dma = 0;
	}

	/* unmap the second buffer */
	if (buffer->dma2) {
        	NMSGPR_ERR("dma_unmap_page %s: %s: %d\n", __FILE__, __FUNCTION__, __LINE__);
		dma_unmap_page(&pdata->pdev->dev, buffer->dma2,
				PAGE_SIZE, DMA_FROM_DEVICE);
		buffer->dma2 = 0;
	}

	if (buffer->skb) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}

	DBGPR("<--DWC_ETH_QOS_unmap_rx_skb\n");
}

/*!
* \brief API to re-allocate the new skb to rx descriptors.
*
* \details This function is used to re-allocate & re-assign the new skb to
* receive descriptors from which driver has read the data. Also ownership bit
* and other bits are reset so that device can reuse the descriptors.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void DWC_ETH_QOS_re_alloc_skb(struct DWC_ETH_QOS_prv_data *pdata,
				UINT chInx)
{
	int i;
	struct DWC_ETH_QOS_rx_wrapper_descriptor *desc_data =
	    GET_RX_WRAPPER_DESC(chInx);
	struct DWC_ETH_QOS_rx_buffer *buffer = NULL;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	int tail_idx;

	DBGPR("-->DWC_ETH_QOS_re_alloc_skb: desc_data->skb_realloc_idx = %d "\
		" chInx = %u\n", desc_data->skb_realloc_idx, chInx);

	for (i = 0; i < desc_data->dirty_rx; i++) {
		buffer = GET_RX_BUF_PTR(chInx, desc_data->skb_realloc_idx);
		/* allocate skb & assign to each desc */
		if (pdata->alloc_rx_buf(chInx, pdata, buffer, GFP_ATOMIC)) {
			NMSGPR_ALERT( "Failed to re allocate skb\n");
			pdata->xstats.q_re_alloc_rx_buf_failed[chInx]++;
			break;
		}

		wmb();
		hw_if->rx_desc_reset(desc_data->skb_realloc_idx, pdata,
				     buffer->inte, chInx);
		INCR_RX_DESC_INDEX(desc_data->skb_realloc_idx, 1);
	}
	tail_idx = desc_data->skb_realloc_idx;
	DECR_RX_DESC_INDEX(tail_idx);
	hw_if->update_rx_tail_ptr(chInx, GET_RX_DESC_DMA_ADDR(chInx, tail_idx),pdata);
	desc_data->dirty_rx = 0;

	NDBGPR_TS1("RX Ch %d : %s \n", chInx, __FUNCTION__);
	DBGPR("<--DWC_ETH_QOS_re_alloc_skb\n");

	return;
}

/*!
* \brief API to initialize the function pointers.
*
* \details This function is called in probe to initialize all the function
* pointers which are used in other functions to manage edscriptors.
*
* \param[in] desc_if - pointer to desc_if_struct structure.
*
* \return void.
*/

void DWC_ETH_QOS_init_function_ptrs_desc(struct desc_if_struct *desc_if)
{

	DBGPR("-->DWC_ETH_QOS_init_function_ptrs_desc\n");

	desc_if->alloc_queue_struct = DWC_ETH_QOS_alloc_queue_struct;
	desc_if->free_queue_struct = DWC_ETH_QOS_free_queue_struct;
	desc_if->alloc_buff_and_desc = allocate_buffer_and_desc;
	desc_if->realloc_skb = DWC_ETH_QOS_re_alloc_skb;
	desc_if->unmap_rx_skb = DWC_ETH_QOS_unmap_rx_skb;
	desc_if->unmap_tx_skb = DWC_ETH_QOS_unmap_tx_skb;
	desc_if->map_tx_skb = DWC_ETH_QOS_map_skb;
	desc_if->tx_free_mem = DWC_ETH_QOS_tx_free_mem;
	desc_if->rx_free_mem = DWC_ETH_QOS_rx_free_mem;
	desc_if->wrapper_tx_desc_init = DWC_ETH_QOS_wrapper_tx_descriptor_init;
	desc_if->wrapper_tx_desc_init_single_q =
	    DWC_ETH_QOS_wrapper_tx_descriptor_init_single_q;
	desc_if->wrapper_rx_desc_init = DWC_ETH_QOS_wrapper_rx_descriptor_init;
	desc_if->wrapper_rx_desc_init_single_q =
	    DWC_ETH_QOS_wrapper_rx_descriptor_init_single_q;

	desc_if->rx_skb_free_mem = DWC_ETH_QOS_rx_skb_free_mem;
	desc_if->rx_skb_free_mem_single_q = DWC_ETH_QOS_rx_skb_free_mem_single_q;
	desc_if->tx_skb_free_mem = DWC_ETH_QOS_tx_skb_free_mem;
	desc_if->tx_skb_free_mem_single_q = DWC_ETH_QOS_tx_skb_free_mem_single_q;

	desc_if->handle_tso = DWC_ETH_QOS_handle_tso;

	DBGPR("<--DWC_ETH_QOS_init_function_ptrs_desc\n");
}
