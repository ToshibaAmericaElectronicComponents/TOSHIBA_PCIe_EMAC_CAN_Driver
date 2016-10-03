/* ============================================================================
* COPYRIGHT � 2015
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
 *      2-March-2016 : Initial 
 *      20-May-2016  : Changes made to Support for multiple Neutrino device
 *                     connection
 *      3-Jun-2016   : New statistics added to Ethtool for EMAC TX/RX DMA
 *                     Descriptor Status & M3 MSI Generation Statistics (SRAM).
 *                     Enabled WDT in INTC and added a counter in M3
 *                     Firmware(FW_OSLESS) for WDT half expiry interrupts.
 */

/*!@file: DWC_ETH_QOS_ethtool.c
 * @brief: Driver functions.
 */
#include "DWC_ETH_QOS_yheader.h"
#include "DWC_ETH_QOS_ethtool.h"

struct DWC_ETH_QOS_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

/* HW extra status */
#define DWC_ETH_QOS_EXTRA_STAT(m) \
	{#m, FIELD_SIZEOF(struct DWC_ETH_QOS_extra_stats, m), \
	offsetof(struct DWC_ETH_QOS_prv_data, xstats.m)}

static const struct DWC_ETH_QOS_stats DWC_ETH_QOS_gstrings_stats[] = {
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[0]),
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[1]),
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[2]),
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[3]),
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[4]),
	DWC_ETH_QOS_EXTRA_STAT(q_re_alloc_rx_buf_failed[5]),

	/* Tx/Rx IRQ error info */
	DWC_ETH_QOS_EXTRA_STAT(tx_process_stopped_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(tx_process_stopped_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(tx_process_stopped_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(tx_process_stopped_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(tx_process_stopped_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_process_stopped_irq_n[5]),
	DWC_ETH_QOS_EXTRA_STAT(tx_buf_unavailable_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(tx_buf_unavailable_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(tx_buf_unavailable_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(tx_buf_unavailable_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(tx_buf_unavailable_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_buf_unavailable_irq_n[5]),
	DWC_ETH_QOS_EXTRA_STAT(rx_watchdog_irq_n),
	DWC_ETH_QOS_EXTRA_STAT(fatal_bus_error_irq_n),
	DWC_ETH_QOS_EXTRA_STAT(pmt_irq_n),
	/* Tx/Rx IRQ Events */
	DWC_ETH_QOS_EXTRA_STAT(tx_normal_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(tx_normal_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(tx_normal_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(tx_normal_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(tx_normal_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(rx_normal_irq_n[5]),
	DWC_ETH_QOS_EXTRA_STAT(napi_poll_n),
	DWC_ETH_QOS_EXTRA_STAT(tx_clean_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(tx_clean_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(tx_clean_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(tx_clean_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(tx_clean_n[4]),
	/* EEE */
	DWC_ETH_QOS_EXTRA_STAT(tx_path_in_lpi_mode_irq_n),
	DWC_ETH_QOS_EXTRA_STAT(tx_path_exit_lpi_mode_irq_n),
	DWC_ETH_QOS_EXTRA_STAT(rx_path_in_lpi_mode_irq_n),
	DWC_ETH_QOS_EXTRA_STAT(rx_path_exit_lpi_mode_irq_n),
	/* Tx/Rx frames */
	DWC_ETH_QOS_EXTRA_STAT(tx_pkt_n),
	DWC_ETH_QOS_EXTRA_STAT(rx_pkt_n),
	DWC_ETH_QOS_EXTRA_STAT(tx_vlan_pkt_n),
	DWC_ETH_QOS_EXTRA_STAT(rx_vlan_pkt_n),
	DWC_ETH_QOS_EXTRA_STAT(tx_timestamp_captured_n),
	DWC_ETH_QOS_EXTRA_STAT(rx_timestamp_captured_n),
	DWC_ETH_QOS_EXTRA_STAT(tx_tso_pkt_n),

	/* Tx/Rx frames per channels/queues */
	DWC_ETH_QOS_EXTRA_STAT(q_tx_pkt_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(q_tx_pkt_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(q_tx_pkt_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(q_tx_pkt_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(q_tx_pkt_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[0]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[1]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[2]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[3]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[4]),
	DWC_ETH_QOS_EXTRA_STAT(q_rx_pkt_n[5]),

	/* TX DMA Descriptors Status for all channels [0-4] */
	DWC_ETH_QOS_EXTRA_STAT(txch_status[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_status[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_status[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_status[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_status[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_control[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_control[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_control[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_control[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_control[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_intrmask[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_intrmask[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_intrmask[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_intrmask[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_intrmask[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_haddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_haddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_haddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_haddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_haddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_laddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_laddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_laddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_laddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_list_laddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_ring_len[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_ring_len[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_ring_len[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_ring_len[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_ring_len[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_curr[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_curr[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_curr[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_curr[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_curr[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_tail[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_tail[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_tail[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_tail[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_tail[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_haddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_haddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_haddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_haddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_haddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_laddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_laddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_laddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_laddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_desc_buf_laddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_cur_tx[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_cur_tx[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_cur_tx[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_cur_tx[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_cur_tx[4]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_dirty_tx[0]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_dirty_tx[1]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_dirty_tx[2]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_dirty_tx[3]),
	DWC_ETH_QOS_EXTRA_STAT(txch_sw_dirty_tx[4]),

	/* RX DMA Descriptors Status for all channels [0-5] */
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_status[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_control[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_intrmask[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_haddr[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_list_laddr[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_ring_len[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_curr[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_tail[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_haddr[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[0]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_desc_buf_laddr[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[0]),	/* desc_data->cur_rx:  Software Checks this index for RX packet availability */
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_cur_rx[5]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[0]), 	/* desc_data->dirty_rx:Software tracks skb counter for recycling */
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[1]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[2]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[3]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[4]),
	DWC_ETH_QOS_EXTRA_STAT(rxch_sw_dirty_rx[5]),

	/*M3 Firmware Counters*/
	DWC_ETH_QOS_EXTRA_STAT(m3_tick_cnt),
	DWC_ETH_QOS_EXTRA_STAT(m3_wdt_cnt),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_txch[0]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_txch[1]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_txch[2]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_txch[3]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_txch[4]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[0]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[1]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[2]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[3]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[4]),
	DWC_ETH_QOS_EXTRA_STAT(m3_msi_rxch[5]),
};
#define DWC_ETH_QOS_EXTRA_STAT_LEN ARRAY_SIZE(DWC_ETH_QOS_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define DWC_ETH_QOS_MMC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct DWC_ETH_QOS_mmc_counters, m),	\
	offsetof(struct DWC_ETH_QOS_prv_data, mmc.m)}

static const struct DWC_ETH_QOS_stats DWC_ETH_QOS_mmc[] = {
	/* MMC TX counters */
	DWC_ETH_QOS_MMC_STAT(mmc_tx_octetcount_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_framecount_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_broadcastframe_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_multicastframe_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_64_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_unicast_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_multicast_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_broadcast_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_underflow_error),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_singlecol_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_multicol_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_deferred),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_latecol),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_exesscol),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_carrier_error),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_octetcount_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_framecount_g),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_excessdef),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_pause_frame),
	DWC_ETH_QOS_MMC_STAT(mmc_tx_vlan_frame_g),

	/* MMC RX counters */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_framecount_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_octetcount_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_octetcount_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_broadcastframe_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_multicastframe_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_crc_errror),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_align_error),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_run_error),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_jabber_error),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_undersize_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_oversize_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_64_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_unicast_g),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_length_error),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_outofrangetype),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_pause_frames),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_fifo_overflow),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_vlan_frames_gb),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_watchdog_error),

	/* IPC */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipc_intr_mask),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipc_intr),

	/* IPv4 */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_gd),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_hderr),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_nopay),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_frag),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_udsbl),

	/* IPV6 */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_gd_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_nopay_octets),

	/* Protocols */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_udp_gd),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_udp_err),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_tcp_gd),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_tcp_err),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_icmp_gd),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_icmp_err),

	/* IPv4 */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_gd_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_frag_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv4_udsbl_octets),

	/* IPV6 */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_gd),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_hderr),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_ipv6_nopay),

	/* Protocols */
	DWC_ETH_QOS_MMC_STAT(mmc_rx_udp_gd_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_udp_err_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_tcp_gd_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_tcp_err_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_icmp_gd_octets),
	DWC_ETH_QOS_MMC_STAT(mmc_rx_icmp_err_octets),
};
#define DWC_ETH_QOS_MMC_STATS_LEN ARRAY_SIZE(DWC_ETH_QOS_mmc)

static const struct ethtool_ops DWC_ETH_QOS_ethtool_ops = {
	.get_pauseparam = DWC_ETH_QOS_get_pauseparam,
	.set_pauseparam = DWC_ETH_QOS_set_pauseparam,
	.get_settings = DWC_ETH_QOS_getsettings,
	.set_settings = DWC_ETH_QOS_setsettings,
	.get_wol = DWC_ETH_QOS_get_wol,
	.set_wol = DWC_ETH_QOS_set_wol,
	.get_coalesce = DWC_ETH_QOS_get_coalesce,
	.set_coalesce = DWC_ETH_QOS_set_coalesce,
	.get_ethtool_stats = DWC_ETH_QOS_get_ethtool_stats,
	.get_strings = DWC_ETH_QOS_get_strings,
	.get_sset_count = DWC_ETH_QOS_get_sset_count,
    .get_ts_info = DWC_ETH_QOS_get_ts_info,
};

struct ethtool_ops *DWC_ETH_QOS_get_ethtool_ops(void)
{
	return (struct ethtool_ops *)&DWC_ETH_QOS_ethtool_ops;
}


/*!
 * \details This function is invoked by kernel when user request to get the
 * pause parameters through standard ethtool command.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] Pause � pointer to ethtool_pauseparam structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_get_pauseparam(struct net_device *dev,
				       struct ethtool_pauseparam *pause)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct phy_device *phydev = pdata->phydev;
	unsigned int data;

	DBGPR("-->DWC_ETH_QOS_get_pauseparam\n");

	pause->rx_pause = 0;
	pause->tx_pause = 0;

	if (pdata->hw_feat.pcs_sel) {
		pause->autoneg = 1;
		data = hw_if->get_an_adv_pause_param(pdata);
		if (!(data == 1) || !(data == 2))
			return;
	} else {
		pause->autoneg = pdata->phydev->autoneg;

		/* return if PHY doesn't support FLOW ctrl */
		if (!(phydev->supported & SUPPORTED_Pause) ||
		    !(phydev->supported & SUPPORTED_Asym_Pause))
			return;
	}

	if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_RX) ==
	    DWC_ETH_QOS_FLOW_CTRL_RX)
		pause->rx_pause = 1;

	if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_TX) ==
	    DWC_ETH_QOS_FLOW_CTRL_TX)
		pause->tx_pause = 1;

	DBGPR("<--DWC_ETH_QOS_get_pauseparam\n");
}

/*!
 * \details This function is invoked by kernel when user request to set the
 * pause parameters through standard ethtool command.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] pause � pointer to ethtool_pauseparam structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

static int DWC_ETH_QOS_set_pauseparam(struct net_device *dev,
				      struct ethtool_pauseparam *pause)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct phy_device *phydev = pdata->phydev;
	int new_pause = DWC_ETH_QOS_FLOW_CTRL_OFF;
	unsigned int data;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_set_pauseparam: "\
	      "autoneg = %d tx_pause = %d rx_pause = %d\n",
	      pause->autoneg, pause->tx_pause, pause->rx_pause);

	/* return if PHY doesn't support FLOW ctrl */
	if (pdata->hw_feat.pcs_sel) {
		data = hw_if->get_an_adv_pause_param(pdata);
		if (!(data == 1) || !(data == 2))
			return -EINVAL;
	} else {
		if (!(phydev->supported & SUPPORTED_Pause) ||
			!(phydev->supported & SUPPORTED_Asym_Pause))
			return -EINVAL;
	}

	if (pause->rx_pause)
		new_pause |= DWC_ETH_QOS_FLOW_CTRL_RX;
	if (pause->tx_pause)
		new_pause |= DWC_ETH_QOS_FLOW_CTRL_TX;

	if (new_pause == pdata->flow_ctrl && !pause->autoneg)
		return -EINVAL;

	pdata->flow_ctrl = new_pause;

	if (pdata->hw_feat.pcs_sel) {
		DWC_ETH_QOS_configure_flow_ctrl(pdata);
	} else {
		phydev->autoneg = pause->autoneg;
		if (phydev->autoneg) {
			if (netif_running(dev))
				ret = phy_start_aneg(phydev);
		} else {
			DWC_ETH_QOS_configure_flow_ctrl(pdata);
		}
	}

	DBGPR("<--DWC_ETH_QOS_set_pauseparam\n");

	return ret;
}

void DWC_ETH_QOS_configure_flow_ctrl(struct DWC_ETH_QOS_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qInx;

	DBGPR("-->DWC_ETH_QOS_configure_flow_ctrl\n");

	if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_RX) ==
	    DWC_ETH_QOS_FLOW_CTRL_RX) {
		hw_if->enable_rx_flow_ctrl(pdata);
	} else {
		hw_if->disable_rx_flow_ctrl(pdata);
	}

	/* As ethtool does not provide queue level configuration
	   Tx flow control is disabled/enabled for all transmit queues */
	if ((pdata->flow_ctrl & DWC_ETH_QOS_FLOW_CTRL_TX) ==
	    DWC_ETH_QOS_FLOW_CTRL_TX) {
		for (qInx = 0; qInx < NTN_TX_QUEUE_CNT; qInx++){
			if(!pdata->tx_q_for_host[qInx])
				continue;
			hw_if->enable_tx_flow_ctrl(qInx, pdata);
		}
	} else {
		for (qInx = 0; qInx < NTN_TX_QUEUE_CNT; qInx++){
			if(!pdata->tx_q_for_host[qInx])
				continue;
			hw_if->disable_tx_flow_ctrl(qInx, pdata);
		}
	}

	pdata->oldflow_ctrl = pdata->flow_ctrl;

	DBGPR("<--DWC_ETH_QOS_configure_flow_ctrl\n");
}

/*!
 * \details This function is invoked by kernel when user request to get the
 * various device settings through standard ethtool command. This function
 * support to get the PHY related settings like link status, interface type,
 * auto-negotiation parameters and pause parameters etc.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] cmd � pointer to ethtool_cmd structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */
#define SPEED_UNKNOWN -1
#define DUPLEX_UNKNOWN 0xff
static int DWC_ETH_QOS_getsettings(struct net_device *dev,
				   struct ethtool_cmd *cmd)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int pause, duplex;
	unsigned int lp_pause, lp_duplex;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_getsettings\n");

	if (pdata->hw_feat.pcs_sel) {
		if (!pdata->pcs_link) {
			ethtool_cmd_speed_set(cmd, SPEED_UNKNOWN);
			cmd->duplex = DUPLEX_UNKNOWN;
			return 0;
		}
		ethtool_cmd_speed_set(cmd, pdata->pcs_speed);
		cmd->duplex = pdata->pcs_duplex;

		pause = hw_if->get_an_adv_pause_param(pdata);
		duplex = hw_if->get_an_adv_duplex_param(pdata);
		lp_pause = hw_if->get_lp_an_adv_pause_param(pdata);
		lp_duplex = hw_if->get_lp_an_adv_duplex_param(pdata);

		if (pause == 1)
			cmd->advertising |= ADVERTISED_Pause;
		if (pause == 2)
			cmd->advertising |= ADVERTISED_Asym_Pause;
		/* MAC always supports Auto-negotiation */
		cmd->autoneg = ADVERTISED_Autoneg;
		cmd->supported |= SUPPORTED_Autoneg;
		cmd->advertising |= ADVERTISED_Autoneg;

		if (duplex) {
			cmd->supported |= (SUPPORTED_1000baseT_Full |
				SUPPORTED_100baseT_Full |
				SUPPORTED_10baseT_Full);
			cmd->advertising |= (ADVERTISED_1000baseT_Full |
				ADVERTISED_100baseT_Full |
				ADVERTISED_10baseT_Full);
		} else {
			cmd->supported |= (SUPPORTED_1000baseT_Half |
				SUPPORTED_100baseT_Half |
				SUPPORTED_10baseT_Half);
			cmd->advertising |= (ADVERTISED_1000baseT_Half |
				ADVERTISED_100baseT_Half |
				ADVERTISED_10baseT_Half);
		}

		/* link partner features */
		cmd->lp_advertising |= ADVERTISED_Autoneg;
		if (lp_pause == 1)
			cmd->lp_advertising |= ADVERTISED_Pause;
		if (lp_pause == 2)
			cmd->lp_advertising |= ADVERTISED_Asym_Pause;

		if (lp_duplex)
			cmd->lp_advertising |= (ADVERTISED_1000baseT_Full |
				ADVERTISED_100baseT_Full |
				ADVERTISED_10baseT_Full);
		else
			cmd->lp_advertising |= (ADVERTISED_1000baseT_Half |
				ADVERTISED_100baseT_Half |
				ADVERTISED_10baseT_Half);

		cmd->port = PORT_OTHER;
	} else {
		if (pdata->phydev == NULL) {
			NMSGPR_ALERT( "%s: PHY is not registered\n", dev->name);
			return -ENODEV;
		}

		if (!netif_running(dev)) {
			NMSGPR_ALERT( "%s: interface is disabled: we cannot track "\
			       "link speed / duplex settings\n", dev->name);
			return -EBUSY;
		}

		cmd->transceiver = XCVR_EXTERNAL;

		spin_lock_irq(&pdata->lock);
		ret = phy_ethtool_gset(pdata->phydev, cmd);
		spin_unlock_irq(&pdata->lock);
	}

	DBGPR("<--DWC_ETH_QOS_getsettings\n");

	return ret;
}

/*!
 * \details This function is invoked by kernel when user request to set the
 * various device settings through standard ethtool command. This function
 * support to set the PHY related settings like link status, interface type,
 * auto-negotiation parameters and pause parameters etc.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] cmd � pointer to ethtool_cmd structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

static int DWC_ETH_QOS_setsettings(struct net_device *dev,
				   struct ethtool_cmd *cmd)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int speed;
	//unsigned int pause, duplex, speed;
	//unsigned int lp_pause, lp_duplex;
	int ret = 0;

	NMSGPR_ALERT( "-->DWC_ETH_QOS_setsettings\n");

	if (pdata->hw_feat.pcs_sel) {
		speed = ethtool_cmd_speed(cmd);

		/* verify the settings we care about */
		if ((cmd->autoneg != AUTONEG_ENABLE) &&
			(cmd->autoneg != AUTONEG_DISABLE))
			return -EINVAL;
/*
		if ((cmd->autoneg == AUTONEG_ENABLE) &&
			(cmd->advertising == 0))
			return -EINVAL;
		if ((cmd->autoneg == AUTONEG_DISABLE) &&
			(speed != SPEED_1000 &&
			 speed != SPEED_100 &&
			 speed != SPEED_10) ||
			(cmd->duplex != DUPLEX_FULL &&
			 cmd->duplex != DUPLEX_HALF))
			 return -EINVAL;
*/
		spin_lock_irq(&pdata->lock);
		if (cmd->autoneg == AUTONEG_ENABLE)
			hw_if->control_an(1, 1, pdata);
		else
			hw_if->control_an(0, 0, pdata);
		spin_unlock_irq(&pdata->lock);
	} else {
		spin_lock_irq(&pdata->lock);
		ret = phy_ethtool_sset(pdata->phydev, cmd);
		spin_unlock_irq(&pdata->lock);
	}

	NMSGPR_ALERT( "<--DWC_ETH_QOS_setsettings\n");

	return ret;
}

/*!
 * \details This function is invoked by kernel when user request to get report
 * whether wake-on-lan is enable or not.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] wol � pointer to ethtool_wolinfo structure.
 *
 * \return void
 */

static void DWC_ETH_QOS_get_wol(struct net_device *dev,
				struct ethtool_wolinfo *wol)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);

	DBGPR("-->DWC_ETH_QOS_get_wol\n");

	wol->supported = 0;
	spin_lock_irq(&pdata->lock);
	if (device_can_wakeup(&pdata->pdev->dev)) {
		if (pdata->hw_feat.mgk_sel)
			wol->supported |= WAKE_MAGIC;
		if (pdata->hw_feat.rwk_sel)
			wol->supported |= WAKE_UCAST;
		wol->wolopts = pdata->wolopts;
	}
	spin_unlock_irq(&pdata->lock);

	DBGPR("<--DWC_ETH_QOS_get_wol\n");

	return;
}

/*!
 * \details This function is invoked by kernel when user request to set
 * pmt parameters for remote wakeup or magic wakeup
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] wol � pointer to ethtool_wolinfo structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

static int DWC_ETH_QOS_set_wol(struct net_device *dev,
			       struct ethtool_wolinfo *wol)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;
	int ret = 0;

	DBGPR("-->DWC_ETH_QOS_set_wol\n");

	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame. */
	if (!pdata->hw_feat.mgk_sel)
		wol->wolopts &= ~WAKE_MAGIC;
	if (!pdata->hw_feat.rwk_sel)
		wol->wolopts &= ~WAKE_UCAST;

	if (!device_can_wakeup(&pdata->pdev->dev))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		NMSGPR_ALERT( "Wakeup enable\n");
		device_set_wakeup_enable(&pdata->pdev->dev, 1);
		enable_irq_wake(pdata->irq_number);
	} else {
		NMSGPR_ALERT( "Wakeup disable\n");
		device_set_wakeup_enable(&pdata->pdev->dev, 0);
		disable_irq_wake(pdata->irq_number);
	}

	spin_lock_irq(&pdata->lock);
	pdata->wolopts = wol->wolopts;
	spin_unlock_irq(&pdata->lock);

	DBGPR("<--DWC_ETH_QOS_set_wol\n");

	return ret;
}

u32 DWC_ETH_QOS_usec2riwt(u32 usec, struct DWC_ETH_QOS_prv_data *pdata)
{
	u32 ret = 0;

	DBGPR("-->DWC_ETH_QOS_usec2riwt\n");

	/* Eg:
	 * System clock is 62.5MHz, each clock cycle would then be 16ns
	 * For value 0x1 in watchdog timer, device would wait for 256
	 * clock cycles,
	 * ie, (16ns x 256) => 4.096us (rounding off to 4us)
	 * So formula with above values is,
	 * ret = usec/4 */

	ret = (usec * (DWC_ETH_QOS_SYSCLOCK/1000000))/256;

	DBGPR("<--DWC_ETH_QOS_usec2riwt\n");

	return ret;
}

static u32 DWC_ETH_QOS_riwt2usec(u32 riwt, struct DWC_ETH_QOS_prv_data *pdata)
{
	u32 ret = 0;

	DBGPR("-->DWC_ETH_QOS_riwt2usec\n");

	/* using formula from 'DWC_ETH_QOS_usec2riwt' */
	ret = (riwt * 256)/(DWC_ETH_QOS_SYSCLOCK/1000000);

	DBGPR("<--DWC_ETH_QOS_riwt2usec\n");

	return ret;
}

/*!
 * \details This function is invoked by kernel when user request to get
 * interrupt coalescing parameters. As coalescing parameters are same
 * for all the channels, so this function will get coalescing
 * details from channel zero and return.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] wol � pointer to ethtool_coalesce structure.
 *
 * \return int
 *
 * \retval 0
 */

static int DWC_ETH_QOS_get_coalesce(struct net_device *dev,
				    struct ethtool_coalesce *ec)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(0);

	DBGPR("-->DWC_ETH_QOS_get_coalesce\n");

	memset(ec, 0, sizeof(struct ethtool_coalesce));

	ec->rx_coalesce_usecs =
	    DWC_ETH_QOS_riwt2usec(rx_desc_data->rx_riwt, pdata);
	ec->rx_max_coalesced_frames = rx_desc_data->rx_coal_frames;

	DBGPR("<--DWC_ETH_QOS_get_coalesce\n");

	return 0;
}

/*!
 * \details This function is invoked by kernel when user request to set
 * interrupt coalescing parameters. This driver maintains same coalescing
 * parameters for all the channels, hence same changes will be applied to
 * all the channels.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] wol � pointer to ethtool_coalesce structure.
 *
 * \return int
 *
 * \retval zero on success and -ve number on failure.
 */

static int DWC_ETH_QOS_set_coalesce(struct net_device *dev,
				    struct ethtool_coalesce *ec)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	struct DWC_ETH_QOS_rx_wrapper_descriptor *rx_desc_data =
	    GET_RX_WRAPPER_DESC(0);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned int rx_riwt, rx_usec, local_use_riwt, chInx;

	DBGPR("-->DWC_ETH_QOS_set_coalesce\n");

	/* Check for not supported parameters  */
	if ((ec->rx_coalesce_usecs_irq) ||
	    (ec->rx_max_coalesced_frames_irq) || (ec->tx_coalesce_usecs_irq) ||
	    (ec->use_adaptive_rx_coalesce) || (ec->use_adaptive_tx_coalesce) ||
	    (ec->pkt_rate_low) || (ec->rx_coalesce_usecs_low) ||
	    (ec->rx_max_coalesced_frames_low) || (ec->tx_coalesce_usecs_high) ||
	    (ec->tx_max_coalesced_frames_low) || (ec->pkt_rate_high) ||
	    (ec->tx_coalesce_usecs_low) || (ec->rx_coalesce_usecs_high) ||
	    (ec->rx_max_coalesced_frames_high) ||
	    (ec->tx_max_coalesced_frames_irq) ||
	    (ec->stats_block_coalesce_usecs) ||
	    (ec->tx_max_coalesced_frames_high) || (ec->rate_sample_interval) ||
	    (ec->tx_coalesce_usecs) || (ec->tx_max_coalesced_frames))
		return -EOPNOTSUPP;

	/* both rx_coalesce_usecs and rx_max_coalesced_frames should
	 * be > 0 in order for coalescing to be active.
	 * */
	if ((ec->rx_coalesce_usecs <= 3) || (ec->rx_max_coalesced_frames <= 1))
		local_use_riwt = 0;
	else
		local_use_riwt = 1;

	NMSGPR_ALERT( "RX COALESCING is %s\n",
	       (local_use_riwt ? "ENABLED" : "DISABLED"));

	rx_riwt = DWC_ETH_QOS_usec2riwt(ec->rx_coalesce_usecs, pdata);

	/* Check the bounds of values for RX */
	if (rx_riwt > DWC_ETH_QOS_MAX_DMA_RIWT) {
		rx_usec = DWC_ETH_QOS_riwt2usec(DWC_ETH_QOS_MAX_DMA_RIWT,
		    pdata);
		NMSGPR_ALERT( "RX Coalesing is limited to %d usecs\n",
		       rx_usec);
		return -EINVAL;
	}
	if (ec->rx_max_coalesced_frames > RX_DESC_CNT) {
		NMSGPR_ALERT( "RX Coalesing is limited to %d frames\n",
		       DWC_ETH_QOS_RX_MAX_FRAMES);
		return -EINVAL;
	}
	if (rx_desc_data->rx_coal_frames != ec->rx_max_coalesced_frames
	    && netif_running(dev)) {
		NMSGPR_ALERT(
		 "Coalesce frame parameter can be changed only if interface is down\n");
		return -EINVAL;
	}
	/* The selected parameters are applied to all the
	 * receive queues equally, so all the queue configurations
	 * are in sync */
	for (chInx = 0; chInx < NTN_RX_DMA_CH_CNT; chInx++) {

		if(!pdata->rx_dma_ch_for_host[chInx])
			continue;
		
		rx_desc_data = GET_RX_WRAPPER_DESC(chInx);
		rx_desc_data->use_riwt = local_use_riwt;
		rx_desc_data->rx_riwt = rx_riwt;
		rx_desc_data->rx_coal_frames = ec->rx_max_coalesced_frames;
		hw_if->config_rx_watchdog(chInx, rx_desc_data->rx_riwt, pdata);
	}

	DBGPR("<--DWC_ETH_QOS_set_coalesce\n");

	return 0;
}


/*!
 * \details This function is invoked by kernel when user
 * requests to get the extended statistics about the device.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] data � pointer in which extended statistics
 *                   should be put.
 *
 * \return void
 */

static void DWC_ETH_QOS_get_ethtool_stats(struct net_device *dev,
	struct ethtool_stats *dummy, u64 *data)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	int i, j = 0;

	DBGPR("-->DWC_ETH_QOS_get_ethtool_stats\n");

	if (pdata->hw_feat.mmc_sel) {
		DWC_ETH_QOS_mmc_read(&pdata->mmc, pdata);

		for (i = 0; i < DWC_ETH_QOS_MMC_STATS_LEN; i++) {
			char *p = (char *)pdata +
					DWC_ETH_QOS_mmc[i].stat_offset;

			data[j++] = (DWC_ETH_QOS_mmc[i].sizeof_stat ==
				sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
		}
	}

	/* Update Extra Stats */
	DWC_ETH_QOS_desc_stats_read(pdata);
	DWC_ETH_QOS_m3fw_stats_read(pdata);

	for (i = 0; i < DWC_ETH_QOS_EXTRA_STAT_LEN; i++) {
		char *p = (char *)pdata +
				DWC_ETH_QOS_gstrings_stats[i].stat_offset;
		data[j++] = (DWC_ETH_QOS_gstrings_stats[i].sizeof_stat ==
				sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}

	DBGPR("<--DWC_ETH_QOS_get_ethtool_stats\n");
}


/*!
 * \details This function returns a set of strings that describe
 * the requested objects.
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] data � pointer in which requested string should be put.
 *
 * \return void
 */

static void DWC_ETH_QOS_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	int i;
	u8 *p = data;

	DBGPR("-->DWC_ETH_QOS_get_strings\n");

	switch(stringset) {
	case ETH_SS_STATS:
		if (pdata->hw_feat.mmc_sel) {
			for (i = 0; i < DWC_ETH_QOS_MMC_STATS_LEN; i++) {
				memcpy(p, DWC_ETH_QOS_mmc[i].stat_string,
					ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		}

		for (i = 0; i < DWC_ETH_QOS_EXTRA_STAT_LEN; i++) {
			memcpy(p, DWC_ETH_QOS_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		WARN_ON(1);
	}

	DBGPR("<--DWC_ETH_QOS_get_strings\n");
}


/*!
 * \details This function gets number of strings that @get_strings
 * will write.
 *
 * \param[in] dev � pointer to net device structure.
 *
 * \return int
 *
 * \retval +ve(>0) on success, 0 if that string is not
 * defined and -ve on failure.
 */

static int DWC_ETH_QOS_get_sset_count(struct net_device *dev, int sset)
{
	struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
	int len = 0;

	DBGPR("-->DWC_ETH_QOS_get_sset_count\n");

	switch(sset) {
	case ETH_SS_STATS:
		if (pdata->hw_feat.mmc_sel)
			len = DWC_ETH_QOS_MMC_STATS_LEN;
		len += DWC_ETH_QOS_EXTRA_STAT_LEN;
		break;
	default:
		len = -EOPNOTSUPP;
	}

	DBGPR("<--DWC_ETH_QOS_get_sset_count\n");

	return len;
}


/*!
 * \details This function gets the PHC index
 *
 * \param[in] dev � pointer to net device structure.
 * \param[in] ethtool_ts_info � pointer to ts info structure.
 *
 * \return int
 *
 * \retval +ve(>0) on success, 0 if that string is not
 * defined and -ve on failure.
 */

static int DWC_ETH_QOS_get_ts_info(struct net_device *dev,
                           struct ethtool_ts_info *info)
{
    struct DWC_ETH_QOS_prv_data *pdata = netdev_priv(dev);
    DBGPR("-->DWC_ETH_QOS_get_ts_info\n");
    info->phc_index = DWC_ETH_QOS_phc_index(pdata);
    NMSGPR_INFO("PHC index = %d\n", info->phc_index);
    DBGPR("<--DWC_ETH_QOS_get_ts_info\n");
    return 0;
}

