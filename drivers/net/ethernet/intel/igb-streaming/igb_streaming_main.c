/*************************************************************************//*
 * avb-streaming
 *
 * Copyright (C) 2014-2017 Renesas Electronics Corporation
 *			   ADIT GmbH
 *
 * Layout and stream handling derived from ravb-streaming_main.c by Renesas
 * Some register settings are inspired from the openAVb igb driver module.
 *
 * License        Dual MIT/GPLv2
 *
 * The contents of this file are subject to the MIT license as set out below.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * the GNU General Public License Version 2 ("GPL") in which case the provisions
 * of GPL are applicable instead of those above.
 *
 * If you wish to allow use of your version of this file only under the terms of
 * GPL, and not to allow others to use your version of this file under the terms
 * of the MIT license, indicate your decision by deleting the provisions above
 * and replace them with the notice and other provisions required by GPL as set
 * out in the file called "GPL-COPYING" included in this distribution. If you do
 * not delete the provisions above, a recipient may use your version of this
 * file under the terms of either the MIT license or GPL.
 *
 * This License is also included in this distribution in the file called
 * "MIT-COPYING".
 *
 * EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * GPLv2:
 * If you wish to use this file under the terms of GPL, following terms are
 * effective.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program
 ****************************************************************************/

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME "/" fmt

#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

#include "igb_streaming.h"

/*TODO:
 *-need to move cache_sync on RX to queue task?
 *-check finished packet for EOP flag
 */

/**
 * global parameters
 */
static char *interface = "eth0";
module_param(interface, charp, 0440);

static int irq_timeout_usec_tx0;
module_param(irq_timeout_usec_tx0, int, 0660);

static int irq_timeout_usec_tx1;
module_param(irq_timeout_usec_tx1, int, 0660);

static int irq_timeout_usec_rx;
module_param(irq_timeout_usec_rx, int, 0660);

static struct streaming_private *stp_ptr;
static struct kmem_cache *streaming_entry_cache;

/**
 * utilities
 */

#define IGB_QUEUE(_hwq) ((_hwq)->tx ? 1 - (_hwq)->chno : (_hwq)->chno)

static void igb_reset_tx_queue(struct igb_adapter *adapter, int i);
static void igb_reset_rx_queue(struct igb_adapter *adapter, int i);

static void igb_enable_tx_queue(struct igb_adapter *adapter, int i);
static void igb_enable_rx_queue(struct igb_adapter *adapter, int i);

static int igb_set_bw(struct igb_adapter *adapter, u32 fraction_a,
		      u32 fraction_b);
static int is_avb_capable_igb(struct net_device *dev);
static int igb_reset_filter(struct igb_adapter *adapter,
			    unsigned int filter_id);
static int igb_set_filter(struct igb_adapter *adapter, unsigned int filter_id,
			  unsigned int queue_id);

static const char *my_netdev_drivername(const struct net_device *dev)
{
	const struct device_driver *driver;
	const struct device *parent;
	const char *empty = "";

	parent = dev->dev.parent;
	if (!parent)
		return empty;

	driver = parent->driver;
	if (driver && driver->name)
		return driver->name;
	return empty;
}

/* TODO: should be removed - this was old stly to identify the 'special'
 * OpenAVb igb driver by its modified name.
 */
static int is_avb_capable_igb(struct net_device *dev)
{
	return !strcmp(my_netdev_drivername(dev), "igb");
}

static u32 e1000_read_reg(struct e1000_hw *hw, u32 reg)
{
	u8 __iomem *hw_addr = READ_ONCE(hw->hw_addr);
	u32 value = 0;

	if (E1000_REMOVED(hw_addr))
		return ~value;

	value = readl(&hw_addr[reg]);

	return value;
}

#define IGB_TX_THRES_DEF \
	(IGB_TX_PTHRESH | (IGB_TX_HTHRESH << 8) | (IGB_TX_WTHRESH << 16))
static void igb_reset_tx_queue(struct igb_adapter *adapter, int i)
{
	struct e1000_hw *hw = &adapter->hw;	/*referenced by IGB_RX_WTHRESH*/
	u64 bus_addr = adapter->tx_ring[i]->dma;

	/* disable queue */
	E1000_WRITE_REG(hw, E1000_TXDCTL(i), IGB_TX_THRES_DEF);

	/* reconfigure descriptors */
	E1000_WRITE_REG(hw, E1000_TDLEN(i), adapter->tx_ring[i]->count *
			sizeof(union e1000_adv_tx_desc));
	E1000_WRITE_REG(hw, E1000_TDBAH(i), (u32)(bus_addr >> 32));
	E1000_WRITE_REG(hw, E1000_TDBAL(i), (u32)bus_addr);

	/* reset pointers and clear ring contents */
	E1000_WRITE_REG(hw, E1000_TDT(i), 0);
	E1000_WRITE_REG(hw, E1000_TDH(i), 0);

	memset((void *)adapter->tx_ring[i]->desc, 0,
	       (sizeof(union e1000_adv_tx_desc)) * adapter->tx_ring[i]->count);
}

/*Enable transmit unit. */
static void igb_enable_tx_queue(struct igb_adapter *adapter, int i)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 txdctl;

	E1000_WRITE_REG(hw, E1000_TXDCTL(i), 0);
	txdctl = IGB_TX_THRES_DEF | E1000_TXDCTL_PRIORITY |
		E1000_TXDCTL_QUEUE_ENABLE;
	E1000_WRITE_REG(hw, E1000_TXDCTL(i), txdctl);
}

#define IGB_RX_THRES_DEF \
	(IGB_RX_PTHRESH | (IGB_RX_HTHRESH << 8) | (IGB_RX_WTHRESH << 16))
static void igb_reset_rx_queue(struct igb_adapter *adapter, int i)
{
	struct e1000_hw *hw = &adapter->hw;	/*referenced by IGB_RX_WTHRESH*/
	u32 srrctl;
	u64 bus_addr = adapter->rx_ring[i]->dma;
	u32 rxdctl;

	srrctl = 2048 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
	srrctl |= E1000_SRRCTL_DESCTYPE_ADV_ONEBUF;

	/* Disable and reconfigure queue */
	E1000_WRITE_REG(hw, E1000_RXDCTL(i), 0);
	E1000_WRITE_REG(hw, E1000_RDLEN(i), adapter->rx_ring[i]->count *
			sizeof(union e1000_adv_rx_desc));
	E1000_WRITE_REG(hw, E1000_RDBAH(i), (uint32_t)(bus_addr >> 32));
	E1000_WRITE_REG(hw, E1000_RDBAL(i), (uint32_t)bus_addr);
	E1000_WRITE_REG(hw, E1000_SRRCTL(i), srrctl);

	/* Enable queue */
	rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
	rxdctl &= 0xFFF00000;
	rxdctl |= E1000_RXDCTL_QUEUE_ENABLE | IGB_RX_THRES_DEF;
	E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);

	/* Clear the ring contents */
	memset((void *)adapter->rx_ring[i]->desc, 0,
	       (sizeof(union e1000_adv_rx_desc)) * adapter->rx_ring[i]->count);
}

/* Enable receive unit. */
static void igb_enable_rx_queue(struct igb_adapter *adapter, int i)
{
	struct e1000_hw *hw = &adapter->hw;	/*referenced by IGB_RX_WTHRESH*/

	u32 rctl, rxcsum = 0;
	u32 rxdctl;

	/* disable receivers and queue */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	E1000_WRITE_REG(hw, E1000_RXDCTL(i), 0);

	/* Enable queue */
	rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
	rxdctl &= 0xFFF00000;
	rxdctl |= E1000_RXDCTL_QUEUE_ENABLE;
	rxdctl |= IGB_RX_PTHRESH;
	rxdctl |= IGB_RX_HTHRESH << 8;
	rxdctl |= IGB_RX_WTHRESH << 16;
	E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);

	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	rxcsum |= E1000_RXCSUM_IPPCSE;
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

	/* Setup and enable Receivers */
	rctl |= E1000_RCTL_SZ_2048 | E1000_RCTL_EN | E1000_RCTL_BAM |
			E1000_RCTL_RDMTS_HALF | E1000_RCTL_SECRC;
	rctl &= ~E1000_RCTL_LPE;
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl &= ~E1000_RCTL_VFE;
	rctl &= ~E1000_RCTL_SBP;
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	/* reset pointers */
	E1000_WRITE_REG(hw, E1000_RDH(i), 0);
	E1000_WRITE_REG(hw, E1000_RDT(i), 0);
}

static int igb_set_bw(struct igb_adapter *adapter, u32 fraction_a,
		      u32 fraction_b)
{
	u32 linkspeed;
	int error = 0;
	struct e1000_hw *hw;

	u32 tqavcc0, tqavcc1;
	u32 tqavhc0, tqavhc1;
	u32 class_a_idle, class_b_idle;

	hw = &adapter->hw;
	linkspeed = adapter->link_speed;

	if (linkspeed < 100)
		return -EINVAL;

	if (!fraction_a && !fraction_b)
		return 0;

	pr_debug("Set BW to %x/%x (%lld/%lld)\n", fraction_a, fraction_b,
		 (u_int64_t)fraction_a * 100 / UINT_MAX,
		 (u_int64_t)fraction_b * 100 / UINT_MAX);

	/*TODO ROUND UP ?*/
	if (linkspeed == 100) {
		class_a_idle = (u32)((u_int64_t)fraction_a * 2 *
				 E1000_TQAVCC_LINKRATE / UINT_MAX) / 10;
		class_b_idle = (u32)((u_int64_t)fraction_b * 2 *
				 E1000_TQAVCC_LINKRATE / UINT_MAX) / 10;
	} else {
		class_a_idle = (u32)((u_int64_t)fraction_a * 2 *
				E1000_TQAVCC_LINKRATE / UINT_MAX);
		class_b_idle = (u32)((u_int64_t)fraction_b * 2 *
				 E1000_TQAVCC_LINKRATE / UINT_MAX);
	}
	class_a_idle++;
	class_b_idle++;

	tqavcc0 = class_a_idle | E1000_TQAVCC_QUEUEMODE;
	tqavcc1 = class_b_idle | E1000_TQAVCC_QUEUEMODE;

	tqavhc0 = E1000_TQAVHC_ZEROCREDIT +
	    (u_int64_t)class_a_idle * 1522 / E1000_TQAVCC_LINKRATE;
	tqavhc1 = E1000_TQAVHC_ZEROCREDIT +
		(u_int64_t)class_b_idle * (1522 + 1522) /
	    (E1000_TQAVCC_LINKRATE - class_a_idle);

	pr_debug("a/b idle: 0x%x 0x%x hc a/b: 0x%x 0x%x\n", class_a_idle,
		 class_b_idle, tqavhc0, tqavhc1);

	E1000_WRITE_REG(hw, E1000_TQAVHC(0), tqavhc0);
	E1000_WRITE_REG(hw, E1000_TQAVCC(0), tqavcc0);
	E1000_WRITE_REG(hw, E1000_TQAVHC(1), tqavhc1);
	E1000_WRITE_REG(hw, E1000_TQAVCC(1), tqavcc1);

	return error;
}

static int igb_reset_filter(struct igb_adapter *adapter, unsigned int filter_id)
{
	struct e1000_hw *hw;
	u32 wufc;

	if (filter_id > 7)
		return -EINVAL;

	hw = &adapter->hw;

	wufc = E1000_READ_REG(hw, E1000_WUFC);
	wufc &= ~(E1000_WUFC_FLX0 << filter_id);
	E1000_WRITE_REG(hw, E1000_WUFC, wufc);

	return 0;
}

/*we can only filter any AVTP frames and need to find proper stq by our own...*/

#define IGB_FHFT_WR(hw, id, idx, val) \
	E1000_WRITE_REG_ARRAY(hw, E1000_FHFT(id), idx, val)

static int igb_set_filter(struct igb_adapter *adapter, unsigned int filter_id,
			  unsigned int queue_id)
{
	struct e1000_hw *hw;
	u32 i, k;
	u32 fhft, wufc;

	u8 filter[128];
	u8 filter_mask[64];
	u32 filter_len;
	struct vlan_ethhdr *ethhdr = NULL;

	if (filter_id > 7)
		return -EINVAL;

	if (!adapter)
		return -ENXIO;

	if (queue_id > 1)
		return -EINVAL;

	hw = &adapter->hw;

	memset(filter_mask, 0, sizeof(filter_mask));
	memset(filter, 0, sizeof(filter));

	/* filter for vlan encapsulated IEEE1722 protocol */
	ethhdr = (struct vlan_ethhdr *)filter;
	ethhdr->h_vlan_proto = htons(ETH_P_8021Q);
	ethhdr->h_vlan_encapsulated_proto = htons(ETH_P_TSN);

	filter_mask[0] = 0x00;	/* 00000000b   */
	filter_mask[1] = 0x30;	/* 00110000b = ethtype */
	filter_mask[2] = 0x03;	/* 00000011b = ethtype after a vlan tag */

	/* length must be 8 byte-aligned */
	filter_len = round_up(VLAN_ETH_HLEN, 8u);

	/* install the filter on desired queue */
	for (i = 0; i < (filter_len / 8); i++) {
		/* setup dw0...*/
		for (fhft = 0, k = 0; k < 4; k++)
			fhft |= ((u32)(filter[(i * 8) + k])) << (k * 8);
		IGB_FHFT_WR(hw, filter_id, (i * 4), fhft);
		/* setup dw1...*/
		for (fhft = 0, k = 0; k < 4; k++)
			fhft |= ((u32)(filter[(i * 8) + 4 + k])) << (k * 8);
		IGB_FHFT_WR(hw, filter_id, (i * 4) + 1, fhft);
		/*.. and corresponding mask*/
		IGB_FHFT_WR(hw, filter_id, (i * 4) + 2, filter_mask[i]);
	}
	/*last dword contains length + queueing field */
	IGB_FHFT_WR(hw, filter_id, 63, (queue_id << 8) | filter_len);

	wufc = E1000_READ_REG(hw, E1000_WUFC) | (E1000_WUFC_FLX0 << filter_id) |
			E1000_WUFC_FLEX_HQ;
	E1000_WRITE_REG(hw, E1000_WUFC, wufc);

	return 0;
}

static inline bool uncached_access(struct stqueue_info *stq)
{
	return !!(stq->flags & O_DSYNC);
}

static inline bool is_readable_count(struct stqueue_info *stq,
				     unsigned int count)
{
	if (stq->blockmode == EAVB_BLOCK_WAITALL)
		return (stq->entrynum.completed >= count) ? true : false;
	else
		return (stq->entrynum.completed > 0) ? true : false;
}

static inline bool is_writeble(struct stqueue_info *stq)
{
	return (stq->entrynum.accepted < RAVB_ENTRY_THRETH) ? true : false;
}

static inline void hwq_sequencer(struct hwqueue_info *hwq, enum AVB_STATE state)
{
	if (hwq->state != state)
		hwq->state = state;
}

static inline void stq_sequencer(struct stqueue_info *stq, enum AVB_STATE state)
{
	if (stq->state != state)
		stq->state = state;
}

static inline u32 hwq_event_irq(struct hwqueue_info *hwq,
				enum AVB_EVENT event, u32 param)
{
	u32 events;

	events = hwq->pendingEvents;

	switch (event) {
	case AVB_EVENT_CLEAR:
		hwq->pendingEvents = 0;
		break;
	case AVB_EVENT_ATTACH:
	case AVB_EVENT_DETACH:
	case AVB_EVENT_RXINT:
	case AVB_EVENT_TXINT:
	case AVB_EVENT_UNLOAD:
	case AVB_EVENT_SWLINK_DOWN:
	case AVB_EVENT_TIMEOUT:
		if (!(events & event)) {
			hwq->pendingEvents |= event;
			wake_up_interruptible(&hwq->waitEvent);
		}
		break;
	default:
		WARN(1, "context error: invalid event type\n");
		break;
	}

	return events;
}

static inline u32 hwq_event(struct hwqueue_info *hwq,
			    enum AVB_EVENT event, u32 param)
{
	struct streaming_private *stp = to_stp(hwq);
	unsigned long flags;
	u32 events;

	spin_lock_irqsave(&stp->lock, flags);

	events = hwq_event_irq(hwq, event, param);

	spin_unlock_irqrestore(&stp->lock, flags);
	return events;
}

static inline u32 hwq_event_clear(struct hwqueue_info *hwq)
{
	return hwq_event(hwq, AVB_EVENT_CLEAR, -1);
}

static void hwq_try_to_start_irq_timeout_timer(struct hwqueue_info *hwq)
{
	int irq_timeout_usec;

	if (hwq->tx) {
		if (!hwq->chno)
			irq_timeout_usec = irq_timeout_usec_tx0;
		else
			irq_timeout_usec = irq_timeout_usec_tx1;
	} else {
		irq_timeout_usec = irq_timeout_usec_rx;
	}

	if (irq_timeout_usec)
		hrtimer_start(&hwq->timer,
			      ns_to_ktime(irq_timeout_usec * NSEC_PER_USEC),
			      HRTIMER_MODE_REL);
}

/**
 * streaming entry operations
 */
static struct stream_entry *get_streaming_entry(void)
{
	struct stream_entry *e;

	e = kmem_cache_alloc(streaming_entry_cache, GFP_KERNEL);
	if (!e)
		return NULL;

	INIT_LIST_HEAD(&e->list);
	memset(e->descs, 0, sizeof(e->descs));
	e->total_bytes = 0;
	e->errors = 0;
	e->stq = NULL;

	return e;
}

static void put_streaming_entry(struct stream_entry *e)
{
	list_del(&e->list);
	kmem_cache_free(streaming_entry_cache, e);
}

static void cachesync_streaming_entry(struct stream_entry *e)
{
	struct eavb_entryvec *evec;
	unsigned int i;
	struct streaming_private *stp = stp_ptr;
	struct net_device *ndev = to_net_dev(stp->device.parent);
	struct device *pdev_dev = ndev->dev.parent;

	evec = e->msg.vec;

	if (e->stq->hwq->tx) {
		for (i = 0; i < e->vecsize; i++, evec++)
			dma_sync_single_for_device(pdev_dev, evec->base,
						   evec->len, DMA_TO_DEVICE);
	} else {
		for (i = 0; i < e->vecsize; i++, evec++)
			dma_sync_single_for_cpu(pdev_dev, evec->base,
						evec->len, DMA_FROM_DEVICE);
	}
}

static void get_rx_mem(struct hwqueue_info *hwq, struct rx_dma *dest)
{
	*dest = hwq->rx_buf[hwq->rx_curr];
	hwq->rx_curr = (hwq->rx_curr + 1) % hwq->ringsize;
}

static int alloc_rx_mem(struct hwqueue_info *hwq)
{
	struct streaming_private *stp = to_stp(hwq);
	struct net_device *ndev = to_net_dev(stp->device.parent);
	struct device *dev = ndev->dev.parent;
	dma_addr_t paddr, pitch;
	unsigned int max_packet_size = 1540;
	int i;
	int size = max_packet_size * hwq->ringsize;

	/* round up to nearest 4K */
	size = ALIGN(size, 4096);

	hwq->rx_vaddr = dma_alloc_coherent(dev, size, &paddr, GFP_KERNEL);
	hwq->rx_curr = 0;

	for (i = 0; i < hwq->ringsize; i++) {
		hwq->rx_buf[i].len = max_packet_size;
		pitch = max_packet_size * i;
		hwq->rx_buf[i].len = max_packet_size;
		hwq->rx_buf[i].paddr = paddr + pitch;
		hwq->rx_buf[i].vaddr = ((char *)hwq->rx_vaddr) + pitch;
	}
	return 0;
}

/**
 * descriptor encode/decode
 */
static void clear_desc(struct hwqueue_info *hwq)
{
	if (hwq->tx) {
		igb_reset_tx_queue(hwq->stp->iadapter, IGB_QUEUE(hwq));
		igb_enable_tx_queue(hwq->stp->iadapter, IGB_QUEUE(hwq));
	} else {
		igb_reset_rx_queue(hwq->stp->iadapter, IGB_QUEUE(hwq));
		igb_enable_rx_queue(hwq->stp->iadapter, IGB_QUEUE(hwq));
	}

	hwq->remain = hwq->ringsize;
	hwq->curr = 0;
	hwq->rx_curr = 0;
	hwq->dstats.rx_dirty = hwq->dstats.rx_current;
	hwq->dstats.tx_dirty = hwq->dstats.tx_current;
}

static void *get_desc(struct hwqueue_info *hwq)
{
	struct igb_common_desc *desc;

	if (hwq->tx)
		desc =
		    (struct igb_common_desc *)(hwq->stp->iadapter->
					       tx_ring[IGB_QUEUE(hwq)]->desc) +
		    hwq->curr;
	else
		desc =
		    (struct igb_common_desc *)(hwq->stp->iadapter->
					       rx_ring[IGB_QUEUE(hwq)]->desc) +
		    hwq->curr;

	hwq->remain--;
	hwq->curr = (hwq->curr + 1) % hwq->ringsize;
	/* Keep lowest free descriptor use - since driver load */
	hwq->minremain = min_t(s32, hwq->minremain, hwq->remain);

	return desc;
}

static void put_desc(struct hwqueue_info *hwq, void *buf)
{
	if (buf)
		hwq->remain++;
}

static int desc_pre_encode_rx(struct stream_entry *e, struct hwqueue_info *hwq)
{
	struct eavb_entryvec *evec;
	int i;

	evec = e->msg.vec;

	for (i = 0; i < EAVB_ENTRYVECNUM; i++, evec++) {
		union e1000_adv_rx_desc *desc;

		if (!evec->len)
			break;

		desc = (union e1000_adv_rx_desc *)&e->pre_enc[i];
		desc->read.hdr_addr = 0;
		desc->read.pkt_addr = 0;
	}

	return i;
}

static int desc_pre_encode_tx(struct stream_entry *e, struct hwqueue_info *hwq)
{
	struct eavb_entryvec *evec;
	int i;

	evec = e->msg.vec;

	for (i = 0; i < EAVB_ENTRYVECNUM; i++, evec++) {
		union e1000_adv_tx_desc *desc;
		struct e1000_adv_tx_context_desc *ctx;

		if (!evec->base && !evec->len)
			break;

		ctx = (struct e1000_adv_tx_context_desc *)&e->pre_enc[i];
		ctx->vlan_macip_lens = 0;
		ctx->type_tucmd_mlhl =
		    cpu_to_le32((u32)
				(E1000_ADVTXD_DCMD_DEXT |
				 E1000_ADVTXD_DTYP_CTXT));
		ctx->mss_l4len_idx = 0;
		i++;

		desc = (union e1000_adv_tx_desc *)&e->pre_enc[i];
		desc->read.buffer_addr = cpu_to_le64(evec->base);
		/* Last Descriptor of Packet needs End Of Packet (EOP)
		 * and Report Status (RS)
		 */
		desc->read.cmd_type_len =
		    cpu_to_le32(evec->
				len | E1000_ADVTXD_DTYP_DATA |
				E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT
				| E1000_ADVTXD_DCMD_EOP | E1000_ADVTXD_DCMD_RS);
		/*FIXME: E1000_TXD_STAT_TS is response - timestamping is
		 * enabled by 1588_STAT_EN flag in the TQAVCTRL register
		 */
		desc->read.olinfo_status =
		    cpu_to_le32(E1000_TXD_STAT_TS | evec->
				len << E1000_ADVTXD_PAYLEN_SHIFT);
	}
	return i;
}

static int desc_pre_encode(struct stream_entry *e, struct hwqueue_info *hwq)
{
	if (hwq->tx)
		return desc_pre_encode_tx(e, hwq);
	else
		return desc_pre_encode_rx(e, hwq);
}

/* Caller must check remain */
static void desc_copy(struct hwqueue_info *hwq, struct stream_entry *e)
{
	u64 dstats_current = 0;
	struct e1000_hw *hw = &hwq->stp->iadapter->hw;

	if (hwq->tx) {
		unsigned int i;

		for (i = 0; i < e->vecsize; i++) {
			union e1000_adv_tx_desc *desc;

			desc = get_desc(hwq);
			*(struct igb_common_desc *)desc = e->pre_enc[i];

			dstats_current++;
			e->descs[i] = (struct igb_common_desc *)desc;
		}
		/*make sure desc is updated fefore queue index is updated*/
		wmb();
		E1000_WRITE_REG(hw, E1000_TDT(IGB_QUEUE(hwq)), hwq->curr);
		hwq->dstats.tx_current += dstats_current;
	} else {
		union e1000_adv_rx_desc *desc;

		e->vecsize = 1;
		desc = get_desc(hwq);
		get_rx_mem(hwq, &e->hw);
		desc->read.pkt_addr = cpu_to_le64(e->hw.paddr);
		desc->read.hdr_addr = 0;
		dstats_current++;
		e->descs[0] = (struct igb_common_desc *)desc;
		/*make sure desc is updated fefore queue index is updated*/
		wmb();
		E1000_WRITE_REG(hw, E1000_RDT(IGB_QUEUE(hwq)), hwq->curr);

		hwq->dstats.rx_current += dstats_current;
	}
}

static bool desc_decode_rx(struct hwqueue_info *hwq, struct stream_entry *e)
{
	union e1000_adv_rx_desc *desc;
	struct eavb_entryvec *evec;
	bool progress = true;
	u32 staterr = 0;
	unsigned int i;

	evec = e->msg.vec;
	for (i = 0; i < e->vecsize; i++, evec++) {
		desc = (union e1000_adv_rx_desc *)e->descs[i];
		if (!desc)
			continue;

		staterr = le32_to_cpu(desc->wb.upper.status_error);
		if ((staterr & E1000_RXD_STAT_DD) == 0)
			continue;

		desc->wb.upper.status_error = 0;
		put_desc(hwq, desc);

		if (staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK) {
			e->descs[i] = NULL;
			evec->len = 0;
			pr_debug("finished RX (ERROR) stat 0x%x remain %d\n",
				 staterr, hwq->remain);
			progress = false;
			continue;
		}
		hwq->dstats.rx_dirty++;
		progress = false;

		evec->len = le16_to_cpu(desc->wb.upper.length);

		if (staterr & E1000_RXD_STAT_VP)
			e->hw.vlan_tag = le16_to_cpu(desc->wb.upper.vlan);
		else
			e->hw.vlan_tag = 0;

		pr_debug("finished RX (len %d vlan %x ) desc.remain %d\n",
			 desc->wb.upper.length, desc->wb.upper.vlan,
			 hwq->remain);

		e->total_bytes += evec->len;

		e->descs[i] = NULL;
	}

	return progress;
}

static bool desc_decode_tx(struct hwqueue_info *hwq, struct stream_entry *e)
{
	struct eavb_entryvec *evec;
	unsigned int i;

	if (e->vecsize != 2)
		pr_err("invalid vecsize %d", e->vecsize);

	evec = e->msg.vec;
	for (i = 0; i < e->vecsize; i++, evec++) {
		union e1000_adv_tx_desc *desc;

		desc = (union e1000_adv_tx_desc *)e->descs[i];
		if (!desc)
			continue;
		/*skip first ...*/
		if (i == 0)
			continue;

		if (!(desc->wb.status & cpu_to_le32(E1000_TXD_STAT_DD)))
			break;

		put_desc(hwq, desc);
		hwq->dstats.tx_dirty++;

		e->total_bytes += evec->len;
		e->descs[i] = NULL;

		/*put back head descriptor*/
		desc = (union e1000_adv_tx_desc *)e->descs[i - 1];
		put_desc(hwq, desc);
		e->descs[i - 1] = NULL;
	}
	return (i == e->vecsize) ? false : true;
}

static bool desc_decode(struct hwqueue_info *hwq, struct stream_entry *e)
{
	if (hwq->tx)
		return desc_decode_tx(hwq, e);
	else
		return desc_decode_rx(hwq, e);
}

/**
 * calcuration CBS parameter functions
 */
static void update_cbs_param(bool add, enum eavb_streamclass class,
			     struct eavb_cbsparam *cbs, bool apply)
{
	struct streaming_private *stp = stp_ptr;
	struct eavb_cbsparam *cbs_total;
	u32 max_frame_size = 2012 * 8;	/* bit */
	u32 max_interference_size =
		((class == EAVB_CLASSA) ? 2100 : 4200) * 8;	/* bit */

	cbs_total = &stp->cbsInfo.param[class];

	if (add) {
		cbs_total->bandwidthFraction += cbs->bandwidthFraction;
		stp->cbsInfo.bandwidthFraction += cbs->bandwidthFraction;
	} else {
		cbs_total->bandwidthFraction -= cbs->bandwidthFraction;
		stp->cbsInfo.bandwidthFraction -= cbs->bandwidthFraction;
	}

	cbs_total->idleSlope = cbs_total->bandwidthFraction >> 16;

	if (!cbs_total->idleSlope)
		cbs_total->idleSlope = 1;

	cbs_total->sendSlope = (U16_MAX - cbs_total->idleSlope) * -1;
	cbs_total->hiCredit = max_interference_size * cbs_total->idleSlope;
	cbs_total->loCredit = max_frame_size * cbs_total->sendSlope;

	if (apply) {
		if (class == EAVB_CLASSA)
			igb_set_bw(stp->iadapter, cbs_total->bandwidthFraction,
				   stp->cbsInfo.param[EAVB_CLASSB].
				   bandwidthFraction);
		else
			igb_set_bw(stp->iadapter,
				   stp->cbsInfo.param[EAVB_CLASSA].
				   bandwidthFraction,
				   cbs_total->bandwidthFraction);
	}
}

#define HWQ_CLASS(_hwq) ((_hwq)->chno)

static inline void register_cbs_param(enum eavb_streamclass class,
				      struct eavb_cbsparam *cbs)
{
	update_cbs_param(true, class, cbs, true);
}

static inline void unregister_cbs_param(enum eavb_streamclass class,
					struct eavb_cbsparam *cbs, bool apply)
{
	update_cbs_param(false, class, cbs, apply);
}

static int separation_filter_initialise(struct igb_adapter *iadapter)
{
	/*we use only first filter*/
	igb_reset_filter(iadapter, 0);
	igb_reset_filter(iadapter, 1);

	return 0;
}

static int register_stream_id(struct stqueue_info *stq, u8 sid[8])
{
	const u8 sid_empty[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	if (!memcmp(sid, sid_empty, 8)) {
		stq->sid = 0;
		igb_reset_filter(stq->hwq->stp->iadapter, IGB_QUEUE(stq->hwq));
	} else {
		int i;

		stq->sid = 0;
		/*store in network order to directly compare
		 *with received frame
		 */
		for (i = 0; i < 8; i++)
			stq->sid |= ((u64)sid[i]) << (i * 8);
		igb_set_filter(stq->hwq->stp->iadapter, IGB_QUEUE(stq->hwq),
			       IGB_QUEUE(stq->hwq));
	}

	return 0;
}

/**
 * stqueue info operations
 */
static void stq_release(struct kobject *kobj)
{
	struct stqueue_info *stq = to_stq(kobj);
	struct hwqueue_info *hwq = stq->hwq;
	struct stream_entry *e, *e1;

	if (hwq->tx)
		unregister_cbs_param(HWQ_CLASS(hwq), &stq->cbs, true);
	list_for_each_entry_safe(e, e1, &stq->entryWaitQueue, list)
		put_streaming_entry(e);
	list_for_each_entry_safe(e, e1, &stq->entryLogQueue, list)
		put_streaming_entry(e);

	/* merge statistics values */
	hwq->pstats.rx_packets += stq->pstats.rx_packets;
	hwq->pstats.tx_packets += stq->pstats.tx_packets;
	hwq->pstats.rx_bytes += stq->pstats.rx_bytes;
	hwq->pstats.tx_bytes += stq->pstats.tx_bytes;
	hwq->pstats.rx_errors += stq->pstats.rx_errors;
	hwq->pstats.tx_errors += stq->pstats.tx_errors;

	kfree(stq);
}

static struct kobj_type stq_ktype = {
	.release = stq_release,
};

static struct stqueue_info *get_stq(struct hwqueue_info *hwq, int idx)
{
	struct stqueue_info *stq;

	stq = kzalloc(sizeof(*stq), GFP_KERNEL);
	if (unlikely(!stq))
		goto no_memory;

	stq->qno = idx;

	if (hwq->tx)
		stq->index = hwq->index * RAVB_STQUEUE_NUM + idx;
	else
		stq->index = RAVB_HWQUEUE_TXNUM * RAVB_STQUEUE_NUM +
		    hwq->index - RAVB_HWQUEUE_TXNUM;

	init_waitqueue_head(&stq->waitEvent);
	INIT_LIST_HEAD(&stq->entryWaitQueue);
	INIT_LIST_HEAD(&stq->entryLogQueue);

	stq->list.next = LIST_POISON1;	/* for debug */
	stq->list.prev = LIST_POISON2;	/* for debug */
	stq->hwq = hwq;

	stq_sequencer(stq, AVB_STATE_IDLE);

	stq->kobj.kset = hwq->attached;

	if (kobject_init_and_add(&stq->kobj, &stq_ktype,
				 NULL, "%s:%d", hwq_name(hwq), stq->qno))
		goto no_kobj;

	return stq;

 no_kobj:
 no_memory:
	return NULL;
}

static void put_stq(struct stqueue_info *stq)
{
	kobject_del(&stq->kobj);
	kobject_put(&stq->kobj);
}

/**
 * streaming driver API functions
 */
/* This API can be called from only the kernel driver */
static long ravb_get_entrynum_kernel(void *handle,
				     struct eavb_entrynum *entrynum)
{
	struct stqueue_info *stq = handle;

	if (!stq)
		return -EINVAL;

	if (!entrynum)
		return -EINVAL;

	*entrynum = stq->entrynum;

	return 0;
}

/* This API can be called from only the kernel driver */
static long ravb_get_linkspeed(void *handle)
{
	struct stqueue_info *stq = handle;
	struct hwqueue_info *hwq;
	struct streaming_private *stp;

	if (!stq)
		return -EINVAL;

	hwq = stq->hwq;
	stp = to_stp(hwq);

	return (long)stp->iadapter->link_speed;
}

/* This API can be called from only the kernel driver */
static long ravb_blocking_cancel_kernel(void *handle)
{
	struct stqueue_info *stq = handle;
	struct hwqueue_info *hwq;

	if (!stq)
		return -EINVAL;

	hwq = stq->hwq;
	if (!(stq->flags & O_NONBLOCK)) {
		down(&hwq->sem);
		stq->cancel = true;
		up(&hwq->sem);
		wake_up_interruptible(&stq->waitEvent);
	}

	return 0;
}

static long ravb_set_txparam_kernel(void *handle, struct eavb_txparam *txparam)
{
	struct streaming_private *stp = stp_ptr;
	struct stqueue_info *stq = handle;
	u64 bw;

	if (!stq)
		return -EINVAL;

	if (!txparam)
		return -EINVAL;

	if (!stq->hwq->tx)
		return -EPERM;

	if (stq->state != AVB_STATE_IDLE)
		return -EBUSY;

	pr_debug("set_txparam: %s %08x %08x %08x\n",
		 stq_name(stq),
		 txparam->cbs.bandwidthFraction,
		 txparam->cbs.idleSlope, txparam->cbs.sendSlope);

	down(&stp->sem);

	bw = stp->cbsInfo.bandwidthFraction;
	bw -= stq->cbs.bandwidthFraction;
	bw += txparam->cbs.bandwidthFraction;

	if (bw > RAVB_CBS_BANDWIDTH_LIMIT) {
		up(&stp->sem);
		return -ENOSPC;
	}

	unregister_cbs_param(HWQ_CLASS(stq->hwq), &stq->cbs, false);
	memcpy(&stq->cbs, &txparam->cbs, sizeof(stq->cbs));
	register_cbs_param(HWQ_CLASS(stq->hwq), &stq->cbs);

	up(&stp->sem);

	return 0;
}

static long ravb_get_txparam_kernel(void *handle, struct eavb_txparam *txparam)
{
	struct stqueue_info *stq = handle;

	if (!stq)
		return -EINVAL;

	if (!txparam)
		return -EINVAL;

	if (!stq->hwq->tx)
		return -EPERM;

	pr_debug("get_txparam: %s\n", stq_name(stq));

	memcpy(&txparam->cbs, &stq->cbs, sizeof(stq->cbs));

	return 0;
}

static long ravb_set_rxparam_kernel(void *handle, struct eavb_rxparam *rxparam)
{
	struct streaming_private *stp = stp_ptr;
	struct stqueue_info *stq = handle;
	int ret;

	if (!stq)
		return -EINVAL;

	if (!rxparam)
		return -EINVAL;

	if (stq->hwq->tx)
		return -EPERM;

	down(&stp->sem);
	ret = register_stream_id(stq, rxparam->streamid);
	up(&stp->sem);
	if (ret)
		return ret;

	pr_debug("set_rxparam: %s %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
		 stq_name(stq),
		 rxparam->streamid[0], rxparam->streamid[1],
		 rxparam->streamid[2], rxparam->streamid[3],
		 rxparam->streamid[4], rxparam->streamid[5],
		 rxparam->streamid[6], rxparam->streamid[7]);
	pr_debug("SID %llx\n", stq->sid);

	return 0;
}

static long ravb_get_rxparam_kernel(void *handle, struct eavb_rxparam *rxparam)
{
	struct stqueue_info *stq = handle;

	if (!stq)
		return -EINVAL;

	if (!rxparam)
		return -EINVAL;

	if (stq->hwq->tx)
		return -EPERM;

	pr_debug("get_rxparam: %s\n", stq_name(stq));

	memcpy(rxparam->streamid, stq->hwq->streamID,
	       sizeof(rxparam->streamid));

	return 0;
}

static long ravb_set_option_kernel(void *handle, struct eavb_option *option)
{
	struct stqueue_info *stq = handle;

	if (!stq)
		return -EINVAL;

	if (!option)
		return -EINVAL;

	pr_debug("set_option: %s %d %08x\n",
		 stq_name(stq), option->id, option->param);

	switch (option->id) {
	case EAVB_OPTIONID_BLOCKMODE:
		switch (option->param) {
		case EAVB_BLOCK_NOWAIT:
		case EAVB_BLOCK_WAITALL:
			stq->blockmode = option->param;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static long ravb_get_option_kernel(void *handle, struct eavb_option *option)
{
	struct stqueue_info *stq = handle;

	if (!stq)
		return -EINVAL;

	if (!option)
		return -EINVAL;

	switch (option->id) {
	case EAVB_OPTIONID_BLOCKMODE:
		option->param = stq->blockmode;
		break;
	default:
		return -EINVAL;
	}

	pr_debug("get_option: %s %d %08x\n",
		 stq_name(stq), option->id, option->param);

	return 0;
}

static int ravb_streaming_read_stq_kernel(void *handle,
					  struct eavb_entry *buf,
					  unsigned int num)
{
	struct stqueue_info *stq = handle;
	struct hwqueue_info *hwq;
	struct stream_entry *e;
	unsigned int i;
	int err;

	if (!stq)
		return -EINVAL;

	if (!buf)
		return -EINVAL;

	pr_debug("read: %s > num=%d\n", stq_name(stq), num);

	if (!num)
		return 0;

	if ((stq->blockmode == EAVB_BLOCK_WAITALL) && (num > RAVB_ENTRY_THRETH))
		return -ENOMEM;

	hwq = stq->hwq;

	down(&hwq->sem);
	stq->cancel = false;
	if (!is_readable_count(stq, num)) {
		up(&hwq->sem);

		if (stq->flags & O_NONBLOCK)
			return -EAGAIN;

		err = wait_event_interruptible(stq->waitEvent,
					       is_readable_count(stq, num) ||
					       stq->cancel);
		if (err < 0)
			return -EINTR;

		down(&hwq->sem);
	}

	num = min_t(u32, (u32)num, stq->entrynum.completed);
	for (i = 0; i < num; i++) {
		e = list_first_entry(&stq->entryLogQueue,
				     struct stream_entry, list);
		memcpy(buf + i, &e->msg, sizeof(struct eavb_entry));
		if (!uncached_access(stq))
			cachesync_streaming_entry(e);
		put_streaming_entry(e);
	}

	stq->entrynum.accepted -= i;
	stq->entrynum.completed -= i;
	if (hwq->tx) {
		if (stq->dstats.tx_entry_complete >= (u64)i) {
			stq->dstats.tx_entry_complete -= (u64)i;
		} else {
			pr_warn("read: tx underflow (0x%016llx < %d)\n",
				stq->dstats.tx_entry_complete, i);
			stq->dstats.tx_entry_complete = 0x8000000000000000ull;
		}
	} else {
		if (stq->dstats.rx_entry_complete >= (u64)i) {
			stq->dstats.rx_entry_complete -= (u64)i;
		} else {
			pr_warn("read: rx underflow (0x%016llx < %d)\n",
				stq->dstats.rx_entry_complete, i);
			stq->dstats.rx_entry_complete = 0x8000000000000000ull;
		}
	}
	if (hwq->state == AVB_STATE_DOWN)
		err = -ENODEV;
	else
		err = i;

	up(&hwq->sem);
	wake_up_interruptible(&stq->waitEvent);

	pr_debug("read: %s < num=%d err %d\n", stq_name(stq), i, err);

	return err;
}

static int ravb_streaming_write_stq_kernel(void *handle,
					   struct eavb_entry *buf,
					   unsigned int num)
{
	struct stqueue_info *stq = handle;
	struct hwqueue_info *hwq;
	struct stream_entry *e;
	struct list_head entry_queue;
	unsigned int i;
	int err;

	if (!stq)
		return -EINVAL;

	if (!buf)
		return -EINVAL;

	pr_debug("write: %s > num=%d\n", stq_name(stq), num);

	if (!num)
		return 0;

	hwq = stq->hwq;

	down(&hwq->sem);
	stq->cancel = false;
	if (!is_writeble(stq)) {
		up(&hwq->sem);

		if (stq->flags & O_NONBLOCK)
			return -EAGAIN;

		err = wait_event_interruptible(stq->waitEvent,
					       is_writeble(stq) || stq->cancel);
		if (err < 0)
			return -EINTR;

		down(&hwq->sem);
	}

	num = min_t(u32, (u32)num, RAVB_ENTRY_THRETH - stq->entrynum.accepted);
	up(&hwq->sem);
	/* entry remain is full */
	if (!num)
		return 0;

	INIT_LIST_HEAD(&entry_queue);
	for (i = 0; i < num; i++) {
		e = get_streaming_entry();
		if (!e)
			break;
		memcpy(&e->msg, buf + i, sizeof(struct eavb_entry));

		e->vecsize = desc_pre_encode(e, stq->hwq);
		if (e->vecsize == 0) {
			/* TODO countup invalid entry num */
			pr_warn("write: %s invalid entry(%08x) ignored\n",
				stq_name(stq), e->msg.seq_no);
			put_streaming_entry(e);
		} else {
			e->stq = stq;
			if (!uncached_access(stq))
				cachesync_streaming_entry(e);
			list_move_tail(&e->list, &entry_queue);
		}
	}

	down(&hwq->sem);

	stq->entrynum.accepted += i;
	if (hwq->tx)
		stq->dstats.tx_entry_wait += (u64)i;
	else
		stq->dstats.rx_entry_wait += (u64)i;

	list_splice_tail(&entry_queue, &stq->entryWaitQueue);

	/* if IDLE or WAITCOMPLETE, attach to hwq */
	if ((stq->state == AVB_STATE_IDLE) ||
	    (stq->state == AVB_STATE_WAITCOMPLETE)) {
		stq_sequencer(stq, AVB_STATE_ACTIVE);
		list_add_tail(&stq->list, &hwq->activeStreamQueue);
		hwq_event(hwq, AVB_EVENT_ATTACH, stq->qno);
	}
	if (hwq->state == AVB_STATE_DOWN)
		err = -ENODEV;
	else
		err = i;

	up(&hwq->sem);

	pr_debug("write: %s < num=%d err %d\n", stq_name(stq), i, err);

	return err;
}

int ravb_streaming_open_stq_kernel(enum AVB_DEVNAME dev_name,
				   struct ravb_streaming_kernel_if *kif,
				   unsigned int flags)
{
	struct streaming_private *stp = stp_ptr;
	struct hwqueue_info *hwq;
	struct stqueue_info *stq;
	int index, qno, n_queues;

	if (!kif)
		return -EINVAL;
	index = dev_name;

	/*16 rx + 2 tx 'queues' regardless of hardware support */
	if ((index >= (RAVB_STQUEUE_RXNUM + 2)) || (index < 0))
		return -ENODEV;

	/*  mapping:
	 *  0 ->0 (TX0)
	 *  1 ->1 (TX1)
	 *  2.. ->2 (RX0) RX1 unused
	 */
	if (index >= RAVB_HWQUEUE_TXNUM)
		index = RAVB_HWQUEUE_TXNUM;

	pr_debug("open dev %d -> index %d\n", dev_name, index);

	hwq = &stp->hwqueueInfoTable[index];

	/*allow 16 streams on TX, 16/hw_que_cnt on RX */
	n_queues =
	    (hwq->tx) ? RAVB_STQUEUE_TXNUM : RAVB_STQUEUE_RXNUM /
	    RAVB_HWQUEUE_RXNUM;

	down(&hwq->sem);
	qno = find_first_zero_bit(hwq->stream_map, n_queues);
	if (!(qno < n_queues)) {
		up(&hwq->sem);
		return -EBUSY;
	}

	stq = get_stq(hwq, qno);
	if (!stq) {
		up(&hwq->sem);
		return -ENOMEM;
	}

	kif->handle = stq;
	kif->read = &ravb_streaming_read_stq_kernel;
	kif->write = &ravb_streaming_write_stq_kernel;
	kif->set_txparam = &ravb_set_txparam_kernel;
	kif->get_txparam = &ravb_get_txparam_kernel;
	kif->set_rxparam = &ravb_set_rxparam_kernel;
	kif->get_rxparam = &ravb_get_rxparam_kernel;
	kif->set_option = &ravb_set_option_kernel;
	kif->get_option = &ravb_get_option_kernel;
	kif->get_entrynum = &ravb_get_entrynum_kernel;
	kif->get_linkspeed = &ravb_get_linkspeed;
	kif->blocking_cancel = &ravb_blocking_cancel_kernel;

	stq->flags = flags;

	hwq->stqueue_table[qno] = stq;
	set_bit(qno, hwq->stream_map);

	up(&hwq->sem);

	pr_debug("open: %s\n", stq_name(stq));

	return 0;
}
EXPORT_SYMBOL(ravb_streaming_open_stq_kernel);

int ravb_streaming_release_stq_kernel(void *handle)
{
	struct streaming_private *stp = stp_ptr;
	struct stqueue_info *stq = handle;
	struct hwqueue_info *hwq;
	int res;

	if (!stq)
		return -EINVAL;

	pr_debug("close: %s\n", stq_name(stq));

	hwq = stq->hwq;

	/**
	 * if ACTIVE or WAITCOMPLETE,
	 * wait complete all entry processed.
	 */
	down(&hwq->sem);
	switch (stq->state) {
	case AVB_STATE_ACTIVE:
	case AVB_STATE_WAITCOMPLETE:
		if (!hwq->tx) {
			/*on rx we can directly remove ourselve from
			 *activeStreamQueue and enter idle state
			 */
			list_del(&stq->list);
			stq_sequencer(stq, AVB_STATE_IDLE);
		}

		hwq_event(hwq, AVB_EVENT_DETACH, stq->qno);
		up(&hwq->sem);
		do {
			res = wait_event_interruptible(stq->waitEvent,
						       stq->state ==
						       AVB_STATE_IDLE);
		} while (res);
		down(&hwq->sem);
		break;
	default:
		break;
	}

	clear_bit(stq->qno, hwq->stream_map);
	up(&hwq->sem);

	down(&stp->sem);
	put_stq(stq);
	up(&stp->sem);

	return 0;
}
EXPORT_SYMBOL(ravb_streaming_release_stq_kernel);

static inline int ravb_enable_interrupt(struct net_device *ndev,
					struct hwqueue_info *hwq)
{
	struct e1000_hw *hw = &hwq->stp->iadapter->hw;
	/*IGB has one IRQ for RX/TX in common.
	 *Enable in HW if RX or TX side requests enabling
	 */
	spin_lock(&hwq->stp->hwqueueInfoTable[hwq->chno].hw_lock);
	hwq->irqen = true;

	if (hwq->tx)
		E1000_WRITE_REG(hw, E1000_EIMS,
				hwq->stp->iadapter->tx_ring[IGB_QUEUE(hwq)]->
				q_vector->eims_value);
	else
		E1000_WRITE_REG(hw, E1000_EIMS,
				hwq->stp->iadapter->rx_ring[IGB_QUEUE(hwq)]->
				q_vector->eims_value);
	spin_unlock(&hwq->stp->hwqueueInfoTable[hwq->chno].hw_lock);

	/* if irq timeout isn't zero, start irq timeout timer */
	hwq_try_to_start_irq_timeout_timer(hwq);

	return 0;
}

static inline int ravb_disable_interrupt(struct net_device *ndev,
					 struct hwqueue_info *hwq)
{
	struct hwqueue_info *hwq_other;
	struct e1000_hw *hw = &hwq->stp->iadapter->hw;

	if (hwq->tx)
		hwq_other =
		    &hwq->stp->hwqueueInfoTable[RAVB_HWQUEUE_TXNUM + hwq->chno];
	else
		hwq_other = &hwq->stp->hwqueueInfoTable[hwq->chno];

	/*IGB has one IRQ for RX/TX in common.
	 *Disable in HW if RX and TX side requests disabling
	 */
	spin_lock(&hwq->stp->hwqueueInfoTable[hwq->chno].hw_lock);
	hwq->irqen = false;

	if (!hwq_other->irqen) {
		if (hwq->tx)
			E1000_WRITE_REG(hw, E1000_EIMC,
					hwq->stp->iadapter->
					tx_ring[IGB_QUEUE(hwq)]->q_vector->
					eims_value);
		else
			E1000_WRITE_REG(hw, E1000_EIMC,
					hwq->stp->iadapter->
					rx_ring[IGB_QUEUE(hwq)]->q_vector->
					eims_value);
	}
	spin_unlock(&hwq->stp->hwqueueInfoTable[hwq->chno].hw_lock);

	return 0;
}

static int hwq_task_process_terminate(struct hwqueue_info *hwq)
{
	struct stqueue_info *stq, *stq1;
	struct stream_entry *e, *e1;
	struct stqueue_info *stq_pool[RAVB_STQUEUE_NUM] = { NULL };
	int i;

	hwq_sequencer(hwq, AVB_STATE_DOWN);

	/* clear descriptor chain */
	clear_desc(hwq);

	/* flush activeStreamQueue */
	list_for_each_entry_safe(stq, stq1, &hwq->activeStreamQueue, list) {
		stq_pool[stq->qno] = stq;
		list_del(&stq->list);
	}
	/* flush completeWaitQueue */
	list_for_each_entry_safe(e, e1, &hwq->completeWaitQueue, list) {
		if (e->stq)
			stq_pool[e->stq->qno] = e->stq;
		put_streaming_entry(e);
	}

	/* raise stream queue */
	for (i = 0; i < RAVB_STQUEUE_NUM; i++) {
		stq = stq_pool[i];
		if (stq) {
			stq_sequencer(stq, AVB_STATE_IDLE);
			wake_up_interruptible(&stq->waitEvent);
		}
	}

	return 0;
}

static int hwq_task_process_encode(struct hwqueue_info *hwq)
{
	struct stqueue_info *stq = NULL;
	struct stream_entry *e;

	while (hwq->remain >= EAVB_ENTRYVECNUM &&
	       !list_empty(&hwq->activeStreamQueue)) {
		if (hwq->tx) {
			stq = list_first_entry(&hwq->activeStreamQueue,
					       struct stqueue_info, list);
			e = list_first_entry(&stq->entryWaitQueue,
					     struct stream_entry, list);
			desc_copy(hwq, e);
			list_move_tail(&e->list, &hwq->completeWaitQueue);
			stq->entrynum.processed++;

			stq->dstats.tx_entry_wait--;

			if (list_empty(&stq->entryWaitQueue)) {
				list_del(&stq->list);
				stq_sequencer(stq, AVB_STATE_WAITCOMPLETE);
			} else {
				list_move_tail(&stq->list,
					       &hwq->activeStreamQueue);
			}
		} else {
			/*if there is any stream active we schedule a rx packet
			 *to hw queue
			 */
			e = get_streaming_entry();
			desc_copy(hwq, e);

			list_move_tail(&e->list, &hwq->completeWaitQueue);
		}
	}
	return 0;
}

/*get matching entry waiting for this packet...*/
static struct stream_entry *get_rx_entry(struct hwqueue_info *hwq,
					 struct stream_entry *e_hw)
{
	struct stqueue_info *stq;
	struct stream_entry *e_user = NULL;
	//minlen: ethheader + 1722 header up to STREMAID
	unsigned int minlen = VLAN_ETH_HLEN + 12;
	unsigned int sid_offs = VLAN_ETH_HLEN + 4;
	u64 rx_sid;

	if (e_hw->hw.vlan_tag) {
		minlen -= VLAN_HLEN;
		sid_offs -= VLAN_HLEN;
	}

	if (e_hw->msg.vec[0].len < minlen)
		return NULL;

	rx_sid = *(u64 *)((char *)(e_hw->hw.vaddr) + sid_offs);

	list_for_each_entry(stq, &hwq->activeStreamQueue, list) {
		if (stq->sid == rx_sid) {
			e_user =
			    list_first_entry_or_null(&stq->entryWaitQueue,
						     struct stream_entry, list);
			break;
		}
	}
	return e_user;
}

static int hwq_task_process_decode(struct hwqueue_info *hwq)
{
	struct stqueue_info *stq;
	struct stream_entry *e, *e1, *es;
	struct stqueue_info *stq_pool[RAVB_STQUEUE_NUM] = { NULL };
	int i;
	bool progress;

	progress = false;

	list_for_each_entry_safe(e, e1, &hwq->completeWaitQueue, list) {
		progress = desc_decode(hwq, e);
		if (progress)
			break;
		if (!hwq->tx) {
			/*find matching entry in streamqueues... */
			es = get_rx_entry(hwq, e);
			if (!es) {
				/*drop packet */
				put_streaming_entry(e);
				continue;
			}
			/*correct the backlog... */
			es->stq->entrynum.processed++;
			es->stq->dstats.rx_entry_wait--;
			if (e->hw.vlan_tag) {
				struct vlan_ethhdr *vlan = es->msg.vec[0].vaddr;

				memcpy(es->msg.vec[0].vaddr, e->hw.vaddr,
				       ETH_ALEN * 2);
				vlan->h_vlan_proto = htons(ETH_P_8021Q);
				vlan->h_vlan_TCI = htons(e->hw.vlan_tag);
				memcpy((char *)(es->msg.vec[0].vaddr) +
				       ETH_ALEN * 2 + VLAN_HLEN,
				       (char *)(e->hw.vaddr) + ETH_ALEN * 2,
				       e->msg.vec[0].len - ETH_ALEN * 2);
			} else {
				memcpy(es->msg.vec[0].vaddr, e->hw.vaddr,
				       e->msg.vec[0].len);
			}
			put_streaming_entry(e);

		} else {
			es = e;
		}
		stq = es->stq;
		list_move_tail(&es->list, &stq->entryLogQueue);
		stq->entrynum.processed--;
		stq->entrynum.completed++;

		if (hwq->tx) {
			stq->dstats.tx_entry_complete++;
			stq->pstats.tx_packets++;
			stq->pstats.tx_errors += (u64)es->errors;
			stq->pstats.tx_bytes += (u64)es->total_bytes;
		} else {
			stq->dstats.rx_entry_complete++;
			stq->pstats.rx_packets++;
			stq->pstats.rx_errors += (u64)es->errors;
			stq->pstats.rx_bytes += (u64)es->total_bytes;
		}

		if (stq->entrynum.processed == 0 &&
		    stq->state == AVB_STATE_WAITCOMPLETE)
			stq_sequencer(stq, AVB_STATE_IDLE);

		stq_pool[stq->qno] = stq;
	}

	for (i = 0; i < RAVB_STQUEUE_NUM; i++) {
		stq = stq_pool[i];
		if (stq)
			wake_up_interruptible(&stq->waitEvent);
	}

	return progress;
}

static int hwq_task_process_judge(struct hwqueue_info *hwq, bool progress)
{
	struct streaming_private *stp = to_stp(hwq);
	struct net_device *ndev = to_net_dev(stp->device.parent);

	if (list_empty(&hwq->activeStreamQueue)) {
		if (list_empty(&hwq->completeWaitQueue)) {
			hwq_sequencer(hwq, AVB_STATE_IDLE);
		} else {
			hwq_sequencer(hwq, AVB_STATE_WAITCOMPLETE);
			/* enable interrupt */
			ravb_enable_interrupt(ndev, hwq);
		}
	} else {
		if (!progress && list_empty(&hwq->completeWaitQueue)) {
			hwq_sequencer(hwq, AVB_STATE_ACTIVE);
		} else {
			hwq_sequencer(hwq, AVB_STATE_WAITCOMPLETE);
			/* enable interrupt */
			ravb_enable_interrupt(ndev, hwq);
		}
	}

	return 0;
}

/**
 * sequencer task each hwqueue_info
 */
static int ravb_hwq_task(void *param)
{
	struct hwqueue_info *hwq = param;
	struct streaming_private *stp = to_stp(hwq);
	struct net_device *ndev = to_net_dev(stp->device.parent);
	bool progress;
	u32 events;

	while (!kthread_should_stop()) {
		int ret;

		ret = wait_event_interruptible(hwq->waitEvent,
					       hwq->pendingEvents);
		if (ret < 0) {
			pr_err("hwq_task.%d: wait_event error\n", hwq->index);
			continue;
		}

		events = hwq_event_clear(hwq);

		/* unload event */
		if (events & AVB_EVENT_UNLOAD) {
			down(&hwq->sem);
			/* terminate hardware queue */
			hwq_task_process_terminate(hwq);
			up(&hwq->sem);
			break;
		}

		if (events & AVB_EVENT_SWLINK_DOWN) {
			down(&hwq->sem);
			/* terminate hardware queue */
			hwq_task_process_terminate(hwq);
			up(&hwq->sem);
			continue;
		}

		switch (hwq->state) {
		case AVB_STATE_IDLE:
			if (list_empty(&hwq->activeStreamQueue))
				break;
			/* fall through */
		case AVB_STATE_WAITCOMPLETE:
			/* disable interrupt */
			ravb_disable_interrupt(ndev, hwq);
			hwq_sequencer(hwq, AVB_STATE_ACTIVE);
			/* fall through */
		case AVB_STATE_ACTIVE:
			/* TODO implement SW scheduler */
			do {
				down(&hwq->sem);

				/* convert new entry to build desciptor */
				hwq_task_process_encode(hwq);
				/* process completed descriptor by HW */
				progress = hwq_task_process_decode(hwq);
				/* judge hwq Task state */
				hwq_task_process_judge(hwq, progress);

				up(&hwq->sem);
			} while (hwq->state == AVB_STATE_ACTIVE);
			break;
		default:
			break;
		}
	}

	/* TODO finalize task */

	return 0;
}

static enum hrtimer_restart ravb_streaming_timer_handler(struct hrtimer *timer)
{
	struct hwqueue_info *hwq;

	hwq = container_of(timer, struct hwqueue_info, timer);
	if (hwq->state == AVB_STATE_IDLE)
		return HRTIMER_NORESTART;

	hwq_event(hwq, AVB_EVENT_TIMEOUT, hwq->chno);

	return HRTIMER_NORESTART;
}

static irqreturn_t igb_avb_txirq(struct igb_adapter *adapter, int queue)
{
	//queue = 0..1
	struct streaming_private *stp = stp_ptr;
	struct hwqueue_info *hwq;
///XAP MAPPING
	queue = queue ? 0 : 1;	/*reverse order for TX!!! */

	hwq = &stp->hwqueueInfoTable[queue];

	if (!hwq->irqen)
		return IRQ_HANDLED;

//      hwq_event_irq(hwq, AVB_EVENT_TXINT, 0); //param not used !!
	hwq_event(hwq, AVB_EVENT_TXINT, 0);	//param not used !!
	hwq->dstats.tx_interrupts++;
	/* if irq timeout isn't zero, start irq timeout timer */
	hwq_try_to_start_irq_timeout_timer(hwq);
	return IRQ_HANDLED;
}

static irqreturn_t igb_avb_rxirq(struct igb_adapter *adapter, int queue)
{
	//queue = 0..1
	struct streaming_private *stp = stp_ptr;
	struct hwqueue_info *hwq =
	    &stp->hwqueueInfoTable[queue + RAVB_HWQUEUE_TXNUM];

	if (!hwq->irqen)
		return IRQ_HANDLED;

//      hwq_event_irq(hwq, AVB_EVENT_TXINT, 0); //param not used !!
	hwq_event(hwq, AVB_EVENT_RXINT, 0);	//param not used !!
	hwq->dstats.rx_interrupts++;
	/* if irq timeout isn't zero, start irq timeout timer */
	hwq_try_to_start_irq_timeout_timer(hwq);
	return IRQ_HANDLED;
}

/**
 * device resource management callbacks
 */
static void stp_dev_release(struct device *dev)
{
	/* reserved */
}

static void hwq_dev_release(struct device *dev)
{
	/* reserved */
}

/**
 * initialize streaming API
 */
static void ravb_streaming_cleanup(void);

static void igb_linkstat(struct igb_adapter *adapter, bool up)
{
	int i;
	struct streaming_private *stp = stp_ptr;

	pr_debug("link %s\n", up ? "UP" : "DOWN");

	if (up)
		return;
	/* cleanup hwqueue info */
	for (i = 0; i < RAVB_HWQUEUE_NUM; i++) {
		struct hwqueue_info *hwq;

		hwq = &stp->hwqueueInfoTable[i];
		if (hwq->task)
			hwq_event(hwq, AVB_EVENT_SWLINK_DOWN, -1);
	}

	usleep_range(10000, 20000);	//TODO sync!!
}

static int ravb_streaming_init(void)
{
	int err;
	unsigned int i;
	struct net_device *ndev = dev_get_by_name(&init_net, interface);
	struct hwqueue_info *hwq;
	struct device *dev;

	pr_info("init: start(%s)\n", interface);

	/* check supported devices */
	if (!ndev || !is_avb_capable_igb(ndev)) {
		pr_err("unsupport or no network interface\n");
		return -ENODEV;
	}

	stp_ptr = vzalloc(sizeof(*stp_ptr));
	if (!stp_ptr)
		return -ENOMEM;

	stp_ptr->iadapter = netdev_priv(ndev);

	/* create class */
	stp_ptr->avb_class = class_create(THIS_MODULE, "avb");
	if (IS_ERR(stp_ptr->avb_class)) {
		err = PTR_RET(stp_ptr->avb_class);
		pr_err("init: failed to create avb class\n");
		goto no_class;
	}

	/* Initialize separation filter */
	err = separation_filter_initialise(stp_ptr->iadapter);
	if (err) {
		pr_err("init: failed to separation filter initialize\n");
		goto err_initdevice;
	}

	/* initialize streaming private */
	sema_init(&stp_ptr->sem, 1);
	spin_lock_init(&stp_ptr->lock);

	/* create entry cache */
	streaming_entry_cache = kmem_cache_create("avb_entry_cache",
						  sizeof(struct stream_entry),
						  0, 0, NULL);

	/* device initialize */
	dev = &stp_ptr->device;
	device_initialize(dev);
	dev->parent = &ndev->dev;
	dev->class = stp_ptr->avb_class;
	dev_set_drvdata(dev, stp_ptr);
	dev->release = stp_dev_release;
	dev_set_name(dev, "avb_ctrl");

	err = device_add(dev);
	if (err)
		goto err_initstp;

	/* initialize hwqueue info */
	for (i = 0; i < RAVB_HWQUEUE_NUM; i++) {
		hwq = &stp_ptr->hwqueueInfoTable[i];
		hwq->stp = stp_ptr;
		hwq->index = i;
		hwq->tx = (hwq->index < RAVB_HWQUEUE_TXNUM);
		hwq->chno = hwq->index - (RAVB_HWQUEUE_TXNUM * !hwq->tx);
		hwq->state = AVB_STATE_IDLE;
		hwq->ringsize = RAVB_RINGSIZE;
		spin_lock_init(&hwq->hw_lock);
		hwq->irqen = false;

		if (hwq->tx) {
			igb_reset_tx_queue(stp_ptr->iadapter, IGB_QUEUE(hwq));
			igb_enable_tx_queue(stp_ptr->iadapter, IGB_QUEUE(hwq));
		} else {
			igb_reset_rx_queue(stp_ptr->iadapter, IGB_QUEUE(hwq));
			igb_enable_rx_queue(stp_ptr->iadapter, IGB_QUEUE(hwq));
			alloc_rx_mem(hwq);
		}
		/* clear descriptor chain */
		clear_desc(hwq);

		sema_init(&hwq->sem, 1);
		init_waitqueue_head(&hwq->waitEvent);
		INIT_LIST_HEAD(&hwq->activeStreamQueue);
		INIT_LIST_HEAD(&hwq->completeWaitQueue);

		/* device initialize */
		dev = &hwq->device;
		device_initialize(dev);
		dev->parent = &stp_ptr->device;
		dev->class = stp_ptr->avb_class;
		dev_set_drvdata(dev, hwq);
		dev->release = hwq_dev_release;
		dev_set_name(dev, (hwq->tx) ? "avb_tx%d" : "avb_rx%d",
			     hwq->chno);
		err = device_add(dev);
		if (err)
			goto err_inithwqueue;
		hwq->device_add_flag = true;

		hwq->attached = kset_create_and_add("attached", NULL,
						    &hwq->device.kobj);
		if (!hwq->attached) {
			err = -ENOMEM;
			goto err_inithwqueue;
		}

		hwq->task = kthread_run(ravb_hwq_task, hwq, (hwq->tx) ?
					"avb_tx%d" : "avb_rx%d", hwq->chno);
		if (IS_ERR(hwq->task)) {
			pr_err("init: cannot run AVB streaming task\n");
			err = PTR_RET(hwq->task);
			hwq->task = NULL;
			goto err_inithwqueue;
		}

		hrtimer_init(&hwq->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hwq->timer.function = ravb_streaming_timer_handler;
	}

	igb_register_avb_linkstat_cb(stp_ptr->iadapter, igb_linkstat);
	igb_register_avb_txirq(stp_ptr->iadapter, igb_avb_txirq);
	igb_register_avb_rxirq(stp_ptr->iadapter, igb_avb_rxirq);

	pr_info("init: success\n");

	return 0;

 err_inithwqueue:
	for (i = 0; i < RAVB_HWQUEUE_NUM; i++) {
		hwq = &stp_ptr->hwqueueInfoTable[i];
		if (hwq->task)
			kthread_stop(hwq->task);
		if (hwq->attached)
			kset_unregister(hwq->attached);
		if (hwq->device_add_flag)
			device_unregister(&hwq->device);
	}
	device_unregister(&stp_ptr->device);
 err_initstp:
	kmem_cache_destroy(streaming_entry_cache);
 err_initdevice:
	class_destroy(stp_ptr->avb_class);
 no_class:
	vfree(stp_ptr);
	stp_ptr = NULL;

	pr_info("init: failed\n");

	return err;
}

/**
 * cleanup streaming API
 */
static void ravb_streaming_cleanup(void)
{
	unsigned int i, j;
	struct hwqueue_info *hwq;
	struct stream_entry *e, *e1;
	struct streaming_private *stp = stp_ptr;

	pr_info("cleanup: start\n");

	igb_reset_filter(stp->iadapter, 0);
	igb_register_avb_linkstat_cb(stp->iadapter, NULL);
	igb_register_avb_txirq(stp->iadapter, NULL);
	igb_register_avb_rxirq(stp->iadapter, NULL);

	/* cleanup hwqueue info */
	for (i = 0; i < RAVB_HWQUEUE_NUM; i++) {
		hwq = &stp->hwqueueInfoTable[i];
		if (hwq->task) {
			hwq_event(hwq, AVB_EVENT_UNLOAD, -1);
			kthread_stop(hwq->task);
		}

		/* cleanup stream queue info */
		for (j = 0; j < ((hwq->tx) ? RAVB_STQUEUE_NUM : 1); j++)
			if (test_and_clear_bit(j, hwq->stream_map))
				put_stq(hwq->stqueue_table[j]);

		list_for_each_entry_safe(e, e1, &hwq->completeWaitQueue, list)
			put_streaming_entry(e);
		if (hwq->attached)
			kset_unregister(hwq->attached);
		if (hwq->device_add_flag)
			device_unregister(&hwq->device);
	}

	device_unregister(&stp->device);
	class_destroy(stp->avb_class);

	/* destroy entry cache */
	kmem_cache_destroy(streaming_entry_cache);

	stp_ptr = NULL;

	vfree(stp);

	pr_info("cleanup: end\n");
}

module_init(ravb_streaming_init);
module_exit(ravb_streaming_cleanup);

MODULE_AUTHOR("Renesas Electronics Corporation, ADIT GmbH");
MODULE_DESCRIPTION("AVB Streaming Driver for Intel IGB");
MODULE_LICENSE("Dual MIT/GPL");
