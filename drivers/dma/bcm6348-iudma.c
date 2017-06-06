/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * Derived from linux/drivers/dma/bcm63xx-iudma.c:
 *	Copyright (C) 2015 Simon Arlott <simon@fire.lp0.eu>
 *
 * Derived from linux/drivers/net/ethernet/broadcom/bcm63xx_enet.c:
 *	Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *
 * Derived from bcm963xx_4.12L.06B_consumer/shared/opensource/include/bcm963xx/63268_map_part.h:
 *	Copyright (C) 2000-2010 Broadcom Corporation
 *
 * Derived from bcm963xx_4.12L.06B_consumer/bcmdrivers/opensource/net/enet/impl4/bcmenet.c:
 *	Copyright (C) 2010 Broadcom Corporation
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dma-uclass.h>
#include <memalign.h>
#include <net.h>
#include <reset.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#define ALIGN_END_ADDR(type, ptr, size)	\
	((unsigned long)(ptr) + roundup((size) * sizeof(type), ARCH_DMA_MINALIGN))

#define DMA_MAX_BURST_LENGTH	16

/* DMA Channels */
#define DMA_CHAN_FLOWC(x)		((x) >> 1)
#define DMA_CHAN_FLOWC_MAX		8
#define DMA_CHAN_MAX			16
#define DMA_CHAN_SIZE			0x10
#define DMA_CHAN_TOUT			500

/* DMA Global Configuration register */
#define DMA_CFG_REG			0x00
#define DMA_CFG_ENABLE_SHIFT		0
#define DMA_CFG_ENABLE_MASK		(1 << DMA_CFG_ENABLE_SHIFT)
#define DMA_CFG_FLOWC_ENABLE(x)		BIT(DMA_CHAN_FLOWC(x) + 1)
#define DMA_CFG_NCHANS_SHIFT		24
#define DMA_CFG_NCHANS_MASK		(0xf << DMA_CFG_NCHANS_SHIFT)

/* DMA Global Flow Control Threshold registers */
#define DMA_FLOWC_THR_LO_REG(x)		(0x04 + DMA_CHAN_FLOWC(x) * 0x0c)
#define DMA_FLOWC_THR_LO_SHIFT		0
#define DMA_FLOWC_THR_LO_MASK		(5 << DMA_FLOWC_THR_LO_SHIFT)

#define DMA_FLOWC_THR_HI_REG(x)		(0x08 + DMA_CHAN_FLOWC(x) * 0x0c)
#define DMA_FLOWC_THR_HI_SHIFT		0
#define DMA_FLOWC_THR_HI_MASK		(10 << DMA_FLOWC_THR_HI_SHIFT)

/* DMA Global Flow Control Buffer Allocation registers */
#define DMA_FLOWC_ALLOC_REG(x)		(0x0c + DMA_CHAN_FLOWC(x) * 0x0c)
#define DMA_FLOWC_ALLOC_FORCE_SHIFT	31
#define DMA_FLOWC_ALLOC_FORCE_MASK	(1 << DMA_FLOWC_ALLOC_FORCE_SHIFT)

/* DMA Global Reset register */
#define DMA_RST_REG			0x34
#define DMA_RST_CHAN_SHIFT		0
#define DMA_RST_CHAN_MASK(x)		(1 << x)

/* DMA Channel Configuration register */
#define DMAC_CFG_REG(x)			(DMA_CHAN_SIZE * (x) + 0x00)
#define DMAC_CFG_ENABLE_SHIFT		0
#define DMAC_CFG_ENABLE_MASK		(1 << DMAC_CFG_ENABLE_SHIFT)
#define DMAC_CFG_PKT_HALT_SHIFT		1
#define DMAC_CFG_PKT_HALT_MASK		(1 << DMAC_CFG_PKT_HALT_SHIFT)
#define DMAC_CFG_BRST_HALT_SHIFT	2
#define DMAC_CFG_BRST_HALT_MASK		(1 << DMAC_CFG_BRST_HALT_SHIFT)

/* DMA Channel Interrupts registers */
#define DMAC_IR_ST_REG(x)		(DMA_CHAN_SIZE * (x) + 0x04)
#define DMAC_IR_EN_REG(x)		(DMA_CHAN_SIZE * (x) + 0x08)

#define DMAC_IR_DONE_SHIFT		2
#define DMAC_IR_DONE_MASK		(1 << DMAC_IR_DONE_SHIFT)

/* DMA Channel Max Burst Length register */
#define DMAC_BURST_REG(x)		(DMA_CHAN_SIZE * (x) + 0x0c)
#define DMAC_BURST_MAX_SHIFT		0
#define DMAC_BURST_MAX_MASK		(16 << DMAC_BURST_MAX_SHIFT)

/* DMA SRAM Descriptor Ring Start register */
#define DMAS_RSTART_REG(x)		(DMA_CHAN_SIZE * (x) + 0x00)

/* DMA SRAM State/Bytes done/ring offset register */
#define DMAS_STATE_DATA_REG(x)		(DMA_CHAN_SIZE * (x) + 0x04)

/* DMA SRAM Buffer Descriptor status and length register */
#define DMAS_DESC_LEN_STATUS_REG(x)	(DMA_CHAN_SIZE * (x) + 0x08)

/* DMA SRAM Buffer Descriptor status and length register */
#define DMAS_DESC_BASE_BUFPTR_REG(x)	(DMA_CHAN_SIZE * (x) + 0x0c)

struct bcm6348_dma_desc {
	uint16_t length;

	uint16_t status;
#define DMAD_ST_CRC_SHIFT	8
#define DMAD_ST_CRC_MASK	(1 << DMAD_ST_CRC_SHIFT)
#define DMAD_ST_WRAP_SHIFT	12
#define DMAD_ST_WRAP_MASK	(1 << DMAD_ST_WRAP_SHIFT)
#define DMAD_ST_SOP_SHIFT	13
#define DMAD_ST_SOP_MASK	(1 << DMAD_ST_SOP_SHIFT)
#define DMAD_ST_EOP_SHIFT	14
#define DMAD_ST_EOP_MASK	(1 << DMAD_ST_EOP_SHIFT)
#define DMAD_ST_OWN_SHIFT	15
#define DMAD_ST_OWN_MASK	(1 << DMAD_ST_OWN_SHIFT)

	uint32_t address;
} __attribute__((packed, aligned(1)));

struct bcm6348_chan_priv {
	void __iomem *dma_buff;
	uint32_t dma_buff_size;
	void __iomem *dma_ring;
	uint8_t dma_ring_size;
	uint8_t desc_id;
};

struct bcm6348_iudma_priv {
	void __iomem *base;
	void __iomem *chan;
	void __iomem *sram;
	struct bcm6348_chan_priv **ch_priv;
	uint8_t n_channels;
};

static bool bcm6348_iudma_chan_is_rx(uint8_t ch)
{
	return !(ch & 1);
}

static void bcm6348_iudma_chan_stop(struct bcm6348_iudma_priv *priv,
				    uint8_t ch)
{
	unsigned int timeout = DMA_CHAN_TOUT;

	/* disable dma channel interrupts */
	writel_be(0, priv->chan + DMAC_IR_EN_REG(ch));

	do {
		uint32_t cfg, halt;

		if (timeout > DMA_CHAN_TOUT / 2)
			halt = DMAC_CFG_PKT_HALT_MASK;
		else
			halt = DMAC_CFG_BRST_HALT_MASK;

		/* try to stop dma channel */
		writel_be(halt, priv->chan + DMAC_CFG_REG(ch));
		mb();

		/* check if channel was stopped */
		cfg = readl_be(priv->chan + DMAC_CFG_REG(ch));
		if (!(cfg & DMAC_CFG_ENABLE_MASK))
			break;

		udelay(1);
	} while (--timeout);

	if (!timeout)
		printf("unable to stop channel %u\n", ch);

	/* reset dma channel */
	setbits_be32(priv->base + DMA_RST_REG, DMA_RST_CHAN_MASK(ch));
	clrbits_be32(priv->base + DMA_RST_REG, DMA_RST_CHAN_MASK(ch));
}

static int bcm6348_iudma_disable(struct dma *dma)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);

	printf("%s: ch=%lu priv=%p\n", __func__, dma->id, priv);

	return 0;
}

static int bcm6348_iudma_enable(struct dma *dma)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);

	printf("%s: ch=%lu priv=%p\n", __func__, dma->id, priv);

	setbits_be32(priv->chan + DMAC_CFG_REG(dma->id), DMAC_CFG_ENABLE_MASK);

	return 0;
}

static int bcm6348_iudma_free(struct dma *dma)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);

	printf("%s: ch=%lu priv=%p\n", __func__, dma->id, priv);

	return 0;
}

static int bcm6348_iudma_request(struct dma *dma)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);
	struct bcm6348_chan_priv *ch_priv;
	struct bcm6348_dma_desc *dma_desc;
	uint8_t i;

	printf("%s: ch=%lu priv=%p\n", __func__, dma->id, priv);

	/* check if channel is valid */
	if (dma->id >= priv->n_channels)
		return -ENODEV;

	/* alloc channel private data */
	priv->ch_priv[dma->id] = kzalloc(sizeof(struct bcm6348_chan_priv), GFP_KERNEL);
	if (!priv->ch_priv[dma->id])
		return -ENOMEM;
	ch_priv = priv->ch_priv[dma->id];
#if DEBUG
	printf("%s: ch=%lu ch_priv=%p\n", __func__, dma->id, ch_priv);
#endif

	/* alloc dma ring */
	if (bcm6348_iudma_chan_is_rx(dma->id))
		ch_priv->dma_ring_size = PKTBUFSRX;
	else
		ch_priv->dma_ring_size = 1;
	ch_priv->dma_ring = malloc_cache_aligned(sizeof(struct bcm6348_dma_desc) * ch_priv->dma_ring_size);
	if (!ch_priv->dma_ring)
		return -ENOMEM;
#if DEBUG
	printf("%s: ch=%lu dma_ring_size=%u dma_ring=%p\n", __func__, dma->id, ch_priv->dma_ring_size, ch_priv->dma_ring);
#endif

	/* alloc dma buffer */
	ch_priv->dma_buff_size = PKTSIZE_ALIGN * ch_priv->dma_ring_size;
	ch_priv->dma_buff = malloc_cache_aligned(ch_priv->dma_buff_size);
	if (!ch_priv->dma_buff)
		return -ENOMEM;
#if DEBUG
	printf("%s: ch=%lu dma_buff_size=%u dma_buff=%p\n", __func__, dma->id, ch_priv->dma_buff_size, ch_priv->dma_buff);
#endif
	memset(ch_priv->dma_buff, 0, ch_priv->dma_buff_size);

	/* init dma rings */
	dma_desc = ch_priv->dma_ring;
	for (i = 0; i < ch_priv->dma_ring_size; i++) {
		if (bcm6348_iudma_chan_is_rx(dma->id)) {
			dma_desc->status = DMAD_ST_OWN_MASK;
			dma_desc->length = PKTSIZE_ALIGN;
		} else {
			dma_desc->status = 0;
			dma_desc->length = 0;
		}

		dma_desc->address = virt_to_phys(ch_priv->dma_buff +
						 PKTSIZE_ALIGN * i);

		if (i == ch_priv->dma_ring_size - 1)
			dma_desc->status |= DMAD_ST_WRAP_MASK;

#if DEBUG
		printf("%s: ch=%lu ring=%u status=%04x len=%u addr=%08x\n", __func__, dma->id, i, dma_desc->status, dma_desc->length, dma_desc->address);
#endif

		dma_desc += sizeof(struct bcm6348_dma_desc);
	}

	/* init to first descriptor */
	ch_priv->desc_id = 0;

	/* force cache writeback */
	flush_dcache_range((ulong)ch_priv->dma_ring,
			   (ulong)ch_priv->dma_ring +
			   sizeof(struct bcm6348_dma_desc) *
			   ch_priv->dma_ring_size);

	/* clear sram */
	writel_be(0, priv->sram + DMAS_STATE_DATA_REG(dma->id));
	writel_be(0, priv->sram + DMAS_DESC_LEN_STATUS_REG(dma->id));
	writel_be(0, priv->sram + DMAS_DESC_BASE_BUFPTR_REG(dma->id));

	/* set dma ring start */
	writel_be(virt_to_phys(ch_priv->dma_ring),
		  priv->sram + DMAS_RSTART_REG(dma->id));

	/* set flow control */
	if (bcm6348_iudma_chan_is_rx(dma->id)) {
		setbits_be32(priv->base + DMA_CFG_REG, DMA_CFG_FLOWC_ENABLE(dma->id));
		writel_be(DMA_FLOWC_THR_LO_MASK, priv->base + DMA_FLOWC_THR_LO_REG(dma->id));
		writel_be(DMA_FLOWC_THR_HI_MASK, priv->base + DMA_FLOWC_THR_HI_REG(dma->id));
		writel_be(0, priv->base + DMA_FLOWC_ALLOC_REG(dma->id));
	}

	/* set dma max burst */
	writel_be(DMA_MAX_BURST_LENGTH, priv->chan + DMAC_BURST_REG(dma->id));

	/* clear interrupts */
	writel_be(DMAC_IR_DONE_MASK, priv->chan + DMAC_IR_ST_REG(dma->id));
	writel_be(0, priv->chan + DMAC_IR_EN_REG(dma->id));

	return 0;
}

static int bcm6348_iudma_receive(struct dma *dma, void **dst)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);
	struct bcm6348_chan_priv *ch_priv = priv->ch_priv[dma->id];
	struct bcm6348_dma_desc *dma_desc;
	void __iomem *dma_buff;
	uint16_t status;
	int ret;

#if DEBUG
	printf("%s: ch=%lu priv=%p ch_priv=%p dst=%p\n", __func__, dma->id, priv, ch_priv, dst);
#endif

	/* get dma ring descriptor address */
	dma_desc = ch_priv->dma_ring;
	dma_desc += ch_priv->desc_id * (sizeof(struct bcm6348_dma_desc));

	/* invalidate cache data */
	invalidate_dcache_range((ulong)dma_desc, ALIGN_END_ADDR(struct bcm6348_dma_desc, dma_desc, 1));

	/* check dma own */
	if (dma_desc->status & DMAD_ST_OWN_MASK)
		return 0;

	/* check dma end */
	if (!(dma_desc->status & DMAD_ST_EOP_MASK))
		return -EINVAL;

	/* get dma buff descriptor address */
	dma_buff = phys_to_virt(dma_desc->address);

	/* invalidate cache data */
	invalidate_dcache_range((ulong)dma_buff, (ulong)(dma_buff + PKTSIZE_ALIGN));

	/* get dma data */
	dst = dma_buff;
	ret = dma_desc->length;

	printf("%s: received packet %u\n", __func__, ch_priv->desc_id);
	print_buffer((ulong)dma_buff, dma_buff, 1, dma_desc->length, 0);

	/* reinit dma descriptor */
	status = dma_desc->status & DMAD_ST_WRAP_MASK;
	status |= DMAD_ST_OWN_MASK;

	dma_desc->length = PKTSIZE_ALIGN;
	wmb();
	dma_desc->status = status;
	wmb();

	/* flush cache */
	flush_dcache_range((ulong)dma_desc, ALIGN_END_ADDR(struct bcm6348_dma_desc, dma_desc, 1));

	/* set flow control buffer alloc */
	writel_be(1, priv->base + DMA_FLOWC_ALLOC_REG(dma->id));

	/* enable dma */
	setbits_be32(priv->chan + DMAC_CFG_REG(dma->id), DMAC_CFG_ENABLE_MASK);

	/* set interrupt */
	writel_be(DMAC_IR_DONE_MASK, priv->chan + DMAC_IR_EN_REG(dma->id));

	/* increment dma descriptor */
	ch_priv->desc_id++;
	if (ch_priv->desc_id >= ch_priv->dma_ring_size)
		ch_priv->desc_id = 0;

	return ret;
}

static int bcm6348_iudma_send(struct dma *dma, void *src, size_t len)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dma->dev);
	struct bcm6348_chan_priv *ch_priv = priv->ch_priv[dma->id];
	struct bcm6348_dma_desc *dma_desc;
	void __iomem *dma_buff;
	uint16_t val;

	printf("%s: ch=%lu priv=%p ch_priv=%p src=%p len=%x\n", __func__, dma->id, priv, ch_priv, src, len);

	/* get dma ring descriptor address */
	dma_desc = ch_priv->dma_ring;
	dma_desc += ch_priv->desc_id * (sizeof(struct bcm6348_dma_desc));

	/* config dma descriptor */
#if 0
	val = (DMAD_ST_OWN_MASK |
	       DMAD_ST_EOP_MASK |
	       DMAD_ST_CRC_MASK |
	       DMAD_ST_SOP_MASK);
#else
	val = DMAD_ST_OWN_MASK;
#endif
	if (ch_priv->desc_id == ch_priv->dma_ring_size - 1)
		val |= DMAD_ST_WRAP_MASK;

	dma_desc->length = len;
	wmb();
	dma_desc->status = val;
	wmb();

	/* flush cache */
	flush_dcache_range((ulong)dma_desc, ALIGN_END_ADDR(struct bcm6348_dma_desc, dma_desc, 1));

	/* get dma buff descriptor address */
	dma_buff = phys_to_virt(dma_desc->address);

#if DEBUG
	print_buffer((ulong)dma_buff, dma_buff, 1, PKTSIZE_ALIGN, 0);
#endif

	/* copy buffer */
	memcpy(dma_buff, src, len);

	/* flush cache */
	flush_dcache_range((ulong)dma_buff, (ulong)dma_buff + PKTSIZE_ALIGN);

	/* enable dma */
	setbits_be32(priv->chan + DMAC_CFG_REG(dma->id), DMAC_CFG_ENABLE_MASK);

	/* set interrupt */
	writel_be(DMAC_IR_DONE_MASK, priv->chan + DMAC_IR_EN_REG(dma->id));

	/* poll dma status */
	do {
		/* invalidate cache */
		invalidate_dcache_range((ulong)dma_desc, ALIGN_END_ADDR(struct bcm6348_dma_desc, dma_desc, 1));

		if (!(dma_desc->status & DMAD_ST_OWN_MASK))
			break;
	} while(1);

	/* increment dma descriptor */
	ch_priv->desc_id++;
	if (ch_priv->desc_id >= ch_priv->dma_ring_size)
		ch_priv->desc_id = 0;

	return 0;
}

static const struct dma_ops bcm6348_iudma_ops = {
	.disable = bcm6348_iudma_disable,
	.enable = bcm6348_iudma_enable,
	.free = bcm6348_iudma_free,
	.request = bcm6348_iudma_request,
	.receive = bcm6348_iudma_receive,
	.send = bcm6348_iudma_send,
};

static const struct udevice_id bcm6348_iudma_ids[] = {
	{ .compatible = "brcm,bcm6348-iudma", },
	{ /* sentinel */ }
};

static int bcm6348_iudma_probe(struct udevice *dev)
{
	struct bcm6348_iudma_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	uint8_t ch;
	int i;

	/* try to enable clocks */
	for (i = 0; ; i++) {
		struct clk clk;
		int ret;

		ret = clk_get_by_index(dev, i, &clk);
		if (ret < 0)
			break;
		if (clk_enable(&clk))
			printf("failed to enable clock %d\n", i);
		clk_free(&clk);
	}

	/* try to perform resets */
	for (i = 0; ; i++) {
		struct reset_ctl reset;
		int ret;

		ret = reset_get_by_index(dev, i, &reset);
		if (ret < 0)
			break;
		if (reset_deassert(&reset))
			printf("failed to deassert reset %d\n", i);
		reset_free(&reset);
	}

	/* dma global base address */
	addr = dev_get_addr_name(dev, "dma");
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	priv->base = ioremap(addr, 0);

	/* dma channels base address */
	addr = dev_get_addr_name(dev, "dma-channels");
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	priv->chan = ioremap(addr, 0);

	/* dma sram base address */
	addr = dev_get_addr_name(dev, "dma-sram");
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	priv->sram = ioremap(addr, 0);

	/* disable dma controller */
	clrbits_be32(priv->base + DMA_CFG_REG, DMA_CFG_ENABLE_MASK);

	/* get number of channels */
	priv->n_channels = fdtdec_get_uint(gd->fdt_blob, dev_of_offset(dev),
					   "dma-channels", 8);
	if (priv->n_channels > DMA_CHAN_MAX)
		return -EINVAL;

	/* alloc channel private data pointers */
	priv->ch_priv = kzalloc(sizeof(struct bcm6348_chan_priv*) *
				priv->n_channels, GFP_KERNEL);
	if (!priv->ch_priv)
		return -ENOMEM;

	/* stop dma channels */
	for (ch = 0; ch < priv->n_channels; ch++)
		bcm6348_iudma_chan_stop(priv, ch);

	/* enable dma controller */
	setbits_be32(priv->base + DMA_CFG_REG, DMA_CFG_ENABLE_MASK);

	return 0;
}

U_BOOT_DRIVER(bcm6348_iudma) = {
	.name = "bcm6348_iudma",
	.id = UCLASS_DMA,
	.of_match = bcm6348_iudma_ids,
	.ops = &bcm6348_iudma_ops,
	.priv_auto_alloc_size = sizeof(struct bcm6348_iudma_priv),
	.probe = bcm6348_iudma_probe,
};
