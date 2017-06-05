/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2015 Texas Instruments Incorporated, <www.ti.com>
 * Written by Mugunthan V N <mugunthanvnm@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <dma.h>
#include <dma-uclass.h>
#include <dt-structs.h>
#include <errno.h>

DECLARE_GLOBAL_DATA_PTR;

static inline struct dma_ops *dma_dev_ops(struct udevice *dev)
{
	return (struct dma_ops *)dev->driver->ops;
}

#if CONFIG_IS_ENABLED(OF_CONTROL)
# if CONFIG_IS_ENABLED(OF_PLATDATA)
int dma_get_by_index_platdata(struct udevice *dev, int index,
			      struct phandle_2_cell *cells, struct dma *dma)
{
	int ret;

	if (index != 0)
		return -ENOSYS;
	ret = uclass_get_device(UCLASS_DMA, 0, &dma->dev);
	if (ret)
		return ret;
	dma->id = cells[0].id;

	return 0;
}
# else
static int dma_of_xlate_default(struct dma *dma,
				struct fdtdec_phandle_args *args)
{
	debug("%s(dma=%p)\n", __func__, dma);

	if (args->args_count > 1) {
		debug("Invaild args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	if (args->args_count)
		dma->id = args->args[0];
	else
		dma->id = 0;

	return 0;
}

int dma_get_by_index(struct udevice *dev, int index, struct dma *dma)
{
	int ret;
	struct fdtdec_phandle_args args;
	struct udevice *dev_dma;
	struct dma_ops *ops;

	debug("%s(dev=%p, index=%d, dma=%p)\n", __func__, dev, index, dma);

	assert(dma);
	ret = fdtdec_parse_phandle_with_args(gd->fdt_blob, dev_of_offset(dev),
					     "dmas", "#dma-cells", 0, index,
					     &args);
	if (ret) {
		debug("%s: fdtdec_parse_phandle_with_args failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_of_offset(UCLASS_DMA, args.node, &dev_dma);
	if (ret) {
		debug("%s: uclass_get_device_by_of_offset failed: err=%d\n",
		      __func__, ret);
		return ret;
	}

	dma->dev = dev_dma;

	ops = dma_dev_ops(dev_dma);

	if (ops->of_xlate)
		ret = ops->of_xlate(dma, &args);
	else
		ret = dma_of_xlate_default(dma, &args);
	if (ret) {
		debug("of_xlate() failed: %d\n", ret);
		return ret;
	}

	return dma_request(dev_dma, dma);
}
# endif /* OF_PLATDATA */

int dma_get_by_name(struct udevice *dev, const char *name, struct dma *dma)
{
	int index;

	debug("%s(dev=%p, name=%s, dma=%p)\n", __func__, dev, name, dma);

	index = fdt_stringlist_search(gd->fdt_blob, dev_of_offset(dev),
				      "dma-names", name);
	if (index < 0) {
		debug("fdt_stringlist_search() failed: %d\n", index);
		return index;
	}

	return dma_get_by_index(dev, index, dma);
}
#endif /* OF_CONTROL */

int dma_request(struct udevice *dev, struct dma *dma)
{
	struct dma_ops *ops = dma_dev_ops(dev);

	debug("%s(dev=%p, dma=%p)\n", __func__, dev, dma);

	dma->dev = dev;

	if (!ops->request)
		return 0;

	return ops->request(dma);
}

int dma_free(struct dma *dma)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->free)
		return 0;

	return ops->free(dma);
}

int dma_enable(struct dma *dma)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->enable)
		return -ENOSYS;

	return ops->enable(dma);
}

int dma_disable(struct dma *dma)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->disable)
		return -ENOSYS;

	return ops->disable(dma);
}

int dma_memcpy(struct dma *dma, void *dst, void *src, size_t len)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->transfer)
		return -ENOSYS;

	/* Invalidate the area, so no writeback into the RAM races with DMA */
	invalidate_dcache_range((unsigned long)dst, (unsigned long)dst +
				roundup(len, ARCH_DMA_MINALIGN));

	return ops->transfer(dma, DMA_MEM_TO_MEM, dst, src, len);
}

int dma_receive(struct dma *dma, void **dst)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->receive)
		return -1;

	return ops->receive(dma, dst);
}

int dma_send(struct dma *dma, void *src, size_t len)
{
	struct dma_ops *ops = dma_dev_ops(dma->dev);

	debug("%s(dma=%p)\n", __func__, dma);

	if (!ops->send)
		return -1;

	return ops->send(dma, src, len);
}

UCLASS_DRIVER(dma) = {
	.id = UCLASS_DMA,
	.name = "dma",
	.flags = DM_UC_FLAG_SEQ_ALIAS,
};
