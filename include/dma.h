/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2015 Texas Instruments Incorporated, <www.ti.com>
 * Written by Mugunthan V N <mugunthanvnm@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _DMA_H_
#define _DMA_H_

#include <linux/errno.h>
#include <linux/types.h>

/*
 * enum dma_transfer_direction - dma transfer direction indicator
 * @DMA_MEM_TO_MEM: Memcpy mode
 * @DMA_MEM_TO_DEV: From Memory to Device
 * @DMA_DEV_TO_MEM: From Device to Memory
 * @DMA_DEV_TO_DEV: From Device to Device
 */
enum dma_transfer_direction {
	DMA_MEM_TO_MEM,
	DMA_MEM_TO_DEV,
	DMA_DEV_TO_MEM,
	DMA_DEV_TO_DEV,
};

/**
 * TODO: Add DMA description.
 */

struct udevice;

/**
 * struct dma - A handle to (allowing control of) a single DMA.
 *
 * Clients provide storage for DMA handles. The content of the structure is
 * managed solely by the DMA API and DMA drivers. A DMA struct is
 * initialized by "get"ing the DMA struct. The DMA struct is passed to all
 * other DMA APIs to identify which DMA channel to operate upon.
 *
 * @dev: The device which implements the DMA channel.
 * @id: The DMA channel ID within the provider.
 *
 * Currently, the DMA API assumes that a single integer ID is enough to
 * identify and configure any DMA channel for any DMA provider. If this
 * assumption becomes invalid in the future, the struct could be expanded to
 * either (a) add more fields to allow DMA providers to store additional
 * information, or (b) replace the id field with an opaque pointer, which the
 * provider would dynamically allocated during its .of_xlate op, and process
 * during is .request op. This may require the addition of an extra op to clean
 * up the allocation.
 */
struct dma {
	struct udevice *dev;
	/*
	 * Written by of_xlate. We assume a single id is enough for now. In the
	 * future, we might add more fields here.
	 */
	unsigned long id;
};

#if CONFIG_IS_ENABLED(OF_CONTROL) && CONFIG_IS_ENABLED(DMA)
struct phandle_2_cell;
int dma_get_by_index_platdata(struct udevice *dev, int index,
			      struct phandle_2_cell *cells, struct dma *dma);

/**
 * dma_get_by_index - Get/request a DMA by integer index.
 *
 * This looks up and requests a DMA. The index is relative to the client
 * device; each device is assumed to have n DMAs associated with it somehow,
 * and this function finds and requests one of them. The mapping of client
 * device DMA indices to provider DMAs may be via device-tree properties,
 * board-provided mapping tables, or some other mechanism.
 *
 * @dev:	The client device.
 * @index:	The index of the DMA to request, within the client's list of
 *		DMA channels.
 * @dma:	A pointer to a DMA struct to initialize.
 * @return 0 if OK, or a negative error code.
 */
int dma_get_by_index(struct udevice *dev, int index, struct dma *dma);

/**
 * dma_get_by_name - Get/request a DMA by name.
 *
 * This looks up and requests a DMA. The name is relative to the client
 * device; each device is assumed to have n DMAs associated with it somehow,
 * and this function finds and requests one of them. The mapping of client
 * device DMA names to provider DMAs may be via device-tree properties,
 * board-provided mapping tables, or some other mechanism.
 *
 * @dev:	The client device.
 * @name:	The name of the DMA to request, within the client's list of
 *		DMA channels.
 * @dma:	A pointer to a DMA struct to initialize.
 * @return 0 if OK, or a negative error code.
 */
int dma_get_by_name(struct udevice *dev, const char *name, struct dma *dma);
#else
static inline int dma_get_by_index(struct udevice *dev, int index,
				   struct dma *dma)
{
	return -ENOSYS;
}

static inline int dma_get_by_name(struct udevice *dev, const char *name,
			   struct dma *dma)
{
	return -ENOSYS;
}
#endif

/**
 * dma_request - Request a DMA by provider-specific ID.
 *
 * This requests a DMA using a provider-specific ID. Generally, this function
 * should not be used, since dma_get_by_index/name() provide an interface that
 * better separates clients from intimate knowledge of DMA providers.
 * However, this function may be useful in core SoC-specific code.
 *
 * @dev: The DMA provider device.
 * @dma: A pointer to a DMA struct to initialize. The caller must
 *	 have already initialized any field in this struct which the
 *	 DMA provider uses to identify the DMA channel.
 * @return 0 if OK, or a negative error code.
 */
int dma_request(struct udevice *dev, struct dma *dma);

/**
 * dma_free - Free a previously requested DMA.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @return 0 if OK, or a negative error code.
 */
int dma_free(struct dma *dma);

/**
 * dma_enable() - Enable (turn on) a DMA channel.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @return zero on success, or -ve error code.
 */
int dma_enable(struct dma *dma);

/**
 * dma_disable() - Disable (turn off) a DMA channel.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @return zero on success, or -ve error code.
 */
int dma_disable(struct dma *dma);

/**
 * dma_memcpy() - Try to use DMA to do a mem copy which will be much faster
 *		  than CPU mem copy.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @dst: The destination pointer.
 * @src: The source pointer.
 * @len: Length of the data to be copied.
 * @return zero on success, or -ve error code.
 */
int dma_memcpy(struct dma *dma, void *dst, void *src, size_t len);

/**
 * dma_receive() - Receive a DMA transfer.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @dst: The destination pointer.
 * @return zero on success, or -ve error code.
 */
int dma_receive(struct dma *dma, void **dst);

/**
 * dma_send() - Send a DMA transfer.
 *
 * @dma: A DMA struct that was previously successfully requested by
 *	 dma_request/get_by_*().
 * @src: The source pointer.
 * @len: Length of the data to be copied.
 * @return zero on success, or -ve error code.
 */
int dma_send(struct dma *dma, void *src, size_t len);

#endif /* _DMA_H_ */
