/*
 * This file is part of the Anlogic DMA IP Core driver for Linux
 *
 * Copyright (c) 2020-present,  anlogic, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#ifndef __ANLOGIC_RING_H__
#define __ANLOGIC_RING_H__
/**
 * @file
 * @brief This file contains the declarations for sg dma bd ring
 *
 */
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/cpuset.h>
#include <linux/signal.h>

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/errno.h>

#define AL_DEFAULT_RXD       512 /* BD RING DEPETH */
#define AL_DEFAULT_TXD       512

#define AL_RXBUFFER_2K       2048

struct al_tx_queue_stats {
    u64 restart_queue;
    u64 tx_busy;
    u64 tx_done_old;
};

struct al_rx_queue_stats {
    u64 rsc_count;
    u64 rsc_flush;
    u64 non_eop_descs;
    u64 alloc_rx_page;
    u64 alloc_rx_page_failed;
    u64 alloc_rx_buff_failed;
    u64 csum_err;
};

struct al_tx_buffer {
    struct sgdma_desc desc;
    
};

struct al_rx_buffer {
    struct sgdma_desc *desc;
    void *data; /* dst data buf virt addr */
    dma_addr_t dma; /* dst data buf phy addr */
};

/* Describes a (SG DMA) bd ring for the engine */
struct al_ring {
    struct list_head entry;     /* queue of non-completed transfers */
    struct sgdma_desc *desc_virt;    /* virt addr of the 1st descriptor */
    struct sgdma_result *res_virt; /* virt addr of result for c2h streaming */
    dma_addr_t res_bus;           /* bus addr for result descriptors */
    dma_addr_t desc_bus;        /* bus addr of the first descriptor */
    int desc_adjacent;      /* adjacent descriptors at desc_bus */
    int desc_num;           /* number of descriptors in transfer */
    int desc_index;         /* index for first descriptor in transfer */
    int desc_cmpl_th;       /* completed descriptor threshold */

    struct device *dev;     /* device for DMA mapping */
    void *desc;         /* descriptor ring memory */

    union {
        struct al_tx_buffer *tx_buffer_info;
        struct al_rx_buffer *rx_buffer_info;
    };
    unsigned long state;
    u8 __iomem *tail;
    dma_addr_t dma;         /* phys. address of descriptor ring */
    unsigned int size;      /* length in bytes */

    union {
        struct al_tx_queue_stats tx_stats;
        struct al_rx_queue_stats rx_stats;
    };

    enum dma_data_direction dir;

    u16 count;          /* amount of descriptors */

    u16 next_to_use;
    u16 next_to_clean;
    u16 rx_buf_len;

    u16 ring_idx;           /* {rx,tx,xdp}_ring back reference idx */
};

int al_clean_rx_event(struct anlogic_dma_engine *engine, struct al_ring *rx_ring,
                   int budget);

void al_alloc_rx_buffers(struct al_ring *rx_ring, u16 cleaned_count);

int engine_service_cyclic(struct anlogic_dma_engine *engine);

void engine_transfer_dequeue(struct anlogic_dma_engine *engine);

int sgdma_cyclic_transfer_setup(struct anlogic_dma_engine *engine);

void sgt_free_with_pages(struct sg_table *sgt, int dir,
                struct pci_dev *pdev);

int sgt_alloc_with_pages(struct sg_table *sgt, unsigned int npages,
                int dir, struct pci_dev *pdev);

ssize_t sgdma_engine_read_cyclic(struct anlogic_dma_engine *engine, char __user *buf,
               size_t count, int timeout_ms);

#endif /* #ifndef __ANLOGIC_RING_H__ */
