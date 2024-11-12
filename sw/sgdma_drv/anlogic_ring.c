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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>

#include "anlogic_pci_lib.h"
#include "anlogic_pci_dbg.h"
#include "anlogic_thread.h"
#include "anlogic_ring.h"

#define AL_RING_RECIVE_WITH_EOP 1

extern unsigned int poll_mode;
extern unsigned int enable_st_c2h_credit;
extern unsigned int enable_st_h2c_credit;

void sgt_free_with_pages(struct sg_table *sgt, int dir,
                struct pci_dev *pdev)
{
    struct scatterlist *sg = sgt->sgl;
    int npages = sgt->orig_nents;
    int i;

    for (i = 0; i < npages; i++, sg = sg_next(sg)) {
        struct page *pg = sg_page(sg);
        dma_addr_t bus = sg_dma_address(sg);

        if (pg) {
            if (pdev)
                pci_unmap_page(pdev, bus, PAGE_SIZE, dir);
            __free_page(pg);
        } else
            break;
    }
    sg_free_table(sgt);
    memset(sgt, 0, sizeof(struct sg_table));
}

/* alloc sg mem for transfer. */
int sgt_alloc_with_pages(struct sg_table *sgt, unsigned int npages,
                int dir, struct pci_dev *pdev)
{
    struct scatterlist *sg;
    int i;

    if (sg_alloc_table(sgt, npages, GFP_KERNEL)) {
        pr_info("sgt OOM.\n");
        return -ENOMEM;
    }

    sg = sgt->sgl;
    for (i = 0; i < npages; i++, sg = sg_next(sg)) {
        struct page *pg = alloc_page(GFP_KERNEL);

            if (!pg) {
            pr_info("%d/%u, page OOM.\n", i, npages);
            goto err_out;
        }

        if (pdev) {
            dma_addr_t bus = pci_map_page(pdev, pg, 0, PAGE_SIZE, dir);
            if (unlikely(pci_dma_mapping_error(pdev, bus))) {
                pr_info("%d/%u, page 0x%p map err.\n", i, npages, pg);
                __free_page(pg);
                goto err_out;
            }

            sg_dma_address(sg) = bus;
            sg_dma_len(sg) = PAGE_SIZE;
        }
        sg_set_page(sg, pg, PAGE_SIZE, 0);
    }

    sgt->orig_nents = sgt->nents = npages;

    return 0;

err_out:
    sgt_free_with_pages(sgt, dir, pdev);
    return -ENOMEM; 
}

/*
 * !NOTE! reference/demo purpose only 
 * sgdma_cyclic_transfer_setup is used for streaming C2H transfers:
 * - A list of buffers are pre-allocated for incoming streaming data
 * - the ring of the buffers is allowed to wrap around
 */
int sgdma_cyclic_transfer_setup(struct anlogic_dma_engine *engine)
{
    struct anlogic_dev *sgdev;
    struct sgdma_transfer *xfer;
    unsigned long flags;
    int i;
    int rc;

    BUG_ON(!engine);
    sgdev = engine->sgdev;
    BUG_ON(!sgdev);

    if (engine->cyclic_req) {
        pr_info("%s: exclusive access already taken.\n",
            engine->name);
        return -EBUSY;
    }

    //spin_lock_irqsave(&engine->lock, flags);

    engine->rx_tail = 0;
    engine->rx_head = 0;
    engine->rx_overrun = 0;
    engine->eop_found = 0;

    rc = sgt_alloc_with_pages(&engine->cyclic_sgt, CYCLIC_RX_PAGES_MAX,
                engine->dir, sgdev->pdev);
    if (rc < 0) {
        pr_info("%s cyclic pages %u OOM.\n",
            engine->name, CYCLIC_RX_PAGES_MAX);
        goto err_out;
    }

    engine->cyclic_req = sgdma_init_request(&engine->cyclic_sgt, 0);
    if (!engine->cyclic_req) {
        pr_info("%s cyclic request OOM.\n", engine->name);
        rc = -ENOMEM;
        goto err_out;
    }

#ifdef __LIBSGDMA_DEBUG__
    sgdma_request_cb_dump(engine->cyclic_req);
#endif

    xfer = &engine->cyclic_req->tfer[0];

    if (engine->dir == DMA_FROM_DEVICE)
        memset(engine->cyclic_result, 0,
            CYCLIC_RX_PAGES_MAX * sizeof(struct sgdma_result));

    rc = transfer_init(engine, engine->cyclic_req, xfer);
    if (rc < 0)
        goto err_out;

    /* set control of all descriptors, clear stop flag for last desc. */
    for (i = 0; i < xfer->desc_num; i++) {
        sgdma_desc_control_clear(xfer->desc_virt + i, LS_BYTE_MASK);
        sgdma_desc_control_set(xfer->desc_virt + i,
         SGDMA_DESC_EOP | SGDMA_DESC_COMPLETED);
    }

    /*
     * make this a cyclic transfer
     * link last descriptor to first descriptor
     */
    sgdma_desc_link(xfer->desc_virt + xfer->desc_num - 1, xfer->desc_virt, xfer->desc_bus);

    /* remember transfer is cyclic */
    xfer->cyclic = 1;

#ifdef __LIBSGDMA_DEBUG__
    transfer_dump(xfer);
#endif

    if (enable_st_c2h_credit || enable_st_h2c_credit) {
        write_register(CYCLIC_RX_PAGES_MAX >> 1, &engine->sgdma_regs->credits,
               (unsigned long)(&engine->sgdma_regs->credits) -
                   (unsigned long)(engine->sgdma_regs));
    }

    //spin_unlock_irqrestore(&engine->lock, flags);

    /* start cyclic transfer */
    transfer_queue(engine, xfer);

    return 0;

    /* unwind on errors */
err_out:
    if (engine->cyclic_req) {
        sgdma_request_free(engine->cyclic_req);
        engine->cyclic_req = NULL;
    }
    
    if (engine->cyclic_sgt.orig_nents) {
        sgt_free_with_pages(&engine->cyclic_sgt, engine->dir,
                sgdev->pdev);
        engine->cyclic_sgt.orig_nents = 0;
        engine->cyclic_sgt.nents = 0;
        engine->cyclic_sgt.sgl = NULL;
    }

    //spin_unlock_irqrestore(&engine->lock, flags);

    return rc;
}

#if 0
static int engine_ring_process(struct anlogic_dma_engine *engine)
{
    struct sgdma_result *result;
    int start;
    int eop_count = 0;

    BUG_ON(!engine);
    result = engine->cyclic_result;
    BUG_ON(!result);

    /* where we start receiving in the ring buffer */
    start = engine->rx_tail;

    /* iterate through all newly received RX result descriptors */
    dbg_tfr("%s, result %d, 0x%x, len 0x%x.\n",
        engine->name, engine->rx_tail, result[engine->rx_tail].status,
        result[engine->rx_tail].length);

    /* update rx_tail index to tell where rx desc pos is single poll end. */
    while (result[engine->rx_tail].status && !engine->rx_overrun) {

        /* EOP bit set in result? */
        if (result[engine->rx_tail].status & RX_STATUS_EOP){
            eop_count++;
        }

        /* increment tail pointer */
        engine->rx_tail = (engine->rx_tail + 1) % CYCLIC_RX_PAGES_MAX;

        dbg_tfr("%s, head %d, tail %d, 0x%x, len 0x%x.\n",
            engine->name, engine->rx_head, engine->rx_tail,
            result[engine->rx_tail].status,
            result[engine->rx_tail].length);

        /* overrun? */
        if (engine->rx_tail == engine->rx_head) {
            dbg_tfr("%s: overrun\n", engine->name);
            /* flag to user space that overrun has occurred */
            engine->rx_overrun = 1;
        }
    }

    return eop_count;
}
#else
static int engine_ring_process(struct anlogic_dma_engine *engine)
{
    struct sgdma_result *result;
    struct sgdma_poll_wb *writeback_data;
    struct sgdma_transfer *xfer;
    int start, free_dsc, wb_cnt;
    int eop_count = 0;

    BUG_ON(!engine);

    /* do rx ring process */
    if (engine->dir == DMA_FROM_DEVICE) {
        result = engine->cyclic_result;
        BUG_ON(!result);

        /* where we start receiving in the ring buffer */
        start = engine->rx_tail;

        /* iterate through all newly received RX result descriptors */
        // dbg_tfr("%s, result %d, 0x%x, len 0x%x.\n",
        //     engine->name, engine->rx_tail, result[engine->rx_tail].status,
        //     result[engine->rx_tail].length);

        /* update rx_tail index to tell where rx desc pos is single poll end. */
        while (result[engine->rx_tail].status && !engine->rx_overrun) {

            /* EOP bit set in result? */
            if (result[engine->rx_tail].status & RX_STATUS_EOP) {
                eop_count++;
            }

            /* increment tail pointer */
            engine->rx_tail = (engine->rx_tail + 1) % CYCLIC_RX_PAGES_MAX;

            dbg_tfr("%s, head %d, tail %d, 0x%x, len 0x%x.\n",
                engine->name, engine->rx_head, engine->rx_tail,
                result[engine->rx_tail].status,
                result[engine->rx_tail].length);

            /* overrun? */
            if (engine->rx_tail == engine->rx_head) {
                dbg_tfr("%s: overrun\n", engine->name);
                /* flag to user space that overrun has occurred */
                engine->rx_overrun = 1;
            }
        }
    } else {
        /* do tx ring process */
        xfer = &engine->cyclic_req->tfer[0];

        writeback_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
        wb_cnt = writeback_data->completed_desc_count;

        if (wb_cnt & WB_ERR_MASK) {
            dbg_tfr("engine wb_cnt get err 0x%x\n", wb_cnt);
            return -1;
        }
        
        writeback_data->completed_desc_count = 0;

        /* calc tx desc cnt in current poll cycle. */
        free_dsc = wb_cnt & WB_COUNT_MASK;

        if (free_dsc <= engine->tx_desc_cnt) {
            dbg_tfr("engine current wb_cnt(%d) less than last(%d)\n", free_dsc, engine->tx_desc_cnt);
            return -1;
        }

        free_dsc -= engine->tx_desc_cnt;

        engine->tx_desc_cnt = wb_cnt & WB_COUNT_MASK;

        dbg_tfr("free desc %d, engine tx_desc_cnt %d\n", free_dsc, engine->tx_desc_cnt);

        for (start = 0; start < free_dsc; start++) {
            /* EOP bit set in result? */
            if (xfer->desc_virt[engine->tx_tail].control & SGDMA_DESC_EOP){
                eop_count++;
            }

            dbg_tfr("%s, head %d, tail %d, len 0x%x.\n",
                engine->name, engine->tx_head, engine->tx_tail,
                xfer->desc_virt[engine->tx_tail].bytes);
            
            /* increment tail pointer */
            engine->tx_tail = (engine->tx_tail + 1) % engine->tx_ring_desc_num;
        }

        /* overrun? */
        if (engine->tx_tail == engine->tx_head) {
            dbg_tfr("%s: overrun\n", engine->name);
            /* flag to user space that overrun has occurred */
            engine->rx_overrun = 1;
        }
    }

    return eop_count;
}
#endif

void engine_transfer_dequeue(struct anlogic_dma_engine *engine)
{
    struct sgdma_transfer *transfer;

    BUG_ON(!engine);

    /* pick first transfer on the queue (was submitted to the engine) */
    transfer = list_entry(engine->transfer_list.next, struct sgdma_transfer,
        entry);
    if (!transfer || transfer != &engine->cyclic_req->tfer[0]) {
        pr_info("%s, xfer 0x%p != 0x%p.\n",
            engine->name, transfer, &engine->cyclic_req->tfer[0]);
        return;
    }
    dbg_tfr("%s engine completed cyclic transfer 0x%p (%d desc).\n",
        engine->name, transfer, transfer->desc_num);
    /* remove completed transfer from list */
    list_del(engine->transfer_list.next);
}

static int engine_service_cyclic_polled(struct anlogic_dma_engine *engine)
{
#define CYCLIC_POLL_TIMEOUT_SECONDS 5
    int eop_count = 0;
    int rc = 0;
    struct sgdma_poll_wb *writeback_data;
    u32 sched_limit = 0;
    unsigned long timeout;

    BUG_ON(!engine);
    BUG_ON(engine->magic != MAGIC_ENGINE);

    writeback_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    engine_status_read(engine, 1, 0);

    /* XXX FIXME need add timeout break. */
    timeout = jiffies + (CYCLIC_POLL_TIMEOUT_SECONDS * HZ);

    while (eop_count == 0) {
        if (sched_limit != 0) {
            if ((sched_limit % NUM_POLLS_PER_SCHED) == 0)
                schedule();
        }
        sched_limit++;

        /* Monitor descriptor writeback address for errors */
        if ((writeback_data->completed_desc_count) & WB_ERR_MASK) {
            rc = -1;
            break;
        }

        /* prevent system from hanging in polled mode */
        if (time_after(jiffies, timeout)) {
            dbg_tfr("cyclic polling timeout occurred");
            break;
        }

        eop_count = engine_ring_process(engine);
    }

    if (eop_count == 0) {
        engine_status_read(engine, 1, 0);
        if ((engine->running) && !(engine->status & SGDMA_STAT_BUSY)) {
            /* transfers on queue? */
            if (!list_empty(&engine->transfer_list))
                engine_transfer_dequeue(engine);

            engine_service_shutdown(engine);
        }
    }

    return rc;
}

static int engine_service_cyclic_interrupt(struct anlogic_dma_engine *engine)
{
    int eop_count = 0;
    struct sgdma_transfer *xfer;

    BUG_ON(!engine);
    BUG_ON(engine->magic != MAGIC_ENGINE);

    engine_status_read(engine, 1, 0);

    eop_count = engine_ring_process(engine);
    /*
     * wake any reader on EOP, as one or more packets are now in
     * the RX buffer
     */
    xfer = &engine->cyclic_req->tfer[0];
    if(enable_st_c2h_credit){
        if (eop_count > 0) {
            //engine->eop_found = 1;
        }
        al_wake_up(&xfer->wq);
    } else {
        if (eop_count > 0) {
            /* awake task on transfer's wait queue */
            dbg_tfr("wake_up_interruptible() due to %d EOP's\n", eop_count);
            engine->eop_found = 1;
            al_wake_up(&xfer->wq);
        }
    }

    /* engine was running but is no longer busy? */
    if ((engine->running) && !(engine->status & SGDMA_STAT_BUSY)) {
        /* transfers on queue? */
        if (!list_empty(&engine->transfer_list))
            engine_transfer_dequeue(engine);

        engine_service_shutdown(engine);
    }

    return 0;
}

/* must be called with engine->lock already acquired */
int engine_service_cyclic(struct anlogic_dma_engine *engine)
{
    int rc = 0;

    dbg_tfr("engine_service_cyclic()");

    BUG_ON(!engine);
    BUG_ON(engine->magic != MAGIC_ENGINE);

    if (poll_mode)
        rc = engine_service_cyclic_polled(engine);
    else
        rc = engine_service_cyclic_interrupt(engine);

    return rc;
}

static struct scatterlist *sglist_index(struct sg_table *sgt, unsigned int idx)
{
    struct scatterlist *sg = sgt->sgl;
    int i;

    if (idx >= sgt->orig_nents)
        return NULL;

    if (!idx)
        return sg;

    for (i = 0; i < idx; i++, sg = sg_next(sg))
        ;

    return sg;
}

static int copy_cyclic_to_user(struct anlogic_dma_engine *engine, int pkt_length,
                int head, char __user *buf, size_t count)
{
    struct scatterlist *sg;
    int more = pkt_length;

    BUG_ON(!engine);
    BUG_ON(!buf);

    dbg_tfr("%s, pkt_len %d, head %d, user buf idx %u.\n",
        engine->name, pkt_length, head, engine->user_buffer_index);

    sg = sglist_index(&engine->cyclic_sgt, head);
    if (!sg) {
        pr_info("%s, head %d OOR, sgl %u.\n",
            engine->name, head, engine->cyclic_sgt.orig_nents);
        return -EIO;
    }

    /* EOP found? Transfer anything from head to EOP */
    while (more) {
        unsigned int copy = more > PAGE_SIZE ? PAGE_SIZE : more;
        unsigned int blen = count - engine->user_buffer_index;
        int rv;

        if (copy > blen)
            copy = blen;

        dbg_tfr("%s sg %d, 0x%p, copy %u to user %u.\n",
            engine->name, head, sg, copy,
            engine->user_buffer_index);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 16, 0)
        if (access_ok(VERIFY_WRITE, &buf[engine->user_buffer_index], copy)) {
#else
        if (access_ok(&buf[engine->user_buffer_index], copy)) {
#endif
            rv = copy_to_user(&buf[engine->user_buffer_index],
                page_address(sg_page(sg)), copy);
        } else {
            /* XXX FIXME do test use, copy to kernel buf. */
            memcpy(&buf[engine->user_buffer_index],
                page_address(sg_page(sg)), copy);

            rv = 0;
        }

        if (rv) {
            pr_info("%s copy_to_user %u failed %d\n",
                engine->name, copy, rv);
            return -EIO;
        }

        more -= copy;
        engine->user_buffer_index += copy;

        if (engine->user_buffer_index == count) {
            /* user buffer used up */
            break;
        }
        
        head++;
        if (head >= CYCLIC_RX_PAGES_MAX) {
            head = 0;
            sg = engine->cyclic_sgt.sgl;
        } else
            sg = sg_next(sg);
    }

    return pkt_length;
}

static int complete_cyclic(struct anlogic_dma_engine *engine, char __user *buf,
               size_t count)
{
    struct sgdma_result *result;
    int pkt_length = 0;
    int fault = 0;
    int eop = 0;
    int head;
    int rc = 0;
    int num_credit = 0;
    unsigned long flags;

    BUG_ON(!engine);
    result = engine->cyclic_result;
    BUG_ON(!result);

    spin_lock_irqsave(&engine->lock, flags);

    /* where the host currently is in the ring buffer */
    head = engine->rx_head;

    /* iterate over newly received results */
    while (engine->rx_head != engine->rx_tail || engine->rx_overrun) {

        WARN_ON(result[engine->rx_head].status==0);

        dbg_tfr("%s, result[%d].status = 0x%x length = 0x%x.\n",
            engine->name, engine->rx_head, 
            result[engine->rx_head].status,
            result[engine->rx_head].length);

        if ((result[engine->rx_head].status >> 16) != C2H_WB) {
            pr_info("%s, result[%d].status 0x%x, no magic.\n",
                engine->name, engine->rx_head,
                result[engine->rx_head].status);
            fault = 1;
        } else if (result[engine->rx_head].length > PAGE_SIZE) {
            pr_info("%s, result[%d].len 0x%x, > PAGE_SIZE 0x%lx.\n",
                engine->name, engine->rx_head,
                result[engine->rx_head].length, PAGE_SIZE);
            fault = 1;
        } else if (result[engine->rx_head].length == 0) {
            pr_info("%s, result[%d].length 0x%x.\n",
                engine->name, engine->rx_head,
                result[engine->rx_head].length);
            fault = 1;
            /* valid result */
        } else {
            pkt_length += result[engine->rx_head].length;
            num_credit++;
            /* seen eop? */
            if (result[engine->rx_head].status & RX_STATUS_EOP){
                eop = 1;
                engine->eop_found = 1;
            }

            dbg_tfr("%s, pkt_length=%d (%s)\n",
                engine->name, pkt_length,
                eop ? "with EOP" : "no EOP yet");
        }
        /* clear result */
        result[engine->rx_head].status = 0;
        result[engine->rx_head].length = 0;
        /* proceed head pointer so we make progress, even when fault */
        engine->rx_head = (engine->rx_head + 1) % CYCLIC_RX_PAGES_MAX;

#if AL_RING_RECIVE_WITH_EOP /* XXX FIXME break without EOP? */
        /* stop processing if a fault/eop was detected */
        if (fault || eop){
            break;
        }
#else
        /* stop processing if a fault was detected */
        if (fault) {
            break;
        }

        /* if rx bytes more than read bytes, exit read loop */
        if (pkt_length >= count) {
            break;
        }
#endif

    }

    spin_unlock_irqrestore(&engine->lock, flags);

    if (fault)
        return -EIO;
        
    rc = copy_cyclic_to_user(engine, pkt_length, head, buf, count);
    engine->rx_overrun = 0; 

    /* if copy is successful, release credits */
    if(rc > 0)
        write_register(num_credit, &engine->sgdma_regs->credits,
               (unsigned long)(&engine->sgdma_regs->credits) -
                   (unsigned long)(engine->sgdma_regs));

    return rc;
}

static int transfer_monitor_cyclic(struct anlogic_dma_engine *engine,
           struct sgdma_transfer *transfer, int timeout_ms)
{
   struct sgdma_result *result;
   int rc = 0;

   BUG_ON(!engine);
   BUG_ON(!transfer);

   result = engine->cyclic_result;
   BUG_ON(!result);

   if (poll_mode) {
       int i ;

       for (i = 0; i < 5; i++) {
           rc = engine_service_poll(engine, 1); /* XXX FIXME */
           if (rc) {
               pr_info("%s service_poll failed %d.\n",
                   engine->name, rc);
               rc = -ERESTARTSYS;
           }
           if (result[engine->rx_head].status)
               return 0;
       }
   } else {
       dbg_tfr("%s: rx_head=%d,rx_tail=%d, wait ...\n",
           engine->name, engine->rx_head, engine->rx_tail);

       if (enable_st_c2h_credit){
           rc = al_wait_event_interruptible_timeout( transfer->wq,
                   (engine->rx_head!=engine->rx_tail ||
                    engine->rx_overrun),
                   msecs_to_jiffies(timeout_ms));
           dbg_tfr("%s: wait returns %d, rx %d/%d, overrun %d.\n",
                engine->name, rc, engine->rx_head,
               engine->rx_tail, engine->rx_overrun);
       } else {
           rc = al_wait_event_interruptible_timeout( transfer->wq,
                   engine->eop_found,
                   msecs_to_jiffies(timeout_ms));
           dbg_tfr("%s: wait returns %d, eop_found %d.\n",
               engine->name, rc, engine->eop_found);
       }
   }

   return 0;
}

ssize_t sgdma_engine_read_cyclic(struct anlogic_dma_engine *engine, char __user *buf,
               size_t count, int timeout_ms)
{
    int i = 0;
    int rc = 0;
    int rc_len = 0;
    struct sgdma_transfer *transfer;

    BUG_ON(!engine);
    BUG_ON(engine->magic != MAGIC_ENGINE);

    transfer = &engine->cyclic_req->tfer[0];
    BUG_ON(!transfer);

    engine->user_buffer_index = 0;

    do {
        rc = transfer_monitor_cyclic(engine, transfer, timeout_ms);
        if (rc < 0)
            return rc;
        rc = complete_cyclic(engine, buf, count);
        if (rc < 0)
            return rc;
        rc_len += rc;
#if AL_RING_RECIVE_WITH_EOP /* XXX FIXME break without EOP? */
        i++;
        if (i > 3)
            break;
    } while (!engine->eop_found);
#else
    } while (rc_len < count);
#endif

    if(enable_st_c2h_credit)
        engine->eop_found = 0;

    return rc_len;
}

ssize_t sgdma_engine_write_cyclic(struct anlogic_dma_engine *engine, char __user *buf,
               size_t count, int timeout_ms)
{
    return 0;
}

