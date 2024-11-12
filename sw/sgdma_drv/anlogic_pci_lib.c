/*
 * This file is part of the anlogic PCIe driver for Linux
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>

#include "cdev_sgdma.h"

#include "anlogic_pci_lib.h"
#include "anlogic_pci_dbg.h"
#include "anlogic_thread.h"
#include "anlogic_ring.h"

/* Module Parameters */
unsigned int poll_mode = 1;
module_param(poll_mode, uint, 0644);
MODULE_PARM_DESC(poll_mode, "Set 1 for hw polling, default is 0 (interrupts)");

unsigned int enable_st_c2h_credit = 0;
module_param(enable_st_c2h_credit, uint, 0644);
MODULE_PARM_DESC(enable_st_c2h_credit,
    "Set 1 to enable ST C2H engine credit feature, default is 0 ( credit control disabled)");

unsigned int enable_st_h2c_credit = 0;
module_param(enable_st_h2c_credit, uint, 0644);
MODULE_PARM_DESC(enable_st_h2c_credit,
    "Set 1 to enable ST H2C engine credit feature, default is 0 ( credit control disabled)");

unsigned int desc_blen_max = SGDMA_DESC_BLEN_MAX;
module_param(desc_blen_max, uint, 0644);
MODULE_PARM_DESC(desc_blen_max, "per descriptor max. buffer length, default is (1 << 28) - 1");

/*
 * RTO - code to detect if MSI/MSI-X capability exists is derived
 * from linux/pci/msi.c - pci_msi_check_device
 */
static unsigned int interrupt_mode = 1;
module_param(interrupt_mode, uint, 0644);
MODULE_PARM_DESC(interrupt_mode, "0 - MSI-x , 1 - MSI, 2 - Legacy");

#define SGDMA_PERF_NUM_DESC 128

/*
 * sgdma device management
 * maintains a list of the sgdma devices
 */
static LIST_HEAD(sgdev_list);
static DEFINE_MUTEX(sgdev_mutex);
static LIST_HEAD(sgdev_rcu_list);
static DEFINE_SPINLOCK(sgdev_rcu_lock);

static irqreturn_t anlogic_channel_irq(int irq, void *dev_id);
static irqreturn_t anlogic_user_irq(int irq, void *dev_id);
static irqreturn_t anlogic_isr(int irq, void *dev_id);

#ifndef list_last_entry
#define list_last_entry(ptr, type, member) list_entry((ptr)->prev, type, member)
#endif

static inline int al_dev_list_add(struct anlogic_dev *sgdev)
{
    mutex_lock(&sgdev_mutex);
    if (list_empty(&sgdev_list)) {
        sgdev->idx = 0;
        if (poll_mode) {
            int rv = sgdma_threads_create(sgdev->h2c_channel_max +
                    sgdev->c2h_channel_max);
            if (rv < 0) {
                mutex_unlock(&sgdev_mutex);
                return rv;
            }
        }
    } else {
        struct anlogic_dev *last;

        last = list_last_entry(&sgdev_list, struct anlogic_dev, list_head);
        sgdev->idx = last->idx + 1;
    }
    list_add_tail(&sgdev->list_head, &sgdev_list);
    mutex_unlock(&sgdev_mutex);

    dbg_init("dev %s, sgdev 0x%p, sgdma idx %d.\n",
         dev_name(&sgdev->pdev->dev), sgdev, sgdev->idx);

    spin_lock(&sgdev_rcu_lock);
    list_add_tail_rcu(&sgdev->rcu_node, &sgdev_rcu_list);
    spin_unlock(&sgdev_rcu_lock);

    return 0;
}

#undef list_last_entry

static inline void al_dev_list_remove(struct anlogic_dev *sgdev)
{
    mutex_lock(&sgdev_mutex);
    list_del(&sgdev->list_head);
    if (poll_mode && list_empty(&sgdev_list))
        sgdma_threads_destroy();
    mutex_unlock(&sgdev_mutex);

    spin_lock(&sgdev_rcu_lock);
    list_del_rcu(&sgdev->rcu_node);
    spin_unlock(&sgdev_rcu_lock);
    synchronize_rcu();
}

/*
 * MSI-X interrupt:
 *  <h2c+c2h channel_max> vectors, followed by <user_max> vectors
 */

#ifndef arch_msi_check_device
static int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
    return 0;
}
#endif

static inline u32 build_u32(u32 hi, u32 lo)
{
    return ((hi & 0xFFFFUL) << 16) | (lo & 0xFFFFUL);
}

static inline u64 build_u64(u64 hi, u64 lo)
{
    return ((hi & 0xFFFFFFFULL) << 32) | (lo & 0xFFFFFFFFULL);
}

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
static int msi_msix_capable(struct pci_dev *dev, int type)
{
    struct pci_bus *bus;
    int ret;

    if (!dev || dev->no_msi)
        return 0;

    for (bus = dev->bus; bus; bus = bus->parent)
        if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
            return 0;

    ret = arch_msi_check_device(dev, 1, type);
    if (ret)
        return 0;

    if (!pci_find_capability(dev, type))
        return 0;

    return 1;
}

static void disable_msi_msix(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    if (sgdev->msix_enabled) {
        pci_disable_msix(pdev);
        sgdev->msix_enabled = 0;
    } else if (sgdev->msi_enabled) {
        pci_disable_msi(pdev);
        sgdev->msi_enabled = 0;
    }
}

static int enable_msi_msix(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    int rv = 0;

    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    if (!interrupt_mode && msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {

        int req_nvec = sgdev->c2h_channel_max + sgdev->h2c_channel_max +
                       sgdev->user_max;
                   
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        dbg_init("Enabling MSI-X\n");
        rv = pci_alloc_irq_vectors(pdev, req_nvec, req_nvec,
                       PCI_IRQ_MSIX);
        dbg_init("PCI alloc irq: %d vector\n ",rv);
#else
        int i;

        dbg_init("Enabling MSI-X\n");
        for (i = 0; i < req_nvec; i++)
            sgdev->entry[i].entry = i;   //linux max support 32 msix vector

        rv = pci_enable_msix(pdev, sgdev->entry, req_nvec);
#endif
        if (rv < 0)
            dbg_init("Couldn't enable MSI-X mode: %d\n", rv);

        sgdev->msix_enabled = 1;

    } else if (interrupt_mode == 1 &&
           msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
        /* enable message signalled interrupts */
        dbg_init("pci_enable_msi()\n");
        rv = pci_enable_msi(pdev);
        if (rv < 0)
            dbg_init("Couldn't enable MSI mode: %d\n", rv);
        sgdev->msi_enabled = 1;

    } else {
        dbg_init("MSI/MSI-X not detected - using legacy interrupts\n");
    }

    return rv;
}

static void pci_check_intr_pend(struct pci_dev *pdev)
{
    u16 v;

    pci_read_config_word(pdev, PCI_STATUS, &v);
    if (v & PCI_STATUS_INTERRUPT) {
        pr_info("%s PCI STATUS Interrupt pending 0x%x.\n",
            dev_name(&pdev->dev), v);
        pci_write_config_word(pdev, PCI_STATUS, PCI_STATUS_INTERRUPT);
    }
}

static void pci_keep_intx_enabled(struct pci_dev *pdev)
{
    /* workaround to a h/w bug:
     * when msix/msi become unavaile, default to legacy.
     * However the legacy enable was not checked.
     * If the legacy was disabled, no ack then everything stuck
     */
    u16 pcmd, pcmd_new;

    pci_read_config_word(pdev, PCI_COMMAND, &pcmd);
    pcmd_new = pcmd & ~PCI_COMMAND_INTX_DISABLE;
    if (pcmd_new != pcmd) {
        pr_info("%s: clear INTX_DISABLE, 0x%x -> 0x%x.\n",
            dev_name(&pdev->dev), pcmd, pcmd_new);
        pci_write_config_word(pdev, PCI_COMMAND, pcmd_new);
    }
}

/* channel_interrupts_enable -- Enable interrupts we are interested in */
void channel_interrupts_enable(struct anlogic_dev *sgdev, u32 mask)
{
    struct interrupt_regs *reg =
        (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                      SGDMA_OFS_INT_CTRL);
#if 1 /* XXX FIXME */
    write_register(mask, &reg->channel_int_enable_w1s, SGDMA_OFS_INT_CTRL);
#else
    write_register(mask, &reg->channel_int_enable, SGDMA_OFS_INT_CTRL);
#endif
}

/* channel_interrupts_disable -- Disable interrupts we not interested in */
void channel_interrupts_disable(struct anlogic_dev *sgdev, u32 mask)
{
    struct interrupt_regs *reg =
        (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                      SGDMA_OFS_INT_CTRL);
#if 1 /* XXX FIXME */
        write_register(mask, &reg->channel_int_enable_w1c, SGDMA_OFS_INT_CTRL);
#else
        write_register(0, &reg->channel_int_enable, SGDMA_OFS_INT_CTRL);
#endif
}

/* read_interrupts -- Print the interrupt controller status */
static u32 read_interrupts(struct anlogic_dev *sgdev)
{
    struct interrupt_regs *reg =
        (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                      SGDMA_OFS_INT_CTRL);
    u32 lo;
    u32 hi;

    /* extra debugging; inspect complete engine set of registers */
    hi = read_register(&reg->user_int_request);
    dbg_io("ioread32(0x%p) returned 0x%08x (user_int_request).\n",
           &reg->user_int_request, hi);
    lo = read_register(&reg->channel_int_request);
    dbg_io("ioread32(0x%p) returned 0x%08x (channel_int_request)\n",
           &reg->channel_int_request, lo);

    /* return interrupts: user in upper 16-bits, channel in lower 16-bits */
    return build_u32(hi, lo);
}

static void prog_irq_msix_user(struct anlogic_dev *sgdev, bool clear)
{
    /* user */
    struct interrupt_regs *int_regs =
        (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                      SGDMA_OFS_INT_CTRL);
    u32 i = sgdev->c2h_channel_max + sgdev->h2c_channel_max;
    u32 max = i + sgdev->user_max;
    int j;

    for (j = 0; i < max; j++) {
        u32 val = 0;
        int k;
        int shift = 0;

        if (clear)
            i += 4;
        else
            for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
                val |= (i & 0x1f) << shift;

        write_register(
            val, &int_regs->user_msi_vector[j],
            SGDMA_OFS_INT_CTRL +
                ((unsigned long)&int_regs->user_msi_vector[j] -
                 (unsigned long)int_regs));

        dbg_init("vector %d, 0x%x.\n", j, val);
    }
}

static void prog_irq_msix_channel(struct anlogic_dev *sgdev, bool clear)
{
    struct interrupt_regs *int_regs =
        (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                      SGDMA_OFS_INT_CTRL);
    u32 max = sgdev->c2h_channel_max + sgdev->h2c_channel_max;
    u32 i;
    int j;

    /* engine */
    for (i = 0, j = 0; i < max; j++) {
        u32 val = 0;
        int k;
        int shift = 0;

        if (clear)
            i += 4;
        else
            for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
                val |= (i & 0x1f) << shift;

        write_register(val, &int_regs->channel_msi_vector[j],
                   SGDMA_OFS_INT_CTRL +
                       ((unsigned long)&int_regs
                        ->channel_msi_vector[j] -
                    (unsigned long)int_regs));
        dbg_init("vector %d, 0x%x.\n", j, val);
    }
}

static void irq_msix_channel_teardown(struct anlogic_dev *sgdev)
{
    struct anlogic_dma_engine *engine;
    int j = 0;
    int i = 0;
  pr_info("sgdev->msix_enabled: %d\n", sgdev->msix_enabled);
    if (!sgdev->msix_enabled)
        return;

    //prog_irq_msix_channel(sgdev, 1);
     pr_info("sgdev->h2c_channel_max: %d\n", sgdev->h2c_channel_max);
     pr_info("sgdev->c2h_channel_max: %d\n", sgdev->c2h_channel_max);

    engine = sgdev->engine_h2c;
    for (i = 0; i < sgdev->h2c_channel_max; i++, j++, engine++) {
         pr_info("engine->msix_irq_line: %d\n", engine->msix_irq_line);
        if (!engine->msix_irq_line)
            break;
        dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
               engine);
        free_irq(engine->msix_irq_line, engine);
    }

    engine = sgdev->engine_c2h;
    for (i = 0; i < sgdev->c2h_channel_max; i++, j++, engine++) {
        pr_info("engine->msix_irq_line: %d\n", engine->msix_irq_line);
        if (!engine->msix_irq_line)
            break;
        dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
               engine);
        free_irq(engine->msix_irq_line, engine);
    }
}

static int irq_msix_channel_setup(struct anlogic_dev *sgdev)
{
    int i;
    int j;
    int rv = 0;
    u32 vector;
  
    struct anlogic_dma_engine *engine;

    if (!sgdev) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (!sgdev->msix_enabled)
        return 0;

    j = sgdev->h2c_channel_max;
    engine = sgdev->engine_h2c;
     pr_info(" sgdev->h2c_channel_max %d .h2c : %d,\n",
                 sgdev->h2c_channel_max, sgdev->c2h_channel_max);
    for (i = 0; i < sgdev->h2c_channel_max; i++, engine++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        vector = pci_irq_vector(sgdev->pdev, i);
#else
        vector = sgdev->entry[i].vector;
#endif
        pr_info(" vector%d  is: %d\n", i, vector);
        rv = request_irq(vector, anlogic_channel_irq, 0, sgdev->mod_name,
                 engine);
        
        if (rv) {
            pr_info("requesti irq#%d failed %d, engine %s.\n",
                vector, rv, engine->name);
            return rv;
        }
        
        if (rv) {
            pr_info("requesti irq#%d failed %d,\n",
                vector, rv);
            return rv;
        }
        pr_info(" irq#%d.\n",vector);
        
        engine->msix_irq_line = vector;
         pr_info("engine->msix_irq_line:%d.\n",engine->msix_irq_line);
    }
    
    engine = sgdev->engine_c2h;
    for (i = 0; i < sgdev->c2h_channel_max; i++, j++, engine++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        vector = pci_irq_vector(sgdev->pdev, j);
#else
        vector = sgdev->entry[j].vector;
#endif
        rv = request_irq(vector, anlogic_channel_irq, 0, sgdev->mod_name,
                 engine);
        if (rv) {
            pr_info("requesti irq#%d failed %d, engine %s.\n",
                vector, rv, engine->name);
            return rv;
        }
        pr_info("engine %s, irq#%d.\n", engine->name, vector);
        engine->msix_irq_line = vector;
        pr_info("engine->msix_irq_line:%d.\n",engine->msix_irq_line);
        
    }
         
    return 0;
}

static void irq_msix_user_teardown(struct anlogic_dev *sgdev)
{
    int i;
    int j;

    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return;
    }

    if (!sgdev->msix_enabled)
        return;

    j = sgdev->h2c_channel_max + sgdev->c2h_channel_max;

    prog_irq_msix_user(sgdev, 1);

    for (i = 0; i < sgdev->user_max; i++, j++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        u32 vector = pci_irq_vector(sgdev->pdev, j);
#else
        u32 vector = sgdev->entry[j].vector;
#endif
        dbg_init("user %d, releasing IRQ#%d\n", i, vector);
        free_irq(vector, &sgdev->user_irq[i]);
    }
   
}

static int irq_msi_setup(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    int rv;

    sgdev->irq_line = (int)pdev->irq;
    rv = request_irq(pdev->irq, anlogic_isr, 0, sgdev->mod_name, sgdev);//IRQF_TRIGGER_RISING
    if (rv)
        dbg_init("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
    else
        dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, sgdev);

    return rv;
}

static int irq_msix_user_setup(struct anlogic_dev *sgdev)
{
    int i;
    int j = sgdev->h2c_channel_max + sgdev->c2h_channel_max;
    int rv = 0;

    /* vectors set in probe_scan_for_msi() */
    for (i = 0; i < sgdev->user_max; i++, j++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        u32 vector = pci_irq_vector(sgdev->pdev, j);
#else
        u32 vector = sgdev->entry[j].vector;
#endif
        rv = request_irq(vector, anlogic_user_irq, 0, sgdev->mod_name,
                 &sgdev->user_irq[i]);













        if (rv) {
            pr_info("user %d couldn't use IRQ#%d, %d\n", i, vector,
                rv);
            break;
        }
        pr_info("%d-USR-%d, IRQ#%d with 0x%p\n", sgdev->idx, i, vector,
            &sgdev->user_irq[i]);
    }

    /* If any errors occur, free IRQs that were successfully requested */
    if (rv) {
        for (i--, j--; i >= 0; i--, j--) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
            u32 vector = pci_irq_vector(sgdev->pdev, j);
#else
            u32 vector = sgdev->entry[j].vector;
#endif
            free_irq(vector, &sgdev->user_irq[i]);
        }
    }

    return rv;
}

static int irq_legacy_setup(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    return -ENOSYS; /* XXX FIXME: not support legacy irq */
}

static void irq_teardown(struct anlogic_dev *sgdev)
{
    if (sgdev->msix_enabled) {
        irq_msix_channel_teardown(sgdev);
        irq_msix_user_teardown(sgdev);
    } else if (sgdev->irq_line != -1) {
        dbg_init("Releasing IRQ#%d\n", sgdev->irq_line);
        free_irq(sgdev->irq_line, sgdev);
    }
}

static int irq_setup(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    pci_keep_intx_enabled(pdev);

    if (sgdev->msix_enabled) {
        int rv = irq_msix_channel_setup(sgdev);
        if (rv)
            return rv;

        rv = irq_msix_user_setup(sgdev);
        if (rv)
            return rv;

        prog_irq_msix_channel(sgdev, 0);
        prog_irq_msix_user(sgdev, 0);

        return 0;
    } else if (sgdev->msi_enabled)
        return irq_msi_setup(sgdev, pdev);

    return irq_legacy_setup(sgdev, pdev);
}

static irqreturn_t user_irq_service(int irq, struct anlogic_user_irq *user_irq)
{
    unsigned long flags;

    if (!user_irq) {
        pr_err("Invalid user_irq\n");
        return IRQ_NONE;
    }

    if (user_irq->handler)
        return user_irq->handler(user_irq->user_idx, user_irq->dev);

    spin_lock_irqsave(&(user_irq->events_lock), flags);
    if (!user_irq->events_irq) {
        user_irq->events_irq = 1;
        wake_up_interruptible(&(user_irq->events_wq));
    }
    spin_unlock_irqrestore(&(user_irq->events_lock), flags);

    return IRQ_HANDLED;
}

/*
 * anlogic_user_irq() - Interrupt handler for user interrupts in MSI-X mode
 *
 * @dev_id pointer to sgdma_dev
 */
static irqreturn_t anlogic_user_irq(int irq, void *dev_id)
{
    struct anlogic_user_irq *user_irq;

    dbg_irq("(irq=%d) <<<< INTERRUPT SERVICE ROUTINE\n", irq);

    if (!dev_id) {
        pr_err("Invalid dev_id on irq line %d\n", irq);
        return IRQ_NONE;
    }
    user_irq = (struct anlogic_user_irq *)dev_id;

    return user_irq_service(irq, user_irq);
}

/*
 * anlogic_channel_irq() - Interrupt handler for channel interrupts in MSI-X mode
 *
 * @dev_id pointer to sgdma_dev
 */
static irqreturn_t anlogic_channel_irq(int irq, void *dev_id)
{
    struct anlogic_dev *sgdev;
    sgdev = (struct anlogic_dev*)dev_id;

    dbg_irq("(irq=%d) <<<< INTERRUPT service ROUTINE\n", irq);

    /*
     * RTO - need to protect access here if multiple MSI-X are used for
     * user interrupts
     */
    sgdev->irq_count++;
    return IRQ_HANDLED;
}

void enable_perf(struct anlogic_dma_engine *engine)
{
    u32 w;

    w = SGDMA_PERF_CLEAR;
    write_register(w, &engine->regs->perf_ctrl,
               (unsigned long)(&engine->regs->perf_ctrl) -
                   (unsigned long)(&engine->regs));
    read_register(&engine->regs->identifier);
    w = SGDMA_PERF_AUTO | SGDMA_PERF_RUN;
    write_register(w, &engine->regs->perf_ctrl,
               (unsigned long)(&engine->regs->perf_ctrl) -
                   (unsigned long)(&engine->regs));
    read_register(&engine->regs->identifier);

    dbg_perf("IOCTL_SGDMA_PERF_START\n");
}
EXPORT_SYMBOL_GPL(enable_perf);

void get_perf_stats(struct anlogic_dma_engine *engine)
{
    u32 hi;
    u32 lo;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return;
    }

    if (!engine->sgdma_perf) {
        pr_info("%s perf struct not set up.\n", engine->name);
        return;
    }

    hi = 0;
    lo = read_register(&engine->regs->completed_desc_count);
    engine->sgdma_perf->iterations = build_u64(hi, lo);

    hi = read_register(&engine->regs->perf_cyc_hi);
    lo = read_register(&engine->regs->perf_cyc_lo);

    engine->sgdma_perf->clock_cycle_count = build_u64(hi, lo);

    hi = read_register(&engine->regs->perf_dat_hi);
    lo = read_register(&engine->regs->perf_dat_lo);
    engine->sgdma_perf->data_cycle_count = build_u64(hi, lo);

    hi = read_register(&engine->regs->perf_pnd_hi);
    lo = read_register(&engine->regs->perf_pnd_lo);
    engine->sgdma_perf->pending_count = build_u64(hi, lo);
}
EXPORT_SYMBOL_GPL(get_perf_stats);

int engine_reg_dump(struct anlogic_dma_engine *engine)
{
    u32 w;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    w = read_register(&engine->regs->identifier);
    pr_info("%s: ioread32(0x%p) = 0x%08x (id).\n", engine->name,
        &engine->regs->identifier, w);
    w &= BLOCK_ID_MASK;
    if (w != BLOCK_ID_HEAD) {
        pr_err("%s: engine id missing, 0x%08x exp. & 0x%x = 0x%x\n",
               engine->name, w, BLOCK_ID_MASK, BLOCK_ID_HEAD);
        return -EINVAL;
    }
    /* extra debugging; inspect complete engine set of registers */
    w = read_register(&engine->regs->status);
    pr_info("%s: ioread32(0x%p) = 0x%08x (status).\n", engine->name,
        &engine->regs->status, w);
    w = read_register(&engine->regs->control);
    pr_info("%s: ioread32(0x%p) = 0x%08x (control)\n", engine->name,
        &engine->regs->control, w);
    w = read_register(&engine->sgdma_regs->first_desc_lo);
    pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_lo)\n", engine->name,
        &engine->sgdma_regs->first_desc_lo, w);
    w = read_register(&engine->sgdma_regs->first_desc_hi);
    pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_hi)\n", engine->name,
        &engine->sgdma_regs->first_desc_hi, w);
    w = read_register(&engine->sgdma_regs->first_desc_adjacent);
    pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_adjacent).\n",
        engine->name, &engine->sgdma_regs->first_desc_adjacent, w);
    w = read_register(&engine->regs->completed_desc_count);
    pr_info("%s: ioread32(0x%p) = 0x%08x (completed_desc_count).\n",
        engine->name, &engine->regs->completed_desc_count, w);
    w = read_register(&engine->regs->interrupt_enable_mask);
    pr_info("%s: ioread32(0x%p) = 0x%08x (interrupt_enable_mask)\n",
        engine->name, &engine->regs->interrupt_enable_mask, w);

    return 0;
}

void __engine_status_dump(struct anlogic_dma_engine *engine, int (*printf)(const char *fmt, ...))
{
    u32 v = engine->status;
    char buffer[256];
    char *buf = buffer;
    int len = 0;

    len = sprintf(buf, "SG engine %s status: 0x%08x: ", engine->name, v);

    if ((v & SGDMA_STAT_BUSY))
        len += sprintf(buf + len, "BUSY,");
    if ((v & SGDMA_STAT_DESC_STOPPED))
        len += sprintf(buf + len, "DESC_STOPPED,");
    if ((v & SGDMA_STAT_DESC_COMPLETED))
        len += sprintf(buf + len, "DESC_COMPL,");

    /* common H2C & C2H */
    if ((v & SGDMA_STAT_COMMON_ERR_MASK)) {
        if ((v & SGDMA_STAT_ALIGN_MISMATCH))
            len += sprintf(buf + len, "ALIGN_MISMATCH ");
        if ((v & SGDMA_STAT_MAGIC_STOPPED))
            len += sprintf(buf + len, "MAGIC_STOPPED ");
        if ((v & SGDMA_STAT_INVALID_LEN))
            len += sprintf(buf + len, "INVLIAD_LEN ");
        if ((v & SGDMA_STAT_IDLE_STOPPED))
            len += sprintf(buf + len, "IDLE_STOPPED ");
        buf[len - 1] = ',';
    }

    if (engine->dir == DMA_TO_DEVICE) {
        /* H2C only */
        if ((v & SGDMA_STAT_H2C_R_ERR_MASK)) {
            len += sprintf(buf + len, "R:");
            if ((v & SGDMA_STAT_H2C_R_UNSUPP_REQ))
                len += sprintf(buf + len, "UNSUPP_REQ ");
            if ((v & SGDMA_STAT_H2C_R_COMPL_ABORT))
                len += sprintf(buf + len, "COMPL_ABORT ");
            if ((v & SGDMA_STAT_H2C_R_PARITY_ERR))
                len += sprintf(buf + len, "PARITY ");
            if ((v & SGDMA_STAT_H2C_R_HEADER_EP))
                len += sprintf(buf + len, "HEADER_EP ");
            if ((v & SGDMA_STAT_H2C_R_UNEXP_COMPL))
                len += sprintf(buf + len, "UNEXP_COMPL ");
            buf[len - 1] = ',';
        }

        if ((v & SGDMA_STAT_H2C_W_ERR_MASK)) {
            len += sprintf(buf + len, "W:");
            if ((v & SGDMA_STAT_H2C_W_DECODE_ERR))
                len += sprintf(buf + len, "DECODE_ERR ");
            if ((v & SGDMA_STAT_H2C_W_SLAVE_ERR))
                len += sprintf(buf + len, "SLAVE_ERR ");
            buf[len - 1] = ',';
        }

    } else {
        /* C2H only */
        if ((v & SGDMA_STAT_C2H_R_ERR_MASK)) {
            len += sprintf(buf + len, "R:");
            if ((v & SGDMA_STAT_C2H_R_DECODE_ERR))
                len += sprintf(buf + len, "DECODE_ERR ");
            if ((v & SGDMA_STAT_C2H_R_SLAVE_ERR))
                len += sprintf(buf + len, "SLAVE_ERR ");
            buf[len - 1] = ',';
        }
    }

    /* common H2C & C2H */
    if ((v & SGDMA_STAT_DESC_ERR_MASK)) {
        len += sprintf(buf + len, "DESC_ERR:");
        if ((v & SGDMA_STAT_DESC_UNSUPP_REQ))
            len += sprintf(buf + len, "UNSUPP_REQ ");
        if ((v & SGDMA_STAT_DESC_COMPL_ABORT))
            len += sprintf(buf + len, "COMPL_ABORT ");
        if ((v & SGDMA_STAT_DESC_PARITY_ERR))
            len += sprintf(buf + len, "PARITY ");
        if ((v & SGDMA_STAT_DESC_HEADER_EP))
            len += sprintf(buf + len, "HEADER_EP ");
        if ((v & SGDMA_STAT_DESC_UNEXP_COMPL))
            len += sprintf(buf + len, "UNEXP_COMPL ");
        buf[len - 1] = ',';
    }

    buf[len - 1] = '\0';
    printf("%s\n", buffer);
}

static void engine_status_dump(struct anlogic_dma_engine *engine)
{
#ifndef printk
    __engine_status_dump(engine, printk);
#else
    __engine_status_dump(engine, _printk);
#endif
}

/**
 * engine_status_read() - read status of SG DMA engine (optionally reset)
 *
 * Stores status in engine->status.
 *
 * @return error value on failure, 0 otherwise
 */
int engine_status_read(struct anlogic_dma_engine *engine, bool clear, bool dump)
{
    int rv = 0;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (dump) {
        rv = engine_reg_dump(engine);
        if (rv < 0) {
            pr_err("Failed to dump register\n");
            return rv;
        }
    }

    /* read status register */
    if (clear)
        //engine->status = read_register(&engine->regs->status_rc);
        engine->status = read_register(&engine->regs->status); /* XXX FIXME */
    else
        engine->status = read_register(&engine->regs->status);

    if (dump)
        engine_status_dump(engine);

    return rv;
}

int engine_start_mode_config(struct anlogic_dma_engine *engine)
{
    u32 w;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    /* If a perf test is running, enable the engine interrupts */
    if (engine->sgdma_perf) {
        w = SGDMA_CTRL_IE_DESC_STOPPED;
        w |= SGDMA_CTRL_IE_DESC_COMPLETED;
        w |= SGDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
        w |= SGDMA_CTRL_IE_MAGIC_STOPPED;
        w |= SGDMA_CTRL_IE_IDLE_STOPPED;
        w |= SGDMA_CTRL_IE_READ_ERROR;
        w |= SGDMA_CTRL_IE_DESC_ERROR;

        write_register(
            w, &engine->regs->interrupt_enable_mask,
            (unsigned long)(&engine->regs->interrupt_enable_mask) -
                (unsigned long)(engine->regs));
    }

    /* write control register of SG DMA engine */
    w = (u32)SGDMA_CTRL_RUN_STOP;
    w |= (u32)SGDMA_CTRL_IE_READ_ERROR;
    w |= (u32)SGDMA_CTRL_IE_DESC_ERROR;
    w |= (u32)SGDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
    w |= (u32)SGDMA_CTRL_IE_MAGIC_STOPPED;

    if (poll_mode) {
        w |= (u32)SGDMA_CTRL_POLL_MODE_WB;
        w |= (u32)SGDMA_CTRL_IE_DESC_COMPLETED; /* FIXME hwb */
    } else {
        w |= (u32)SGDMA_CTRL_IE_DESC_STOPPED;
        w |= (u32)SGDMA_CTRL_IE_DESC_COMPLETED;
    }

    /* set non-incremental addressing mode */
    if (engine->non_incr_addr)
        w |= (u32)SGDMA_CTRL_NON_INCR_ADDR;

    dbg_tfr("iowrite32(0x%08x to 0x%p) (control)\n", w,
        (void *)&engine->regs->control);

    /* start the engine */
    write_register(w, &engine->regs->control,
               (unsigned long)(&engine->regs->control) -
                   (unsigned long)(engine->regs));

    /* dummy read of status register to flush all previous writes */
    w = read_register(&engine->regs->status);
    dbg_tfr("ioread32(0x%p) = 0x%08x (dummy read flushes writes).\n",
        &engine->regs->status, w);

    return 0;
}

/* transfer_desc_init() - Chains the descriptors as a singly-linked list
 *
 * Each descriptor's next * pointer specifies the bus address
 * of the next descriptor.
 * Terminates the last descriptor to form a singly-linked list
 *
 * @transfer Pointer to SG DMA transfers
 * @count Number of descriptors allocated in continuous PCI bus addressable
 * memory
 *
 * @return 0 on success, EINVAL on failure
 */
static int transfer_desc_init(struct sgdma_transfer *transfer, int count)
{
    struct sgdma_desc *desc_virt = transfer->desc_virt;
    dma_addr_t desc_bus = transfer->desc_bus;
    int i;

    /* create singly-linked list for SG DMA controller */
    for (i = 0; i < count - 1; i++) {
        /* increment bus address to next in array */
        desc_bus += sizeof(struct sgdma_desc);

        /* singly-linked list uses bus addresses */
        desc_virt[i].next_lo = cpu_to_le32(PCI_DMA_L(desc_bus));
        desc_virt[i].next_hi = cpu_to_le32(PCI_DMA_H(desc_bus));
        desc_virt[i].bytes = cpu_to_le32(0);

        desc_virt[i].control = cpu_to_le32(DESC_MAGIC);
    }
    /* { i = number - 1 } */
    /* zero the last descriptor next pointer */
    desc_virt[i].next_lo = cpu_to_le32(0);
    desc_virt[i].next_hi = cpu_to_le32(0);
    desc_virt[i].bytes = cpu_to_le32(0);
    desc_virt[i].control = cpu_to_le32(DESC_MAGIC);

    return 0;
}

/* sgdma_desc_done - recycle cache-coherent linked list of descriptors.
 *
 * @dev Pointer to pci_dev
 * @number Number of descriptors to be allocated
 * @desc_virt Pointer to (i.e. virtual address of) first descriptor in list
 * @desc_bus Bus address of first descriptor in list
 */
static inline void sgdma_desc_done(struct sgdma_desc *desc_virt)
{
    memset(desc_virt, 0, SGDMA_TRANSFER_MAX_DESC * sizeof(struct sgdma_desc));
}

/* sgdma_desc() - Fill a descriptor with the transfer details
 *
 * @desc pointer to descriptor to be filled
 * @addr root complex address
 * @ep_addr end point address
 * @len number of bytes, must be a (non-negative) multiple of 4.
 * @dir, dma direction
 * is the end point address. If zero, vice versa.
 *
 * Does not modify the next pointer
 */
void sgdma_desc_set(struct sgdma_desc *desc, dma_addr_t rc_bus_addr,
              u64 ep_addr, int len, int dir)
{
    /* transfer length */
    desc->bytes = cpu_to_le32(len);
    if (dir == DMA_TO_DEVICE) {
        /* read from root complex memory (source address) */
        desc->src_addr_lo = cpu_to_le32(PCI_DMA_L(rc_bus_addr));
        desc->src_addr_hi = cpu_to_le32(PCI_DMA_H(rc_bus_addr));
        /* write to end point address (destination address) */
        desc->dst_addr_lo = cpu_to_le32(PCI_DMA_L(ep_addr));
        desc->dst_addr_hi = cpu_to_le32(PCI_DMA_H(ep_addr));
    } else {
        /* read from end point address (source address) */
        desc->src_addr_lo = cpu_to_le32(PCI_DMA_L(ep_addr));
        desc->src_addr_hi = cpu_to_le32(PCI_DMA_H(ep_addr));
        /* write to root complex memory (destination address) */
        desc->dst_addr_lo = cpu_to_le32(PCI_DMA_L(rc_bus_addr));
        desc->dst_addr_hi = cpu_to_le32(PCI_DMA_H(rc_bus_addr));
    }
}

/* sgdma_desc_link() - Link two descriptors
 *
 * Link the first descriptor to a second descriptor, or terminate the first.
 *
 * @first first descriptor
 * @second second descriptor, or NULL if first descriptor must be set as last.
 * @second_bus bus address of second descriptor
 */
void sgdma_desc_link(struct sgdma_desc *first, struct sgdma_desc *second,
               dma_addr_t second_bus)
{
    /*
     * remember reserved control in first descriptor, but zero
     * extra_adjacent!
     */
    u32 control = le32_to_cpu(first->control) & 0x00FFC0ffUL;
    /* second descriptor given? */
    if (second) {
        /*
         * link last descriptor of 1st array to first descriptor of
         * 2nd array
         */
        first->next_lo = cpu_to_le32(PCI_DMA_L(second_bus));
        first->next_hi = cpu_to_le32(PCI_DMA_H(second_bus));
        WARN_ON(first->next_hi);
        /* no second descriptor given */
    } else {
        /* first descriptor is the last */
        first->next_lo = 0;
        first->next_hi = 0;
    }
    /* merge magic, extra_adjacent and control field */
    control |= DESC_MAGIC;

    /* write bytes and next_num */
    first->control = cpu_to_le32(control);
}

/* sgdma_desc_adjacent -- Set how many descriptors are adjacent to this one */
void sgdma_desc_adjacent(struct sgdma_desc *desc, u32 next_adjacent)
{
#ifdef AL_IP_VER0
    /* remember reserved and control bits */
    u32 control = le32_to_cpu(desc->control) & 0xffffc0ffUL;

    if (next_adjacent)
        next_adjacent = next_adjacent - 1;
    if (next_adjacent > MAX_EXTRA_ADJ)
        next_adjacent = MAX_EXTRA_ADJ;

    /* merge adjacent and control field */
    control |= (next_adjacent << 8);
    /* write control and next_adjacent */
    desc->control = cpu_to_le32(control);
#else
    /* remember reserved and control bits */
    u32 control = le32_to_cpu(desc->control) & 0xfffff0ffUL;
    /* merge adjacent and control field */
    control |= (next_adjacent << 8);
    /* write control and next_adjacent */
    desc->control = cpu_to_le32(control);
#endif
}

/* sgdma_desc_control -- Set complete control field of a descriptor. */
int sgdma_desc_control_set(struct sgdma_desc *first, u32 control_field)
{
    /* remember magic and adjacent number */
    u32 control = le32_to_cpu(first->control) & ~(LS_BYTE_MASK);

    if (control_field & ~(LS_BYTE_MASK)) {
        pr_err("Invalid control field\n");
        return -EINVAL;
    }
    /* merge adjacent and control field */
    control |= control_field;
    /* write control and next_adjacent */
    first->control = cpu_to_le32(control);
    return 0;
}

/* sgdma_desc_clear -- Clear bits in control field of a descriptor. */
void sgdma_desc_control_clear(struct sgdma_desc *first, u32 clear_mask)
{
    /* remember magic and adjacent number */
    u32 control = le32_to_cpu(first->control);

    BUG_ON(clear_mask & ~(LS_BYTE_MASK));

    /* merge adjacent and control field */
    control &= (~clear_mask);
    /* write control and next_adjacent */
    first->control = cpu_to_le32(control);
}

/**
 * sgdma_get_next_adj()
 *
 * Get the number for adjacent descriptors to set in a descriptor, based on the
 * remaining number of descriptors and the lower bits of the address of the
 * next descriptor.
 * Since the number of descriptors in a page (SGDMA_PAGE_SIZE) is 128 and the
 * maximum size of a block of adjacent descriptors is 64 (63 max adjacent
 * descriptors for any descriptor), align the blocks of adjacent descriptors
 * to the block size.
 */
u32 sgdma_get_next_adj(unsigned int remaining, u32 next_lo)
{
    unsigned int next_index;

    dbg_desc("%s: remaining_desc %u, next_lo 0x%x\n",__func__, remaining,
            next_lo);

    if (remaining <= 1)
        return 0;

    /* shift right 5 times corresponds to a division by
     * sizeof(sgdma_desc) = 32
     */
    next_index = ((next_lo & (SGDMA_PAGE_SIZE - 1)) >> 5) %
        SGDMA_MAX_ADJ_BLOCK_SIZE;
    return min(SGDMA_MAX_ADJ_BLOCK_SIZE - next_index - 1, remaining - 1);
}

/**
 * sgdma_engine_stop() - stop an SG DMA engine
 *
 */
int sgdma_engine_stop(struct anlogic_dma_engine *engine)
{
    u32 w;
    struct anlogic_dev *sgdev = engine->sgdev;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }
    dbg_tfr("%s(engine=%p)\n", __func__, engine);

#ifdef __LIBSGDMA_DEBUG__ /* XXX FIXME do h2c test data check */
    #define H2C_RES_CHECK_REG 0x80004

    w = read_register(sgdev->bar[0] + H2C_RES_CHECK_REG);
    dbg_tfr("h2c data result check: %u\n", w);
    w = read_register(sgdev->bar[0] + 0x2010);
    w = read_register(sgdev->bar[0] + 0x20a0);
#endif

    if (enable_st_c2h_credit && engine->streaming &&
        engine->dir == DMA_FROM_DEVICE)
        write_register(0, &engine->sgdma_regs->credits, 
            (unsigned long)(&engine->sgdma_regs->credits) -
                (unsigned long)(engine->sgdma_regs));

    w = 0;
    w |= (u32)SGDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
    w |= (u32)SGDMA_CTRL_IE_MAGIC_STOPPED;
    w |= (u32)SGDMA_CTRL_IE_READ_ERROR;
    w |= (u32)SGDMA_CTRL_IE_DESC_ERROR;

    if (poll_mode) {
        w |= (u32)SGDMA_CTRL_POLL_MODE_WB;
        w |= (u32)SGDMA_CTRL_IE_DESC_COMPLETED; /* xxx fix me hwb */
    } else {
        w |= (u32)SGDMA_CTRL_IE_DESC_STOPPED;
        w |= (u32)SGDMA_CTRL_IE_DESC_COMPLETED;
    }

    dbg_tfr("Stopping SG DMA %s engine; writing 0x%08x to 0x%p.\n",
        engine->name, w, (u32 *)&engine->regs->control);
    write_register(w, &engine->regs->control,
            (unsigned long)(&engine->regs->control) -
                (unsigned long)(engine->regs));
    /* dummy read of status register to flush all previous writes */
    dbg_tfr("%s(%s) done\n", __func__, engine->name);
    engine->running = 0;
    engine->desc_idx = 0; /* XXX FIXME */
    return 0;
}

struct sgdma_request_cb * sgdma_request_alloc(unsigned int sdesc_nr)
{
    struct sgdma_request_cb *req;
    unsigned int size = sizeof(struct sgdma_request_cb) +
                sdesc_nr * sizeof(struct sw_desc);

    req = kzalloc(size, GFP_KERNEL);
    if (!req) {
        req = vmalloc(size);
        if (req)
            memset(req, 0, size);
    }
    if (!req) {
        pr_info("OOM, %u sw_desc, %u.\n", sdesc_nr, size);
        return NULL;
    }

    return req;
}

void sgdma_request_free(struct sgdma_request_cb *req)
{
    if (((unsigned long)req) >= VMALLOC_START &&
        ((unsigned long)req) < VMALLOC_END)
        vfree(req);
    else
        kfree(req);
}

void dump_desc(struct sgdma_desc *desc_virt)
{
    int j;
    u32 *p = (u32 *)desc_virt;
    static char *const field_name[] = { "magic|extra_adjacent|control",
                        "bytes",
                        "src_addr_lo",
                        "src_addr_hi",
                        "dst_addr_lo",
                        "dst_addr_hi",
                        "next_addr",
                        "next_addr_pad" };
    char *dummy;
    char control_field[64];

    /* remove warning about unused variable when debug printing is off */
    dummy = field_name[0];

    sprintf(control_field, "magic(0x%04x)|extra_adjacent(%d)|control(%s|%s|%s)", 
        desc_virt->control >> 16, 
        (desc_virt->control & 0x3F00) >> 8,
        ((desc_virt->control & 0x10) != 0) ? "EOP" : "   ",
        ((desc_virt->control & 0x2) != 0) ? "COMP" : "    ", 
        ((desc_virt->control & 0x1) != 0) ? "STOP" : "    ");

    for (j = 0; j < 8; j += 1) {
        pr_info("0x%08lx/0x%02lx: 0x%08x 0x%08x %s\n", (uintptr_t)p,
            (uintptr_t)p & 15, (int)*p, le32_to_cpu(*p),
            (j != 0) ? field_name[j] : control_field);
        p++;
    }
    pr_info("\n");
}

void transfer_dump(struct sgdma_transfer *transfer)
{
    int i;
    struct sgdma_desc *desc_virt = transfer->desc_virt;

    if (desc_virt == NULL)
        pr_info("desc virt is null\n");

    pr_info("xfer 0x%p, state 0x%x, f 0x%x, dir %d, len %u, last %d, desc_cmpl_th %d.\n",
        transfer, transfer->state, transfer->flags, transfer->dir,
        transfer->len, transfer->last_in_request, transfer->desc_cmpl_th);

    pr_info("transfer 0x%p, desc %d, bus 0x%llx, adj %d.\n", transfer,
        transfer->desc_num, (u64)transfer->desc_bus,
        transfer->desc_adjacent);
        
    //if (transfer->dir == DMA_FROM_DEVICE)
    	for (i = 0; i < transfer->desc_num; i += 1)
        	dump_desc(desc_virt + i);
}

void sgt_dump(struct sg_table *sgt)
{
    int i;
    struct scatterlist *sg = sgt->sgl;

    return;

    pr_info("sgt 0x%p, sgl 0x%p, nents %u/%u.\n", sgt, sgt->sgl, sgt->nents,
        sgt->orig_nents);

    for (i = 0; i < sgt->orig_nents; i++, sg = sg_next(sg))
        pr_info("%d, 0x%p, pg 0x%p,%u+%u, dma 0x%llx,%u.\n", i, sg,
            sg_page(sg), sg->offset, sg->length, sg_dma_address(sg),
            sg_dma_len(sg));
}

void sgdma_request_cb_dump(struct sgdma_request_cb *req)
{
    int i;

    pr_info("request 0x%p, total %u, ep 0x%llx, sw_desc %u, sgt 0x%p.\n",
        req, req->total_len, req->ep_addr, req->sw_desc_cnt, req->sgt);
    sgt_dump(req->sgt);
    for (i = 0; i < req->sw_desc_cnt; i++)
        pr_info("%d/%u, 0x%llx, %u.\n", i, req->sw_desc_cnt,
            req->sdesc[i].addr, req->sdesc[i].len);
}

struct sgdma_request_cb * sgdma_init_request(struct sg_table *sgt,
                        u64 ep_addr)
{
    struct sgdma_request_cb *req;
    struct scatterlist *sg = sgt->sgl;
    int max = sgt->nents;
    int extra = 0;
    int i, j = 0;

    for (i = 0;  i < max; i++, sg = sg_next(sg)) {
        unsigned int len = sg_dma_len(sg);

        if (unlikely(len > desc_blen_max))
            extra += (len + desc_blen_max - 1) / desc_blen_max;
    }

    max += extra;

    //pr_info("ep 0x%llx, desc %u extra %u.\n", ep_addr, max, extra);
    req = sgdma_request_alloc(max);
    if (!req)
        return NULL;

    req->sgt = sgt; 
    req->ep_addr = ep_addr;

    for (i = 0, sg = sgt->sgl;  i < sgt->nents; i++, sg = sg_next(sg)) {
        unsigned int tlen = sg_dma_len(sg);
        dma_addr_t addr = sg_dma_address(sg);
        void *sg_virt = page_address(sg_page(sg));

        //pr_info("tlen: %08x\n", tlen);
        //memset(sg_virt, 0, tlen); /* XXX FIX ME do c2h test use. */

        req->total_len += tlen;
        while (tlen) {
            req->sdesc[j].addr = addr;
            if (tlen > desc_blen_max) {
                req->sdesc[j].len = desc_blen_max;
                addr += desc_blen_max;
                tlen -= desc_blen_max;  
            } else {
                req->sdesc[j].len = tlen;
                tlen = 0;
            }
            j++;
        }
    }
    BUG_ON(j > max);

    req->sw_desc_cnt = j;

    //pr_info("req desc cnt: %d\n", j);

    return req;
}

/**
 * engine_start() - start an idle engine with its first transfer on queue
 *
 * The engine will run and process all transfers that are queued using
 * transfer_queue() and thus have their descriptor lists chained.
 *
 * During the run, new transfers will be processed if transfer_queue() has
 * chained the descriptors before the hardware fetches the last descriptor.
 * A transfer that was chained too late will invoke a new run of the engine
 * initiated from the engine_service() routine.
 *
 * The engine must be idle and at least one transfer must be queued.
 * This function does not take locks; the engine spinlock must already be
 * taken.
 *
 */
struct sgdma_transfer *engine_start(struct anlogic_dma_engine *engine)
{
    struct sgdma_transfer *transfer;
    u32 w, next_adj, extra_adj;
    int rv;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return NULL;
    }

    /* engine must be idle */
    if (engine->running) {
        pr_info("%s engine is not in idle state to start\n",
            engine->name);
        return NULL;
    }

    /* engine transfer queue must not be empty */
    if (list_empty(&engine->transfer_list)) {
        pr_debug("%s engine transfer queue must not be empty\n",
             engine->name);
        return NULL;
    }
    /* inspect first transfer queued on the engine */
    transfer = list_entry(engine->transfer_list.next, struct sgdma_transfer,
                  entry);
    if (!transfer) {
        pr_debug("%s queued transfer must not be empty\n",
             engine->name);
        return NULL;
    }

    /* engine is no longer shutdown */
    engine->shutdown = ENGINE_SHUTDOWN_NONE;

    dbg_tfr("%s(%s): transfer=0x%p.\n", __func__, engine->name, transfer);

    /* ring mode do not need init credit again */
    if (engine->cyclic_req == NULL) {
        /* Add credits for Streaming mode C2H */
        if (enable_st_c2h_credit && engine->streaming &&
            engine->dir == DMA_FROM_DEVICE)
            write_register(engine->desc_used,
                        &engine->sgdma_regs->credits, 
                   (unsigned long)(&engine->sgdma_regs->credits) -
                       (unsigned long)(engine->sgdma_regs));
    }

    /* initialize number of descriptors of dequeued transfers */
    engine->desc_dequeued = 0;

    //pr_info("%s ops reg\n", engine->name);

    /* write lower 32-bit of bus address of transfer first descriptor */
    w = cpu_to_le32(PCI_DMA_L(transfer->desc_bus));
    dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_lo)\n", w,
        (void *)&engine->sgdma_regs->first_desc_lo);
    write_register(w, &engine->sgdma_regs->first_desc_lo,
               (unsigned long)(&engine->sgdma_regs->first_desc_lo) -
                   (unsigned long)(engine->sgdma_regs));
    /* write upper 32-bit of bus address of transfer first descriptor */
    w = cpu_to_le32(PCI_DMA_H(transfer->desc_bus));
    dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_hi)\n", w,
        (void *)&engine->sgdma_regs->first_desc_hi);
    write_register(w, &engine->sgdma_regs->first_desc_hi,
               (unsigned long)(&engine->sgdma_regs->first_desc_hi) -
                   (unsigned long)(engine->sgdma_regs));
#ifdef AL_IP_VER0
    if (transfer->desc_adjacent > 2) {
        extra_adj = transfer->desc_adjacent - 2;
        if (extra_adj > MAX_EXTRA_ADJ) {
            extra_adj = MAX_EXTRA_ADJ;
        }
    } else {
        extra_adj = 0;
    }

    dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_adjacent)\n", extra_adj,
        (void *)&engine->sgdma_regs->first_desc_adjacent);

    write_register(
        extra_adj, &engine->sgdma_regs->first_desc_adjacent,
        (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
            (unsigned long)(engine->sgdma_regs));
#else
    next_adj = sgdma_get_next_adj(transfer->desc_adjacent,
                     cpu_to_le32(PCI_DMA_L(transfer->desc_bus)));

    dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_adjacent)\n", next_adj,
        (void *)&engine->sgdma_regs->first_desc_adjacent);

    write_register(
        next_adj, &engine->sgdma_regs->first_desc_adjacent,
        (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
            (unsigned long)(engine->sgdma_regs));
#endif

    dbg_tfr("ioread32(0x%p) (dummy read flushes writes).\n",
        &engine->regs->status);

#ifdef HAS_MMIOWB
    mmiowb();
#endif

    rv = engine_start_mode_config(engine);
    if (rv < 0) {
        pr_err("Failed to start engine mode config\n");
        return NULL;
    }

    rv = engine_status_read(engine, 0, 0);
    if (rv < 0) {
        pr_err("Failed to read engine status\n");
        return NULL;
    }
    dbg_tfr("%s engine 0x%p now running\n", engine->name, engine);
    /* remember the engine is running */
    engine->running = 1;
    return transfer;
}

/**
 * engine_service() - service an SG DMA engine
 *
 * must be called with engine->lock already acquired
 *
 * @engine pointer to struct sgdma_engine
 *
 */
int engine_service_shutdown(struct anlogic_dma_engine *engine)
{
    int rv;
    /* if the engine stopped with RUN still asserted, de-assert RUN now */

    dbg_tfr("engine just went idle, resetting RUN_STOP.\n");
    rv = sgdma_engine_stop(engine);
    if (rv < 0) {
        pr_err("Failed to stop engine\n");
        return rv;
    }

    /* awake task on engine's shutdown wait queue */
    al_wake_up(&engine->shutdown_wq);
    return 0;
}

static struct sgdma_transfer *engine_transfer_completion(
        struct anlogic_dma_engine *engine,
        struct sgdma_transfer *transfer)
{
    if (!engine) {
        pr_err("dma engine NULL\n");
        return NULL;
    }

    if (unlikely(!transfer)) {
        pr_info("%s: xfer empty.\n", engine->name);
        return NULL;
    }

    /* synchronous I/O? */
    /* awake task on transfer's wait queue */
    al_wake_up(&transfer->wq);

    /* Send completion notification for Last transfer */
    if (transfer->cb && transfer->last_in_request)
        transfer->cb->io_done((unsigned long)transfer->cb, 0);

    return transfer;
}

static struct sgdma_transfer *
engine_service_transfer_list(struct anlogic_dma_engine *engine,
                 struct sgdma_transfer *transfer,
                 u32 *pdesc_completed)
{
    if (!engine) {
        pr_err("dma engine NULL\n");
        return NULL;
    }

    if (!pdesc_completed) {
        pr_err("%s completed descriptors are null.\n", engine->name);
        return NULL;
    }

    if (unlikely(!transfer)) {
        pr_info("%s xfer empty, pdesc completed %u.\n", engine->name,
            *pdesc_completed);
        return NULL;
    }

    /*
     * iterate over all the transfers completed by the engine,
     * except for the last (i.e. use > instead of >=).
     */
   /* Must make sure transfer->desc_num < *pdesc_completed, this mean the transfer is finished all. */
    while (transfer && (!transfer->cyclic) &&
           (*pdesc_completed > transfer->desc_num)) {
        /* remove this transfer from pdesc_completed */
        *pdesc_completed -= transfer->desc_num;
        dbg_tfr("%s engine completed non-cyclic xfer 0x%p (%d desc)\n",
            engine->name, transfer, transfer->desc_num);

        /* remove completed transfer from list */
        list_del(engine->transfer_list.next);
        /* add to dequeued number of descriptors during this run */
        engine->desc_dequeued += transfer->desc_num;
        /* mark transfer as succesfully completed */
        transfer->state = TRANSFER_STATE_COMPLETED;

        /*
         * Complete transfer - sets transfer to NULL if an async
         * transfer has completed
         */
        transfer = engine_transfer_completion(engine, transfer);

        /* if exists, get the next transfer on the list */
        if (!list_empty(&engine->transfer_list)) {
            transfer = list_entry(engine->transfer_list.next,
                          struct sgdma_transfer, entry);
            dbg_tfr("Non-completed transfer %p\n", transfer);
        } else {
            /* no further transfers? */
            transfer = NULL;
        }
    }

    return transfer;
}

 static int engine_err_handle(struct anlogic_dma_engine *engine,
                  struct sgdma_transfer *transfer, u32 desc_completed)
 {
     u32 value;
     int rv = 0;
     /*
      * The BUSY bit is expected to be clear now but older HW has a race
      * condition which could cause it to be still set.  If it's set, re-read
      * and check again.  If it's still set, log the issue.
      */
     if (engine->status & SGDMA_STAT_BUSY) {
         value = read_register(&engine->regs->status);
         if ((value & SGDMA_STAT_BUSY))
             printk_ratelimited(KERN_INFO
                     "%s has errors but is still BUSY\n",
                     engine->name);
     }
 
     printk_ratelimited(KERN_INFO "%s, s 0x%x, aborted xfer 0x%p, cmpl %d/%d\n",
             engine->name, engine->status, transfer, desc_completed,
             transfer->desc_num);
 
     /* mark transfer as failed */
     transfer->state = TRANSFER_STATE_FAILED;
     rv = sgdma_engine_stop(engine);
     if (rv < 0)
         pr_err("Failed to stop engine\n");
     return rv;
 }

static struct sgdma_transfer *
engine_service_final_transfer(struct anlogic_dma_engine *engine,
                   struct sgdma_transfer *transfer,
                   u32 *pdesc_completed)
 {
     if (!engine) {
         pr_err("dma engine NULL\n");
         return NULL;
     }
 
     if (!pdesc_completed) {
         pr_err("%s completed descriptors are null.\n", engine->name);
         return NULL;
     }
 
     /* inspect the current transfer */
     if (unlikely(!transfer)) {
         pr_info("%s xfer empty, pdesc completed %u.\n", engine->name,
             *pdesc_completed);
         return NULL;
     }
 
     if (((engine->dir == DMA_FROM_DEVICE) &&
          (engine->status & SGDMA_STAT_C2H_ERR_MASK)) ||
         ((engine->dir == DMA_TO_DEVICE) &&
          (engine->status & SGDMA_STAT_H2C_ERR_MASK))) {
         pr_info("engine %s, status error 0x%x.\n", engine->name,
             engine->status);
         engine_status_dump(engine);
         engine_err_handle(engine, transfer, *pdesc_completed);
         goto transfer_del;
     }
 
     if (engine->status & SGDMA_STAT_BUSY)
         pr_debug("engine %s is unexpectedly busy - ignoring\n",
              engine->name);
 
     /* the engine stopped on current transfer? */
     if (*pdesc_completed < transfer->desc_num) {
        /* Here is telled us not all desc finished on current transfer. */
         if (engine->eop_flush) {
             /* check if eop received */
             struct sgdma_result *result = transfer->res_virt;
             int i;
             int max = *pdesc_completed;
 
             /* Foreach stream result write back addr until find the last desc. */
             for (i = 0; i < max; i++) {
                 if ((result[i].status & RX_STATUS_EOP) != 0) {
                     transfer->flags |=
                         XFER_FLAG_ST_C2H_EOP_RCVED;
                     break;
                 }
             }
 
             transfer->desc_cmpl += *pdesc_completed;
             if (!(transfer->flags & XFER_FLAG_ST_C2H_EOP_RCVED)) {
                 return NULL;
             }
 
             /* mark transfer as successfully completed */
             engine_service_shutdown(engine);
 
             transfer->state = TRANSFER_STATE_COMPLETED;
 
             engine->desc_dequeued += transfer->desc_cmpl;
         } else {
             transfer->state = TRANSFER_STATE_FAILED;
             pr_info("%s, xfer 0x%p, stopped half-way, %d/%d.\n",
                 engine->name, transfer, *pdesc_completed,
                 transfer->desc_num);
 
             /* add dequeued number of descriptors during this run */
             engine->desc_dequeued += transfer->desc_num;
             transfer->desc_cmpl = *pdesc_completed;
         }
     } else {
         /* Here is telled us a transfer is finished all desc on it. */
         dbg_tfr("engine %s completed transfer\n", engine->name);
         dbg_tfr("Completed transfer ID = 0x%p\n", transfer);
         dbg_tfr("*pdesc_completed=%d, transfer->desc_num=%d",
             *pdesc_completed, transfer->desc_num);
 
         if (!transfer->cyclic) {
             /*
              * if the engine stopped on this transfer,
              * it should be the last
              */
             WARN_ON(*pdesc_completed > transfer->desc_num);
         }
         /* mark transfer as successfully completed */
         transfer->state = TRANSFER_STATE_COMPLETED;
         transfer->desc_cmpl = transfer->desc_num;
         /* add dequeued number of descriptors during this run */
         engine->desc_dequeued += transfer->desc_num;
     }
 
 transfer_del:
     /* remove completed transfer from list */
     list_del(engine->transfer_list.next);
 
     /*
      * Complete transfer - sets transfer to NULL if an asynchronous
      * transfer has completed
      */
     transfer = engine_transfer_completion(engine, transfer);
 
     return transfer;

 }

static int engine_service_resume(struct anlogic_dma_engine *engine)
{
   struct sgdma_transfer *transfer_started;

   if (!engine) {
       pr_err("dma engine NULL\n");
       return -EINVAL;
   }

   /* engine stopped? */
   if (!engine->running) {
       /* in the case of shutdown, let it finish what's in the Q */
       if (!list_empty(&engine->transfer_list)) {
           /* (re)start engine */
           transfer_started = engine_start(engine);
           if (!transfer_started) {
               pr_err("Failed to start dma engine\n");
               return -EINVAL;
           }
           dbg_tfr("re-started %s engine with pending xfer 0x%p\n",
               engine->name, transfer_started);
           /* engine was requested to be shutdown? */
       } else if (engine->shutdown & ENGINE_SHUTDOWN_REQUEST) {
           engine->shutdown |= ENGINE_SHUTDOWN_IDLE;
           /* awake task on engine's shutdown wait queue */
           al_wake_up(&engine->shutdown_wq);
       } else {
           dbg_tfr("no pending transfers, %s engine stays idle.\n",
               engine->name);
       }
   } else if (list_empty(&engine->transfer_list)) {
       engine_service_shutdown(engine);
   }
   return 0;
}

/**
 * engine_service() - service an SG DMA engine
 *
 * must be called with engine->lock already acquired
 *
 * @engine pointer to struct sgdma_engine
 *
 */
static int engine_service(struct anlogic_dma_engine *engine, int desc_writeback)
{
    struct sgdma_transfer *transfer = NULL;
    u32 desc_count = desc_writeback & WB_COUNT_MASK;
    u32 err_flag = desc_writeback & WB_ERR_MASK;
    int rv = 0;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    /* Service the engine */
    if (!engine->running) {
        dbg_tfr("Engine was not running!!! Clearing status\n");
        rv = engine_status_read(engine, 1, 0);
        if (rv < 0) {
            pr_err("%s failed to read status\n", engine->name);
            return rv;
        }
        return 0;
    }

    /*
     * If called by the ISR or polling detected an error, read and clear
     * engine status. For polled mode descriptor completion, this read is
     * unnecessary and is skipped to reduce latency
     */
    if ((desc_count == 0) || (err_flag != 0)) {
        rv = engine_status_read(engine, 1, 0);
        if (rv < 0) {
            pr_err("Failed to read engine status\n");
            return rv;
        }
    }

    /*
     * engine was running but is no longer busy, or writeback occurred,
     * shut down
     */
    if ((engine->running && !(engine->status & SGDMA_STAT_BUSY)) ||
        (!engine->eop_flush && desc_count != 0)) {
        rv = engine_service_shutdown(engine);
        if (rv < 0) {
            pr_err("Failed to shutdown engine\n");
            return rv;
        }
    }

    /*
     * If called from the ISR, or if an error occurred, the descriptor
     * count will be zero.  In this scenario, read the descriptor count
     * from HW.  In polled mode descriptor completion, this read is
     * unnecessary and is skipped to reduce latency
     */
    if (!desc_count)
        desc_count = read_register(&engine->regs->completed_desc_count);

    dbg_tfr("%s wb 0x%x, desc_count %u, err %u, dequeued %u.\n",
        engine->name, desc_writeback, desc_count, err_flag,
        engine->desc_dequeued);

    if (!desc_count)
        goto done;

    /* transfers on queue? */
    if (!list_empty(&engine->transfer_list)) {
        /* pick first transfer on queue (was submitted to the engine) */
        transfer = list_entry(engine->transfer_list.next,
                      struct sgdma_transfer, entry);

        dbg_tfr("head of queue transfer 0x%p has %d descriptors\n",
            transfer, (int)transfer->desc_num);

        dbg_tfr("Engine completed %d desc, %d not yet dequeued\n",
            (int)desc_count,
            (int)desc_count - engine->desc_dequeued);
    }

    /* account for already dequeued transfers during this engine run */
    desc_count -= engine->desc_dequeued;

    /* Process all but the last transfer */
    transfer = engine_service_transfer_list(engine, transfer, &desc_count);

    /*
     * Process final transfer - includes checks of number of descriptors to
     * detect faulty completion
     */
    transfer = engine_service_final_transfer(engine, transfer, &desc_count);

    /* Restart the engine following the servicing */
    if (!engine->eop_flush) {
        rv = engine_service_resume(engine);
        if (rv < 0)
            pr_err("Failed to resume engine\n");
    }

done:
    /* If polling detected an error, signal to the caller */
    return err_flag ? -1 : 0;
}

/* engine_service_work */
static void engine_service_work(struct work_struct *work)
{
    struct anlogic_dma_engine *engine;
    unsigned long flags;
    int rv;

    engine = container_of(work, struct anlogic_dma_engine, work);
    if (engine->magic != MAGIC_ENGINE) {
        pr_err("%s has invalid magic number %lx\n", engine->name,
               engine->magic);
        return;
    }

    /* lock the engine */
    spin_lock_irqsave(&engine->lock, flags);

    dbg_tfr("engine_service() for %s engine %p\n", engine->name, engine);
    if (engine->cyclic_req) {
        rv = engine_service_cyclic(engine);
        if (rv < 0) {
            pr_err("Failed to service cyclic engine\n");
            goto unlock;
        }
    } else {
        rv = engine_service(engine, 0);
        if (rv < 0) {
            pr_err("Failed to service engine\n");
            goto unlock;
        }
    }
    /* re-enable interrupts for this engine */
    if (engine->sgdev->msix_enabled) {
        write_register(
            engine->interrupt_enable_mask_value,
            &engine->regs->interrupt_enable_mask_w1s,
            (unsigned long)(&engine->regs
                         ->interrupt_enable_mask_w1s) -
                (unsigned long)(&engine->regs));
    } else
        channel_interrupts_enable(engine->sgdev, engine->irq_bitmask);

    /* unlock the engine */
unlock:
    spin_unlock_irqrestore(&engine->lock, flags);
}

static u32 engine_service_wb_monitor(struct anlogic_dma_engine *engine,
                     u32 expected_wb)
{
    struct sgdma_poll_wb *wb_data;
    u32 desc_wb = 0;
    u32 sched_limit = 0;
    unsigned long timeout;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    /*
     * Poll the writeback location for the expected number of
     * descriptors / error events This loop is skipped for cyclic mode,
     * where the expected_desc_count passed in is zero, since it cannot be
     * determined before the function is called
     */

    timeout = jiffies + (POLL_TIMEOUT_SECONDS * HZ);
    while (expected_wb != 0) {

#if 1 /* FIXME XXX TODO h2c use wb as cnt ??? */
        desc_wb = wb_data->completed_desc_count;

        if (desc_wb)
            wb_data->completed_desc_count = 0;
#else
        desc_wb = read_register(&engine->regs->completed_desc_count);

        if (desc_wb)
            write_register(0, &engine->regs->completed_desc_count, 0);
#endif

        if (desc_wb & WB_ERR_MASK)
            break;
        else if ((desc_wb & WB_COUNT_MASK) >= expected_wb)
            break;

        /* prevent system from hanging in polled mode */
        if (time_after(jiffies, timeout)) {
            pr_info("Polling timeout occurred");
            pr_info("desc_wb = 0x%08x, expected 0x%08x\n", desc_wb,
                expected_wb);
            if ((desc_wb & WB_COUNT_MASK) > expected_wb)
                desc_wb = expected_wb | WB_ERR_MASK;

            break;
        }

        /*
         * Define NUM_POLLS_PER_SCHED to limit how much time is spent
         * in the scheduler
         */
        if (sched_limit != 0) {
            if ((sched_limit % NUM_POLLS_PER_SCHED) == 0)
                schedule();

        }
        sched_limit++;
    }

    return desc_wb;
}

int engine_service_poll(struct anlogic_dma_engine *engine,
                   u32 expected_desc_count)
{
    u32 desc_wb = 0;
    unsigned long flags;
    int rv = 0;

    dbg_tfr("%s service.\n", engine->name);

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (engine->magic != MAGIC_ENGINE) {
        pr_err("%s has invalid magic number %lx\n", engine->name,
               engine->magic);
        return -EINVAL;
    }

    if (engine->cyclic_req == NULL) {
        /*
         * Poll the writeback location for the expected number of
         * descriptors / error events This loop is skipped for cyclic mode,
         * where the expected_desc_count passed in is zero, since it cannot be
         * determined before the function is called
         */
        desc_wb = engine_service_wb_monitor(engine, expected_desc_count);
        if (!desc_wb) {
            pr_err("%s not get comp desc at this poll work.\n", engine->name);
            return 0;
        }
    }

    spin_lock_irqsave(&engine->lock, flags);

    if (engine->cyclic_req) {
        rv = engine_service_cyclic(engine);
    } else {
        rv = engine_service(engine, desc_wb);
    }

    spin_unlock_irqrestore(&engine->lock, flags);

    return rv;
}

/* transfer_queue() - Queue a DMA transfer on the engine
*
* @engine DMA engine doing the transfer
* @transfer DMA transfer submitted to the engine
*
* Takes and releases the engine spinlock
*/
int transfer_queue(struct anlogic_dma_engine *engine,
            struct sgdma_transfer *transfer)
{
    int rv = 0;
    struct sgdma_transfer *transfer_started;
    struct anlogic_dev *sgdev;
    unsigned long flags;

    if (!engine) {
       pr_err("dma engine NULL\n");
       return -EINVAL;
   }

    if (!engine->sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    if (!transfer) {
        pr_err("%s Invalid DMA transfer\n", engine->name);
        return -EINVAL;
    }

    if (transfer->desc_num == 0) {
        pr_err("%s void descriptors in the transfer list\n",
               engine->name);
        return -EINVAL;
    }
    dbg_tfr("%s (transfer=0x%p).\n", __func__, transfer);

    /* lock the engine state */
    spin_lock_irqsave(&engine->lock, flags);

    engine->prev_cpu = get_cpu();
    put_cpu();

    /* engine is being shutdown; do not accept new transfers */
    if (engine->shutdown & ENGINE_SHUTDOWN_REQUEST) {
        pr_info("engine %s offline, transfer 0x%p not queued.\n",
            engine->name, transfer);
        rv = -EBUSY;
        goto shutdown;
    }

    /* mark the transfer as submitted */
    transfer->state = TRANSFER_STATE_SUBMITTED;
    /* add transfer to the tail of the engine transfer queue */
    list_add_tail(&transfer->entry, &engine->transfer_list);

    /* engine is idle? */
    if (!engine->running) {
        /* start engine */
        dbg_tfr("%s(): starting %s engine.\n", __func__, engine->name);
        transfer_started = engine_start(engine);
        if (!transfer_started) {
            pr_err("Failed to start dma engine\n");
            goto shutdown;
        }
        dbg_tfr("transfer=0x%p started %s engine with transfer 0x%p.\n",
            transfer, engine->name, transfer_started);
    } else {
        dbg_tfr("transfer=0x%p queued, with %s engine running.\n",
            transfer, engine->name);
    }

shutdown:
    /* unlock the engine state */
    dbg_tfr("engine->running = %d\n", engine->running);
    spin_unlock_irqrestore(&engine->lock, flags);
    return rv;
}

/* transfer_destroy() - free transfer */
static void transfer_destroy(struct anlogic_dev *sgdev, struct sgdma_transfer *xfer)
{
    /* free descriptors */
    sgdma_desc_done(xfer->desc_virt);

    if (xfer->last_in_request && (xfer->flags & XFER_FLAG_NEED_UNMAP)) {
            struct sg_table *sgt = xfer->sgt;

        if (sgt->nents) {
            pci_unmap_sg(sgdev->pdev, sgt->sgl, sgt->nents,
                xfer->dir);
            sgt->nents = 0;
        }
    }
}

static int transfer_build(struct anlogic_dma_engine *engine,
            struct sgdma_request_cb *req, struct sgdma_transfer *xfer,
            unsigned int desc_max)
{
    struct sw_desc *sdesc = &(req->sdesc[req->sw_desc_idx]);
    int i = 0;
    int j = 0;
    dma_addr_t bus = xfer->res_bus;

    for (; i < desc_max; i++, j++, sdesc++) {

#ifdef __LIBSGDMA_DEBUG__
        dbg_desc("sw desc %d/%u: 0x%llx, 0x%x, ep 0x%llx.\n",
             i + req->sw_desc_idx, req->sw_desc_cnt, sdesc->addr,
             sdesc->len, req->ep_addr);
#endif

        /* fill in descriptor entry j with transfer details */
        sgdma_desc_set(xfer->desc_virt + j, sdesc->addr, req->ep_addr,
                  sdesc->len, xfer->dir);
        xfer->len += sdesc->len;

        /* for non-inc-add mode don't increment ep_addr */
        if (!engine->non_incr_addr)
            req->ep_addr += sdesc->len;

        /* replace source addresses with result write-back addresses */
        if (engine->streaming && engine->dir == DMA_FROM_DEVICE) {
            memset(xfer->res_virt + j, 0,
                sizeof(struct sgdma_result));
            xfer->desc_virt[j].src_addr_lo =
                        cpu_to_le32(PCI_DMA_L(bus));
            xfer->desc_virt[j].src_addr_hi =
                        cpu_to_le32(PCI_DMA_H(bus));
            bus += sizeof(struct sgdma_result);
        }

    }
    req->sw_desc_idx += desc_max;
    return 0;
}

/*
 * should hold the engine->lock;
 */
static int transfer_abort(struct anlogic_dma_engine *engine,
              struct sgdma_transfer *transfer)
{
    struct sgdma_transfer *head;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (!transfer) {
        pr_err("Invalid DMA transfer\n");
        return -EINVAL;
    }

    if (transfer->desc_num == 0) {
        pr_err("%s void descriptors in the transfer list\n",
               engine->name);
        return -EINVAL;
    }

    pr_info("abort transfer 0x%p, desc %d, engine desc queued %d.\n",
        transfer, transfer->desc_num, engine->desc_dequeued);

    head = list_entry(engine->transfer_list.next, struct sgdma_transfer,
              entry);
    if (head == transfer)
        list_del(engine->transfer_list.next);
    else
        pr_info("engine %s, transfer 0x%p NOT found, 0x%p.\n",
            engine->name, transfer, head);

    if (transfer->state == TRANSFER_STATE_SUBMITTED)
        transfer->state = TRANSFER_STATE_ABORTED;
    return 0;
}

int transfer_init(struct anlogic_dma_engine *engine,
            struct sgdma_request_cb *req, struct sgdma_transfer *xfer)
{
    unsigned int desc_max = min_t(unsigned int,
                req->sw_desc_cnt - req->sw_desc_idx,
                engine->desc_max);
    int i = 0;
    int last = 0;
    u32 control;
    unsigned long flags;

    dbg_sg("engine->desc_max %d, req->sw_desc_cnt %d, req->sw_desc_idx %d, desc_max %d.\n",
        engine->desc_max, req->sw_desc_cnt, req->sw_desc_idx, desc_max);

    memset(xfer, 0, sizeof(*xfer));

    /* lock the engine state */
    spin_lock_irqsave(&engine->lock, flags);
    /* initialize wait queue */
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
    init_swait_queue_head(&xfer->wq);
#else
    init_waitqueue_head(&xfer->wq);
#endif

    /* remember direction of transfer */
    xfer->dir = engine->dir;
    xfer->desc_virt = engine->desc + engine->desc_idx;
    xfer->res_virt = engine->cyclic_result + engine->desc_idx;
    xfer->desc_bus = engine->desc_bus +
            (sizeof(struct sgdma_desc) * engine->desc_idx);
    xfer->res_bus = engine->cyclic_result_bus +
            (sizeof(struct sgdma_result) * engine->desc_idx);
    xfer->desc_index = engine->desc_idx;

    /* Need to handle desc_used >= engine->desc_max */
    if ((engine->desc_idx + desc_max) >= engine->desc_max)
        desc_max = engine->desc_max - engine->desc_idx;

    transfer_desc_init(xfer, desc_max);

    dbg_sg("xfer= %p transfer->desc_bus = 0x%llx, engine->desc_idx = %d, desc_max = %d\n",
        xfer, (u64)xfer->desc_bus, engine->desc_idx, desc_max);
    transfer_build(engine, req, xfer, desc_max);

    xfer->desc_adjacent = desc_max;

    /* terminate last descriptor */
    last = desc_max - 1;
    /* stop engine, EOP for AXI ST, req IRQ on last descriptor */
    control = SGDMA_DESC_STOPPED;
    control |= SGDMA_DESC_EOP;
    control |= SGDMA_DESC_COMPLETED;
    sgdma_desc_control_set(xfer->desc_virt + last, control);

    /* XXX FIXME add eop for loop back test */
    for (i = 0; i < last; i++)
        sgdma_desc_control_set(xfer->desc_virt + i,
            SGDMA_DESC_EOP);

    if (engine->eop_flush) {
        for (i = 0; i < last; i++)
            sgdma_desc_control_set(xfer->desc_virt + i,
                    SGDMA_DESC_COMPLETED);
        xfer->desc_cmpl_th = 1;
    } else
        xfer->desc_cmpl_th = desc_max;

    xfer->desc_num = desc_max;
    engine->desc_idx = (engine->desc_idx + desc_max) % engine->desc_max;
    engine->desc_used += desc_max;

    /* fill in adjacent numbers */
#ifdef AL_IP_VER0
    for (i = 0; i < xfer->desc_num; i++) {
        u32 next_adj = xfer->desc_num - i - 1;
        sgdma_desc_adjacent(xfer->desc_virt + i, next_adj);
    }
#else
    for (i = 0; i < xfer->desc_num; i++) {
        u32 next_adj = sgdma_get_next_adj(xfer->desc_num - i - 1,
                        (xfer->desc_virt + i)->next_lo);

        dbg_desc("set next adj at index %d to %u\n", i, next_adj);
        sgdma_desc_adjacent(xfer->desc_virt + i, next_adj);
    }
#endif

#ifdef __LIBSGDMA_DEBUG__
{
    char desc_str[256];
    extern int format_desc(char *buf, int len, struct sgdma_desc *desc);
    for (i = 0; i < xfer->desc_num; i++) {
        format_desc(desc_str, 256, &xfer->desc_virt[i]);
        //printk("%s", desc_str);
    }
}
#endif

    spin_unlock_irqrestore(&engine->lock, flags);
    return 0;
}

ssize_t sgdma_xfer_aperture(struct anlogic_dma_engine *engine, bool write, u64 ep_addr,
            unsigned int aperture, struct sg_table *sgt,
            bool dma_mapped, int timeout_ms)
{
    struct anlogic_dev *sgdev;
    struct sgdma_request_cb *req = NULL;
    struct scatterlist *sg;
    enum dma_data_direction dir = write ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
    unsigned int maxlen = min_t(unsigned int, aperture, desc_blen_max);
    unsigned int sg_max;
    unsigned int tlen = 0;
    u64 ep_addr_max = ep_addr + aperture - 1;
    ssize_t done = 0;
    int i, rv = 0;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (engine->magic != MAGIC_ENGINE) {
        pr_err("%s has invalid magic number %lx\n", engine->name,
               engine->magic);
        return -EINVAL;
    }

    sgdev = engine->sgdev;
    if (sgdma_device_flag_check(sgdev, XDEV_FLAG_OFFLINE)) {
        pr_info("sgdev 0x%p, offline.\n", sgdev);
        return -EBUSY;
    }

    /* check the direction */
    if (engine->dir != dir) {
        pr_info("0x%p, %s, W %d, 0x%x/0x%x mismatch.\n", engine,
            engine->name, write, engine->dir, dir);
        return -EINVAL;
    }

    if (engine->streaming) {
        pr_info("%s aperture not supported in ST.\n", engine->name);
        return -EINVAL;
    }

    if (!dma_mapped) {
        sgt->nents = pci_map_sg(sgdev->pdev, sgt->sgl, sgt->orig_nents,
                    dir);
        if (!sgt->nents) {
            pr_info("map sgl failed, sgt 0x%p.\n", sgt);
            return -EIO;
        }
    } else if (!sgt->nents) {
        pr_err("sg table has invalid number of entries 0x%p.\n", sgt);
        return -EIO;
    }
    sg_max = sgt->nents;

    req = kzalloc(sizeof(struct sgdma_request_cb), GFP_KERNEL);
    if (!req) {
        rv = -ENOMEM;
        goto unmap_sgl;
    }
    memset(req, 0, sizeof(struct sgdma_request_cb));
    req->sgt = sgt;
    req->ep_addr = ep_addr;
    req->aperture = aperture;
    req->sg = sgt->sgl;

    for (i = 0, sg = sgt->sgl; i < sgt->nents; i++, sg = sg_next(sg))
        tlen += sg_dma_len(sg);
    req->total_len = tlen;

    dbg_tfr("%s, aperture: sg cnt %u.\n", engine->name, sgt->nents);

    mutex_lock(&engine->desc_lock);

    while (req->offset < req->total_len) {
        unsigned long flags;
        struct sgdma_transfer *xfer = &req->tfer[0];
        unsigned int sg_offset = req->sg_offset;
        unsigned int desc_idx, desc_max, desc_cnt = 0;
        struct sgdma_desc *desc_virt;
        dma_addr_t desc_bus;

        /* initialize transfer */
        memset(xfer, 0, sizeof(struct sgdma_transfer));
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
        init_swait_queue_head(&xfer->wq);
#else
        init_waitqueue_head(&xfer->wq);
#endif
        xfer->dir = engine->dir;
        if (!dma_mapped)
            xfer->flags = XFER_FLAG_NEED_UNMAP;

        spin_lock_irqsave(&engine->lock, flags);

        desc_idx = engine->desc_idx;
        desc_max = engine->desc_max;

        xfer->desc_virt = desc_virt = engine->desc + desc_idx;
        xfer->res_virt = engine->cyclic_result + desc_idx;
        xfer->desc_bus = desc_bus = engine->desc_bus +
                    (sizeof(struct sgdma_desc) * desc_idx);
        xfer->res_bus = engine->cyclic_result_bus +
                    (sizeof(struct sgdma_result) * desc_idx);
        xfer->desc_index = desc_idx;

        /* build transfer */
        sg = req->sg;
        ep_addr = req->ep_addr + (req->offset & (aperture - 1));
        i = req->sg_idx;
        
        for (sg = req->sg; i < sg_max && desc_idx < desc_max;
            i++, sg = sg_next(sg)) {
            dma_addr_t addr = sg_dma_address(sg);

            tlen = sg_dma_len(sg);
            if (sg_offset) {
                tlen -= sg_offset;
                addr += sg_offset;
            }

            while (tlen) {
                unsigned int len = min_t(unsigned int, tlen,
                                 maxlen);

                if (ep_addr > ep_addr_max)
                    ep_addr = req->ep_addr;

                if ((ep_addr + len) > ep_addr_max)
                    len = ep_addr_max - ep_addr + 1;

                sgdma_desc_set(engine->desc + desc_idx, addr,
                        ep_addr, len, dir);

                        dbg_desc("sg %d -> desc %u: ep 0x%llx, 0x%llx + %u \n",
                    i, desc_idx, ep_addr, addr, len);

                sg_offset += len;
                req->offset += len;
                xfer->len += len;
                ep_addr += len;
                addr += len;
                tlen -= len;
                
                desc_idx++;
                desc_cnt++;
                if (desc_idx == desc_max)
                    break;
            }

            if (!tlen)
                sg_offset = 0;
            else
                break;
        }
        
        req->sg_offset = sg_offset;
        req->sg_idx = i;

        xfer->desc_adjacent = desc_cnt;
        xfer->desc_num = desc_cnt;
        engine->desc_used += desc_cnt;

        /* create the desc linked list */
        for (i = 0; i < (desc_cnt - 1); i++, desc_virt++) {
            desc_bus += sizeof(struct sgdma_desc);
            /* singly-linked list uses bus addresses */
            desc_virt->next_lo = cpu_to_le32(PCI_DMA_L(desc_bus));
                    desc_virt->next_hi = cpu_to_le32(PCI_DMA_H(desc_bus));
                    desc_virt->control = cpu_to_le32(DESC_MAGIC);
        }
        /* terminate the last descriptor next pointer */
        desc_virt->next_lo = cpu_to_le32(0);
        desc_virt->next_hi = cpu_to_le32(0);
        desc_virt->control = cpu_to_le32(DESC_MAGIC |
                    SGDMA_DESC_STOPPED | SGDMA_DESC_EOP |
                    SGDMA_DESC_COMPLETED);

        xfer->desc_cmpl_th = desc_cnt;

        /* fill in adjacent numbers */
        for (i = 0; i < desc_cnt; i++) {
            u32 next_adj = sgdma_get_next_adj(desc_cnt - i - 1,
                    (xfer->desc_virt + i)->next_lo);

                    dbg_desc("set next adj at idx %d to %u\n", i, next_adj);
                    sgdma_desc_adjacent(xfer->desc_virt + i, next_adj);
            }

        engine->desc_idx = (engine->desc_idx + desc_cnt) % desc_max;
        spin_unlock_irqrestore(&engine->lock, flags);

        /* last transfer for the given request? */
        if (req->offset == req->total_len) {
            xfer->last_in_request = 1;
            xfer->sgt = sgt;
        }

        dbg_tfr("xfer %u,%u/%u, ep 0x%llx/0x%x, done %ld, sg %u/%u, desc %u.\n",
            xfer->len, req->offset, req->total_len, req->ep_addr,
            req->aperture, done, req->sg_idx, sg_max, desc_cnt);

        rv = transfer_queue(engine, xfer);
        if (rv < 0) {
            mutex_unlock(&engine->desc_lock);
            pr_info("unable to submit %s, %d.\n", engine->name, rv);
            goto unmap_sgl;
        }

        if (engine->cmplthp)
            sgdma_kthread_wakeup(engine->cmplthp);

        if (timeout_ms > 0)
            al_wait_event_interruptible_timeout(xfer->wq,
                (xfer->state != TRANSFER_STATE_SUBMITTED),
                msecs_to_jiffies(timeout_ms));
        else
            al_wait_event_interruptible(xfer->wq,
                (xfer->state != TRANSFER_STATE_SUBMITTED));

        spin_lock_irqsave(&engine->lock, flags);

        switch (xfer->state) {
        case TRANSFER_STATE_COMPLETED:
            spin_unlock_irqrestore(&engine->lock, flags);

            rv = 0;
            dbg_tfr("transfer %p, %u, ep 0x%llx compl, +%lu.\n",
                xfer, xfer->len, req->ep_addr - xfer->len,
                done);

            done += xfer->len;

            break;
        case TRANSFER_STATE_FAILED:
            pr_info("xfer 0x%p,%u, failed, ep 0x%llx.\n", xfer,
                xfer->len, req->ep_addr - xfer->len);
            spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -EIO;
            break;
        default:
            /* transfer can still be in-flight */
            pr_info("xfer 0x%p,%u, s 0x%x timed out, ep 0x%llx.\n",
                xfer, xfer->len, xfer->state, req->ep_addr);
            rv = engine_status_read(engine, 0, 1);
            if (rv < 0) {
                pr_err("Failed to read engine status\n");
            } else if (rv == 0) {
                //engine_status_dump(engine);
                rv = transfer_abort(engine, xfer);
                if (rv < 0) {
                    pr_err("Failed to stop engine\n");
                } else if (rv == 0) {
                    rv = sgdma_engine_stop(engine);
                    if (rv < 0)
                        pr_err("Failed to stop engine\n");
                }
            }
            spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -ERESTARTSYS;
            break;
        }

        engine->desc_used -= xfer->desc_num;
        transfer_destroy(sgdev, xfer);

        if (rv < 0) {
            mutex_unlock(&engine->desc_lock);
            goto unmap_sgl;
        }
    }
    mutex_unlock(&engine->desc_lock);

unmap_sgl:
    if (!dma_mapped && sgt->nents) {
        pci_unmap_sg(sgdev->pdev, sgt->sgl, sgt->orig_nents, dir);
        sgt->nents = 0;
    }

    if (req)
        sgdma_request_free(req);

    /* as long as some data is processed, return the count */
    return done ? done : rv;
}

ssize_t sgdma_xfer_submit(void *dev_hndl, int channel, bool write, u64 ep_addr,
             struct sg_table *sgt, bool dma_mapped, int timeout_ms)
{
    struct anlogic_dev *sgdev = (struct anlogic_dev *)dev_hndl;
    struct anlogic_dma_engine *engine;
    int rv = 0, tfer_idx = 0, i;
    ssize_t done = 0;
    struct scatterlist *sg = sgt->sgl;
    int nents;
    enum dma_data_direction dir = write ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
    struct sgdma_request_cb *req = NULL;

    if (!dev_hndl)
        return -EINVAL;

    if (write == 1) {
        if (channel >= sgdev->h2c_channel_max) {
            pr_err("H2C channel %d >= %d.\n", channel,
                sgdev->h2c_channel_max);
            return -EINVAL;
        }
        engine = &sgdev->engine_h2c[channel];
    } else if (write == 0) {
        if (channel >= sgdev->c2h_channel_max) {
            pr_err("C2H channel %d >= %d.\n", channel,
                sgdev->c2h_channel_max);
            return -EINVAL;
        }
        engine = &sgdev->engine_c2h[channel];
    }

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (engine->magic != MAGIC_ENGINE) {
        pr_err("%s has invalid magic number %lx\n", engine->name,
               engine->magic);
        return -EINVAL;
    }

    sgdev = engine->sgdev;
    if (sgdma_device_flag_check(sgdev, XDEV_FLAG_OFFLINE)) {
        pr_info("sgdev 0x%p, offline.\n", sgdev);
        return -EBUSY;
    }

    /* check the direction */
    if (engine->dir != dir) {
        pr_info("0x%p, %s, %d, W %d, 0x%x/0x%x mismatch.\n", engine,
            engine->name, channel, write, engine->dir, dir);
        return -EINVAL;
    }

    if (!dma_mapped) {
        nents = pci_map_sg(sgdev->pdev, sg, sgt->orig_nents, dir);
        if (!nents) {
            pr_info("map sgl failed, sgt 0x%p.\n", sgt);
            return -EIO;
        }
        sgt->nents = nents;
    } else {
        if (!sgt->nents) {
            pr_err("sg table has invalid number of entries 0x%p.\n",
                   sgt);
            return -EIO;
        }
    }

    req = sgdma_init_request(sgt, ep_addr);
    if (!req) {
        rv = -ENOMEM;
        goto unmap_sgl;
    }

    dbg_tfr("%s, len %u sg cnt %u.\n", engine->name, req->total_len,
        req->sw_desc_cnt);

    sg = sgt->sgl;
    nents = req->sw_desc_cnt;
    mutex_lock(&engine->desc_lock);

    while (nents) {
        unsigned long flags;
        struct sgdma_transfer *xfer;

        /* build transfer */
        rv = transfer_init(engine, req, &req->tfer[0]);
        if (rv < 0) {
            mutex_unlock(&engine->desc_lock);
            goto unmap_sgl;
        }
        xfer = &req->tfer[0];

        if (!dma_mapped)
            xfer->flags = XFER_FLAG_NEED_UNMAP;

        /* last transfer for the given request? */
        nents -= xfer->desc_num;
        if (!nents) {
            xfer->last_in_request = 1;
            xfer->sgt = sgt;
        }

        dbg_tfr("%s, poll_wb_cnt_phy 0x%llx, xfer %u, ep 0x%llx, done %lu, sg %u/%u.\n", engine->name, engine->poll_mode_bus, xfer->len,
            req->ep_addr, done, req->sw_desc_idx, req->sw_desc_cnt);

#ifdef __LIBSGDMA_DEBUG__
        transfer_dump(xfer);
#endif

        rv = transfer_queue(engine, xfer);
        if (rv < 0) {
            mutex_unlock(&engine->desc_lock);
            pr_info("unable to submit %s, %d.\n", engine->name, rv);
            goto unmap_sgl;
        }

        if (engine->cmplthp)
            sgdma_kthread_wakeup(engine->cmplthp);

        /* xxx hwb wait for transfer complete. */
        if (timeout_ms > 0)
            al_wait_event_interruptible_timeout(xfer->wq,
                (xfer->state != TRANSFER_STATE_SUBMITTED),
                msecs_to_jiffies(timeout_ms));
        else
            al_wait_event_interruptible(xfer->wq,
                (xfer->state != TRANSFER_STATE_SUBMITTED));

        spin_lock_irqsave(&engine->lock, flags);

        switch (xfer->state) {
        case TRANSFER_STATE_COMPLETED:
            spin_unlock_irqrestore(&engine->lock, flags);

            rv = 0;

            /* For C2H streaming use writeback results */
            if (engine->streaming &&
                engine->dir == DMA_FROM_DEVICE) {
                struct sgdma_result *result = xfer->res_virt;

                for (i = 0; i < xfer->desc_cmpl; i++) {
                    done += result[i].length;
				}

                /* finish the whole request */
                if (engine->eop_flush)
                    nents = 0;
            } else
                done += xfer->len;

            dbg_tfr("transfer %p, %u, ep 0x%llx compl, +%lu.\n",
                xfer, xfer->len, req->ep_addr - xfer->len,
                done);

            break;
        case TRANSFER_STATE_FAILED:
            pr_info("xfer 0x%p,%u, failed, ep 0x%llx.\n", xfer,
                xfer->len, req->ep_addr - xfer->len);
            spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -EIO;
            break;
        default:
            /* transfer can still be in-flight */
            pr_info("xfer 0x%p,%u, s 0x%x timed out, ep 0x%llx.\n",
                xfer, xfer->len, xfer->state, req->ep_addr);
            rv = engine_status_read(engine, 0, 1);
            if (rv < 0) {
                pr_err("Failed to read engine status\n");
            } else if (rv == 0) {
                engine_status_dump(engine);
                rv = transfer_abort(engine, xfer);
                if (rv < 0) {
                    pr_err("Failed to stop engine\n");
                } else if (rv == 0) {
                    rv = sgdma_engine_stop(engine);
                    if (rv < 0)
                        pr_err("Failed to stop engine\n");
                }
            }
            spin_unlock_irqrestore(&engine->lock, flags);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -ERESTARTSYS;
            break;
        }

        engine->desc_used -= xfer->desc_num;
        transfer_destroy(sgdev, xfer);

        /* use multiple transfers per request if we could not fit
         * all data within single descriptor chain.
         */
        tfer_idx++;

        if (rv < 0) {
            mutex_unlock(&engine->desc_lock);
            goto unmap_sgl;
        }
    } /* while (sg) */
    mutex_unlock(&engine->desc_lock);

unmap_sgl:
    if (!dma_mapped && sgt->nents) {
        pci_unmap_sg(sgdev->pdev, sgt->sgl, sgt->orig_nents, dir);
        sgt->nents = 0;
    }

    if (req)
        sgdma_request_free(req);

    /* as long as some data is processed, return the count */
    return done ? done : rv;
}

 int sgdma_performance_submit(struct anlogic_dev *sgdev, struct anlogic_dma_engine *engine)
 {
     u32 max_consistent_size = SGDMA_PERF_NUM_DESC * 32 * 1024; /* 1024 pages, 4MB */
     struct sgdma_transfer *transfer;
     u64 ep_addr = 0;
     int num_desc_in_a_loop = SGDMA_PERF_NUM_DESC;
     int size_in_desc = engine->sgdma_perf->transfer_size;
     int size = size_in_desc * num_desc_in_a_loop;
     int i;
     int rv = -ENOMEM;
     unsigned char free_desc = 0;
 
     if (size_in_desc > max_consistent_size) {
         pr_err("%s max consistent size %d is more than supported %d\n",
                engine->name, size_in_desc, max_consistent_size);
         return -EINVAL;
     }
 
     if (size > max_consistent_size) {
         size = max_consistent_size;
         num_desc_in_a_loop = size / size_in_desc;
     }
 
     engine->perf_buf_virt = dma_alloc_coherent(&sgdev->pdev->dev, size_in_desc,
                            &engine->perf_buf_bus,
                      GFP_KERNEL);
     if (!engine->perf_buf_virt) {
         pr_err("dev %s, %s DMA allocation OOM.\n",
                dev_name(&sgdev->pdev->dev), engine->name);
         return rv;
     }
 
     /* allocate transfer data structure */
     transfer = kzalloc(sizeof(struct sgdma_transfer), GFP_KERNEL);
     if (!transfer) {
         pr_err("dev %s, %s transfer request OOM.\n",
                dev_name(&sgdev->pdev->dev), engine->name);
         goto err_engine_transfer;
     }
     /* 0 = write engine (to_dev=0) , 1 = read engine (to_dev=1) */
     transfer->dir = engine->dir;
     /* set number of descriptors */
     transfer->desc_num = num_desc_in_a_loop;
 
     /* allocate descriptor list */
     if (!engine->desc) {
         engine->desc = dma_alloc_coherent(
             &sgdev->pdev->dev,
             num_desc_in_a_loop * sizeof(struct sgdma_desc),
             &engine->desc_bus, GFP_KERNEL);
         if (!engine->desc) {
             pr_err("%s DMA memory allocation for descriptors failed\n",
                    engine->name);
             goto err_engine_desc;
         }
         dbg_init("device %s, engine %s pre-alloc desc 0x%p,0x%llx.\n",
              dev_name(&sgdev->pdev->dev), engine->name, engine->desc,
              engine->desc_bus);
         free_desc = 1;
     }
     transfer->desc_virt = engine->desc;
     transfer->desc_bus = engine->desc_bus;
 
     rv = transfer_desc_init(transfer, transfer->desc_num);
     if (rv < 0) {
         pr_err("Failed to initialize descriptors\n");
         goto err_dma_desc;
     }
 
     dbg_sg("transfer->desc_bus = 0x%llx.\n", (u64)transfer->desc_bus);
 
     for (i = 0; i < transfer->desc_num; i++) {
         struct sgdma_desc *desc = transfer->desc_virt + i;
         dma_addr_t rc_bus_addr = engine->perf_buf_bus/* +
                 (size_in_desc * i)*/;
 
         /* fill in descriptor entry with transfer details */
         sgdma_desc_set(desc, rc_bus_addr, ep_addr, size_in_desc,
                   engine->dir);
     }
 
     /* stop engine and request interrupt on last descriptor */
     rv = sgdma_desc_control_set(transfer->desc_virt, 0);
     if (rv < 0) {
         pr_err("Failed to set desc control\n");
         goto err_dma_desc;
     }
     /* create a linked loop */
     sgdma_desc_link(transfer->desc_virt + transfer->desc_num - 1,
                transfer->desc_virt, transfer->desc_bus);

     /* XXX FIXME init adjacent num */
     transfer->desc_adjacent = num_desc_in_a_loop;

     /* XXX FIXME fill in adjacent numbers */
#ifdef AL_IP_VER0
     for (i = 0; i < transfer->desc_num; i++) {
         u32 next_adj = transfer->desc_num - i - 1;
         sgdma_desc_adjacent(transfer->desc_virt + i, next_adj);
     }
#else
     for (i = 0; i < transfer->desc_num; i++) {
         u32 next_adj = sgdma_get_next_adj(transfer->desc_num - i - 1,
                         (transfer->desc_virt + i)->next_lo);
 
         dbg_desc("set next adj at index %d to %u\n", i, next_adj);
         sgdma_desc_adjacent(transfer->desc_virt + i, next_adj);
     }
#endif

     transfer->cyclic = 1;

     /* initialize wait queue */
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
     init_swait_queue_head(&transfer->wq);
#else
     init_waitqueue_head(&transfer->wq);
#endif
 
     printk("=== Descriptor print for PERF\n");
     transfer_dump(transfer);
 
     dbg_perf("Queueing SGDMA I/O %s request for performance measurement.\n",
          engine->dir ? "write (to dev)" : "read (from dev)");
     rv = transfer_queue(engine, transfer);
     if (rv < 0) {
         pr_err("Failed to queue transfer\n");
         goto err_dma_desc;
     }
     return 0;
 
 err_dma_desc:
     if (free_desc && engine->desc)
         dma_free_coherent(&sgdev->pdev->dev,
                   num_desc_in_a_loop * sizeof(struct sgdma_desc),
                   engine->desc, engine->desc_bus);
     engine->desc = NULL;
 err_engine_desc:
     if (transfer)
         list_del(&transfer->entry);
     kfree(transfer);
     transfer = NULL;
 err_engine_transfer:
     if (engine->perf_buf_virt)
         dma_free_coherent(&sgdev->pdev->dev, size_in_desc, engine->perf_buf_virt,
                   engine->perf_buf_bus);
     engine->perf_buf_virt = NULL;
     return rv;
 }
 EXPORT_SYMBOL_GPL(sgdma_performance_submit);

ssize_t sgdma_xfer_completion(void *cb_hndl, void *dev_hndl, int channel,
            bool write, u64 ep_addr, struct sg_table *sgt,
            bool dma_mapped, int timeout_ms)
{
    struct anlogic_dev *sgdev = (struct anlogic_dev *)dev_hndl;
    struct sgdma_io_cb *cb = (struct sgdma_io_cb *)cb_hndl;
    struct anlogic_dma_engine *engine;
    int rv = 0, tfer_idx = 0;
    ssize_t done = 0;
    int nents;
    enum dma_data_direction dir = write ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
    struct sgdma_request_cb *req = NULL;
    struct sgdma_transfer *xfer;
    int i;
    struct sgdma_result *result;

    if (write == 1) {
        if (channel >= sgdev->h2c_channel_max) {
            pr_warn("H2C channel %d >= %d.\n",
                channel, sgdev->h2c_channel_max);
            return -EINVAL;
        }
        engine = &sgdev->engine_h2c[channel];
    } else if (write == 0) {
        if (channel >= sgdev->c2h_channel_max) {
            pr_warn("C2H channel %d >= %d.\n",
                channel, sgdev->c2h_channel_max);
            return -EINVAL;
        }
        engine = &sgdev->engine_c2h[channel];
    } else {
        pr_warn("write %d, exp. 0|1.\n", write);
        return -EINVAL;
    }

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (engine->magic != MAGIC_ENGINE) {
        pr_err("%s has invalid magic number %lx\n", engine->name,
               engine->magic);
        return -EINVAL;
    }

    sgdev = engine->sgdev;
    req = cb->req;

    nents = req->sw_desc_cnt;
    while (nents) {
        xfer = &req->tfer[tfer_idx];
        nents -= xfer->desc_num;
        switch (xfer->state) {
        case TRANSFER_STATE_COMPLETED:
            dbg_tfr("transfer %p, %u, ep 0x%llx compl, +%lu.\n",
                xfer, xfer->len, req->ep_addr - xfer->len,
                done);

            result = xfer->res_virt;
            /* For C2H streaming use writeback results */
            if (engine->streaming &&
                engine->dir == DMA_FROM_DEVICE) {
                for (i = 0; i < xfer->desc_num; i++)
                    done += result[i].length;
            } else
                done += xfer->len;

            rv = 0;
            break;
        case TRANSFER_STATE_FAILED:
            pr_info("xfer 0x%p,%u, failed, ep 0x%llx.\n",
                xfer, xfer->len, req->ep_addr - xfer->len);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -EIO;
            break;
        default:
            /* transfer can still be in-flight */
            pr_info("xfer 0x%p,%u, s 0x%x timed out, ep 0x%llx.\n",
                xfer, xfer->len, xfer->state, req->ep_addr);
            engine_status_read(engine, 0, 1);
            engine_status_dump(engine);
            transfer_abort(engine, xfer);

            sgdma_engine_stop(engine);

#ifdef __LIBSGDMA_DEBUG__
            transfer_dump(xfer);
            sgt_dump(sgt);
#endif
            rv = -ERESTARTSYS;
            break;
        }

        transfer_destroy(sgdev, xfer);
        engine->desc_used -= xfer->desc_num;

        tfer_idx++;

        if (rv < 0)
            goto unmap_sgl;
    } /* while (sg) */

unmap_sgl:
    if (!dma_mapped && sgt->nents) {
        pci_unmap_sg(sgdev->pdev, sgt->sgl, sgt->orig_nents, dir);
        sgt->nents = 0;
    }

    if (req)
        sgdma_request_free(req);

    return done;
}

/*
 * anlogic_isr() - Interrupt handler for legacy and msi interrupt
 *
 * @dev_id pointer to anlogic_dev
 */
static irqreturn_t anlogic_isr(int irq, void *dev_id)
{
    u32 ch_irq;
    u32 user_irq;
    u32 mask;
    struct anlogic_dev *sgdev;
    struct interrupt_regs *irq_regs;

    if (!dev_id) {
        pr_err("Invalid dev_id on irq line %d\n", irq);
        return -IRQ_NONE;
    }
    sgdev = (struct anlogic_dev *)dev_id;

    if (!sgdev) {
        WARN_ON(!sgdev);
        dbg_irq("%s(irq=%d) sgdev=%p ??\n", __func__, irq, sgdev);
        return IRQ_NONE;
    }

    irq_regs = (struct interrupt_regs *)(sgdev->bar[sgdev->config_bar_idx] +
                         SGDMA_OFS_INT_CTRL);
                        

    /* read channel interrupt requests */
    ch_irq = read_register(&irq_regs->channel_int_request);
    dbg_irq("ch_irq = 0x%08x\n", ch_irq);

    /*
     * disable all interrupts that fired; these are re-enabled individually
     * after the causing module has been fully serviced.
     */
   
    if (ch_irq)
        channel_interrupts_disable(sgdev, ch_irq);

    /* read user interrupts - this read also flushes the above write */
    user_irq = read_register(&irq_regs->user_int_request);
    dbg_irq("user_irq = 0x%08x\n", user_irq);

    if (user_irq) {
        int user = 0;
        u32 mask = 1;
        int max = sgdev->user_max;

        for (; user < max && user_irq; user++, mask <<= 1) {
            if (user_irq & mask) {
                user_irq &= ~mask;
                user_irq_service(irq, &sgdev->user_irq[user]);
            }
        }
    }

    mask = ch_irq & sgdev->mask_irq_h2c;
    if (mask) {
        int channel = 0;
        int max = sgdev->h2c_channel_max;

        // iterate over H2C (PCIe read) 
        for (channel = 0; channel < max && mask; channel++) {
            struct anlogic_dma_engine *engine = &sgdev->engine_h2c[channel];

            // engine present and its interrupt fired? 
            if ((engine->irq_bitmask & mask) &&
                (engine->magic == MAGIC_ENGINE)) {
                mask &= ~engine->irq_bitmask;
                dbg_tfr("schedule_work, %s.\n", engine->name);
                schedule_work(&engine->work);
            }
        }
    }

    mask = ch_irq & sgdev->mask_irq_c2h;
    if (mask) {
        int channel = 0;
        int max = sgdev->c2h_channel_max;

        // iterate over C2H (PCIe write) 
        for (channel = 0; channel < max && mask; channel++) {
            struct anlogic_dma_engine *engine = &sgdev->engine_c2h[channel];

            // engine present and its interrupt fired? 
            if ((engine->irq_bitmask & mask) &&
                (engine->magic == MAGIC_ENGINE)) {
                mask &= ~engine->irq_bitmask;
                dbg_tfr("schedule_work, %s.\n", engine->name);
                schedule_work(&engine->work);
            }
        }
    }

    sgdev->irq_count++;

    return IRQ_HANDLED;
}

static inline void sgdma_device_flag_clear(struct anlogic_dev *sgdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&sgdev->lock, flags);
    sgdev->flags &= ~f;
    spin_unlock_irqrestore(&sgdev->lock, flags);
}

static inline void sgdma_device_flag_set(struct anlogic_dev *sgdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&sgdev->lock, flags);
    sgdev->flags |= f;
    spin_unlock_irqrestore(&sgdev->lock, flags);
}

static int set_dma_mask(struct pci_dev *pdev)
{
    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    dbg_init("sizeof(dma_addr_t) == %ld\n", sizeof(dma_addr_t));
    /* 64-bit addressing capability for SGDMA? */
    if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
        /* query for DMA transfer */
        /* @see Documentation/DMA-mapping.txt */
        dbg_init("pci_set_dma_mask()\n");
        /* use 64-bit DMA */
        dbg_init("Using a 64-bit DMA mask.\n");
        /* use 32-bit DMA for descriptors */
        pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
        /* use 64-bit DMA, 32-bit for consistent */
    } else if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
        dbg_init("Could not set 64-bit DMA mask.\n");
        pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
        /* use 32-bit DMA */
        dbg_init("Using a 32-bit DMA mask.\n");
    } else {
        dbg_init("No suitable DMA possible.\n");
        return -EINVAL;
    }

    return 0;
}

/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
static void unmap_bars(struct anlogic_dev *sgdev, struct pci_dev *dev)
{
    int i;

    for (i = 0; i < SGDMA_BAR_NUM; i++) {
        /* is this BAR mapped? */
        if (sgdev->bar[i]) {
            /* unmap BAR */
            pci_iounmap(dev, sgdev->bar[i]);
            /* mark as unmapped */
            sgdev->bar[i] = NULL;
        }
    }
}

static int map_single_bar(struct anlogic_dev *sgdev, struct pci_dev *dev, int idx)
{
    resource_size_t bar_start;
    resource_size_t bar_len;
    resource_size_t map_len;

    bar_start = pci_resource_start(dev, idx);
    bar_len = pci_resource_len(dev, idx);
    map_len = bar_len;

    sgdev->bar[idx] = NULL;

    /* do not map BARs with length 0. Note that start MAY be 0! */
    if (!bar_len) {
        pr_info("BAR #%d is not present - skipping\n", idx);
        return 0;
    }

    /* BAR size exceeds maximum desired mapping? */
    if (bar_len > INT_MAX) {
        pr_info("Limit BAR %d mapping from %llu to %d bytes\n", idx,
            (u64)bar_len, INT_MAX);
        map_len = (resource_size_t)INT_MAX;
    }
    /*
     * map the full device memory or IO region into kernel virtual
     * address space
     */
    dbg_init("BAR%d: %llu bytes to be mapped.\n", idx, (u64)map_len);
    sgdev->bar[idx] = pci_iomap(dev, idx, map_len);

    if (!sgdev->bar[idx]) {
        pr_info("Could not map BAR %d.\n", idx);
        return -1;
    }

    pr_info("BAR%d at 0x%llx mapped at 0x%pK, length=%llu(/%llu)\n", idx,
        (u64)bar_start, sgdev->bar[idx], (u64)map_len, (u64)bar_len);

    return (int)map_len;
}

/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed
 */
static int map_bars(struct anlogic_dev *sgdev, struct pci_dev *dev)
{
    int rv;

#ifdef SGDMA_CONFIG_BAR_NUM
    rv = map_single_bar(sgdev, dev, SGDMA_CONFIG_BAR_NUM);
    if (rv <= 0) {
        pr_info("%s, map config bar %d failed, %d.\n",
            dev_name(&dev->dev), SGDMA_CONFIG_BAR_NUM, rv);
        return -EINVAL;
    }

    if (is_config_bar(sgdev, SGDMA_CONFIG_BAR_NUM) == 0) {
        pr_info("%s, unable to identify config bar %d.\n",
            dev_name(&dev->dev), SGDMA_CONFIG_BAR_NUM);
        return -EINVAL;
    }
    sgdev->config_bar_idx = SGDMA_CONFIG_BAR_NUM;

    return 0;
#else
    int i;
    int bar_id_list[SGDMA_BAR_NUM];
    int bar_id_idx = 0;

    /* iterate through all the BARs */
    for (i = 0; i < SGDMA_BAR_NUM; i++) {
        int bar_len;

        bar_len = map_single_bar(sgdev, dev, i);
        pr_err("BAR%d len:%d\n",i,bar_len);
        if (bar_len == 0) {
            continue;
        } else if (bar_len < 0) {
            rv = -EINVAL;
            goto fail;
        }

        /* Force BAR0 as SGDMA control BAR */
        sgdev->config_bar_idx = 0;

        bar_id_list[bar_id_idx] = i;
        bar_id_idx++;
    }

    /* The SGDMA config BAR must always be present */
    if (sgdev->config_bar_idx < 0) {
        pr_info("Failed to detect SGDMA config BAR\n");
        rv = -EINVAL;
        goto fail;
    }

    /* successfully mapped all required BAR regions */
    return 0;

fail:
    /* unwind; unmap any BARs that we did map */
    unmap_bars(sgdev, dev);
    return rv;

#endif
}

static int request_regions(struct anlogic_dev *sgdev, struct pci_dev *pdev)
{
    int rv;

    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    dbg_init("pci_request_regions()\n");
    rv = pci_request_regions(pdev, sgdev->mod_name);
    /* could not request all regions? */
    if (rv) {
        dbg_init("pci_request_regions() = %d, device in use?\n", rv);
        /* assume device is in use so do not disable it later */
        sgdev->regions_in_use = 1;
    } else {
        sgdev->got_regions = 1;
    }

    return rv;
}

static inline void anlogic_device_flag_set(struct anlogic_dev *sgdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&sgdev->lock, flags);
    sgdev->flags |= f;
    spin_unlock_irqrestore(&sgdev->lock, flags);
}

static struct anlogic_dev *alloc_dev_instance(struct pci_dev *pdev)
{
    int i;
    struct anlogic_dev *sgdev;
    struct anlogic_dma_engine *engine;

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return NULL;
    }

    /* allocate zeroed device book keeping structure */
    sgdev = kzalloc(sizeof(struct anlogic_dev), GFP_KERNEL);
    if (!sgdev) {
        pr_info("OOM, sgdma_dev.\n");
        return NULL;
    }
    spin_lock_init(&sgdev->lock);

    sgdev->magic = MAGIC_DEVICE;
    sgdev->config_bar_idx = -1;
#ifdef SGDMA_USER_BAR_NUM
    sgdev->user_bar_idx = SGDMA_USER_BAR_NUM;
#else
    sgdev->user_bar_idx = -1;
#endif
    sgdev->bypass_bar_idx = -1;
    sgdev->irq_line = -1;

    /* create a driver to device reference */
    sgdev->pdev = pdev;
    dbg_init("sgdev = 0x%p\n", sgdev);

    engine = sgdev->engine_h2c;
    for (i = 0; i < SGDMA_CHANNEL_NUM_MAX; i++, engine++) {
        spin_lock_init(&engine->lock);
        mutex_init(&engine->desc_lock);
        INIT_LIST_HEAD(&engine->transfer_list);
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
        init_swait_queue_head(&engine->shutdown_wq);
        init_swait_queue_head(&engine->sgdma_perf_wq);
#else
        init_waitqueue_head(&engine->shutdown_wq);
        init_waitqueue_head(&engine->sgdma_perf_wq);
#endif
    }

    engine = sgdev->engine_c2h;
    for (i = 0; i < SGDMA_CHANNEL_NUM_MAX; i++, engine++) {
        spin_lock_init(&engine->lock);
        mutex_init(&engine->desc_lock);
        INIT_LIST_HEAD(&engine->transfer_list);
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
        init_swait_queue_head(&engine->shutdown_wq);
        init_swait_queue_head(&engine->sgdma_perf_wq);
#else
        init_waitqueue_head(&engine->shutdown_wq);
        init_waitqueue_head(&engine->sgdma_perf_wq);
#endif
    }

    return sgdev;
}

#if KERNEL_VERSION(3, 5, 0) <= LINUX_VERSION_CODE
static void pci_enable_capability(struct pci_dev *pdev, int cap)
{
    pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, cap);
}
#else
static void pci_enable_capability(struct pci_dev *pdev, int cap)
{
    u16 v;
    int pos;

    pos = pci_pcie_cap(pdev);
    if (pos > 0) {
        pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
        v |= cap;
        pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
    }
}
#endif

static void engine_alignments(struct anlogic_dma_engine *engine)
{
    u32 w;
    u32 align_bytes;
    u32 granularity_bytes;
    u32 address_bits;

    w = read_register(&engine->regs->alignments);
    dbg_init("engine %p name %s alignments=0x%08x\n", engine, engine->name,
         (int)w);

    align_bytes = (w & 0x00ff0000U) >> 16;
    granularity_bytes = (w & 0x0000ff00U) >> 8;
    address_bits = (w & 0x000000ffU);

    dbg_init("align_bytes = %d\n", align_bytes);
    dbg_init("granularity_bytes = %d\n", granularity_bytes);
    dbg_init("address_bits = %d\n", address_bits);

    if (w) {
        engine->addr_align = align_bytes;
        engine->len_granularity = granularity_bytes;
        engine->addr_bits = address_bits;
    } else {
        /* Some default values if alignments are unspecified */
        engine->addr_align = 1;
        engine->len_granularity = 1;
        engine->addr_bits = 64;
    }
}

static void engine_free_resource(struct anlogic_dma_engine *engine)
{
    struct anlogic_dev *sgdev = engine->sgdev;

    /* Release memory use for descriptor writebacks */
    if (engine->poll_mode_addr_virt) {
        dbg_sg("Releasing memory for descriptor writeback\n");
        dma_free_coherent(&sgdev->pdev->dev, sizeof(struct sgdma_poll_wb),
                  engine->poll_mode_addr_virt,
                  engine->poll_mode_bus);
        dbg_sg("Released memory for descriptor writeback\n");
        engine->poll_mode_addr_virt = NULL;
    }

    if (engine->desc) {
        dbg_init("device %s, engine %s pre-alloc desc 0x%p,0x%llx.\n",
             dev_name(&sgdev->pdev->dev), engine->name, engine->desc,
             engine->desc_bus);
        dma_free_coherent(&sgdev->pdev->dev,
                  engine->desc_max * sizeof(struct sgdma_desc),
                  engine->desc, engine->desc_bus);
        engine->desc = NULL;
    }

    if (engine->cyclic_result) {
        dma_free_coherent(
            &sgdev->pdev->dev,
            engine->desc_max * sizeof(struct sgdma_result),
            engine->cyclic_result, engine->cyclic_result_bus);
        engine->cyclic_result = NULL;
    }
}

static int engine_alloc_resource(struct anlogic_dma_engine *engine)
{
    struct anlogic_dev *sgdev = engine->sgdev;

    engine->desc = dma_alloc_coherent(&sgdev->pdev->dev,
                      engine->desc_max *
                          sizeof(struct sgdma_desc),
                      &engine->desc_bus, GFP_KERNEL);
    if (!engine->desc) {
        pr_warn("dev %s, %s pre-alloc desc OOM.\n",
            dev_name(&sgdev->pdev->dev), engine->name);
        goto err_out;
    }

    if (poll_mode) {
        engine->poll_mode_addr_virt =
            dma_alloc_coherent(&sgdev->pdev->dev,
                       sizeof(struct sgdma_poll_wb),
                       &engine->poll_mode_bus, GFP_KERNEL);
        if (!engine->poll_mode_addr_virt) {
            pr_warn("%s, %s poll pre-alloc writeback OOM.\n",
                dev_name(&sgdev->pdev->dev), engine->name);
            goto err_out;
        }
    }

    if (engine->streaming && engine->dir == DMA_FROM_DEVICE) {
        engine->cyclic_result = dma_alloc_coherent(
            &sgdev->pdev->dev,
            engine->desc_max * sizeof(struct sgdma_result),
            &engine->cyclic_result_bus, GFP_KERNEL);

        if (!engine->cyclic_result) {
            pr_warn("%s, %s pre-alloc result OOM.\n",
                dev_name(&sgdev->pdev->dev), engine->name);
            goto err_out;
        }
    }

    return 0;

err_out:
    engine_free_resource(engine);
    return -ENOMEM;
}

/**
 *engine_cyclic_stop() - stop a cyclic transfer running on an SG DMA engine
 *
 *engine->lock must be taken
 */
struct sgdma_transfer *engine_cyclic_stop(struct anlogic_dma_engine *engine)
{
    int rv;
    struct sgdma_transfer *transfer = 0;
    int size = engine->sgdma_perf->transfer_size;

    /* transfers on queue? */
    if (!list_empty(&engine->transfer_list)) {
        /* pick first transfer on the queue (was submitted to engine) */
        transfer = list_entry(engine->transfer_list.next,
                      struct sgdma_transfer, entry);
        if (!transfer) {
            pr_err("(engine=%s) has void transfer in queue.\n",
                   engine->name);
            return NULL;
        }
        rv = sgdma_engine_stop(engine);
        if (rv < 0) {
            pr_err("Failed to stop engine\n");
            return NULL;
        }
        engine->running = 0;

        if (transfer->cyclic) {
            if (engine->sgdma_perf)
                dbg_perf("Stopping perf transfer on %s\n",
                     engine->name);
            else
                dbg_perf("Stopping cyclic transfer on %s\n",
                     engine->name);
            /* free up the buffer allocated for perf run */
            if (engine->perf_buf_virt)
                dma_free_coherent(&engine->sgdev->pdev->dev, size,
                          engine->perf_buf_virt,
                          engine->perf_buf_bus);
            engine->perf_buf_virt = NULL;
            list_del(&transfer->entry);
        } else {
            dbg_sg("(engine=%p) running transfer is not cyclic\n",
                   engine);
        }
    } else {
        dbg_sg("(engine=%p) found not running transfer.\n", engine);
    }
    return transfer;
}
EXPORT_SYMBOL_GPL(engine_cyclic_stop);

static int engine_writeback_setup(struct anlogic_dma_engine *engine)
{
    u32 w;
    struct anlogic_dev *sgdev;
    struct sgdma_poll_wb *writeback;

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    sgdev = engine->sgdev;
    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    /*
     * better to allocate one page for the whole device during probe()
     * and set per-engine offsets here
     */
    writeback = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    writeback->completed_desc_count = 0;

    dbg_init("Setting writeback location to 0x%llx for engine %p",
         engine->poll_mode_bus, engine);
    w = cpu_to_le32(PCI_DMA_L(engine->poll_mode_bus));
    write_register(w, &engine->regs->poll_mode_wb_lo,
               (unsigned long)(&engine->regs->poll_mode_wb_lo) -
                   (unsigned long)(engine->regs));
    w = cpu_to_le32(PCI_DMA_H(engine->poll_mode_bus));
    write_register(w, &engine->regs->poll_mode_wb_hi,
               (unsigned long)(&engine->regs->poll_mode_wb_hi) -
                   (unsigned long)(engine->regs));

    return 0;
}

/* engine_create() - Create an SG DMA engine bookkeeping data structure
 *
 * An SG DMA engine consists of the resources for a single-direction transfer
 * queue; the SG DMA hardware, the software queue and interrupt handling.
 *
 * @dev Pointer to pci_dev
 * @offset byte address offset in BAR[sgdev->config_bar_idx] resource for the
 * SG DMA * controller registers.
 * @dir: DMA_TO/FROM_DEVICE
 * @streaming Whether the engine is attached to AXI ST (rather than MM)
 */
static int engine_init_regs(struct anlogic_dma_engine *engine)
{
    u32 reg_value;
    int rv = 0;

    write_register(SGDMA_CTRL_NON_INCR_ADDR, &engine->regs->control_w1c,
               (unsigned long)(&engine->regs->control_w1c) -
                   (unsigned long)(engine->regs));

    engine_alignments(engine);

    /* Configure error interrupts by default */
    reg_value = SGDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
    reg_value |= SGDMA_CTRL_IE_MAGIC_STOPPED;
    reg_value |= SGDMA_CTRL_IE_READ_ERROR;
    reg_value |= SGDMA_CTRL_IE_DESC_ERROR;

    /* if using polled mode, configure writeback address */
    if (poll_mode) {
        rv = engine_writeback_setup(engine);
        if (rv) {
            dbg_init("%s descr writeback setup failed.\n",
                 engine->name);
            goto fail_wb;
        }
        reg_value |= SGDMA_CTRL_IE_DESC_COMPLETED; /* XXX hwb */
    } else {
        /* enable the relevant completion interrupts */
        reg_value |= SGDMA_CTRL_IE_DESC_STOPPED;
        reg_value |= SGDMA_CTRL_IE_DESC_COMPLETED;
    }

    /* Apply engine configurations */
    write_register(reg_value, &engine->regs->interrupt_enable_mask,
               (unsigned long)(&engine->regs->interrupt_enable_mask) -
                   (unsigned long)(engine->regs));

    engine->interrupt_enable_mask_value = reg_value;

    /* only enable credit mode for AXI-ST C2H */
    if (enable_st_c2h_credit && engine->streaming &&
        engine->dir == DMA_FROM_DEVICE) {
        struct anlogic_dev *sgdev = engine->sgdev;
        u32 reg_value = (0x1 << engine->channel) << 16;
        struct sgdma_common_regs *reg =
            (struct sgdma_common_regs
                 *)(sgdev->bar[sgdev->config_bar_idx] +
                    (0x6 * TARGET_SPACING));
#if 1 /* XXX FIXME */
        write_register(reg_value, &reg->credit_mode_enable,
            (unsigned long)(&reg->credit_mode_enable) -
                   (unsigned long)(reg));
#else
        write_register(reg_value, &reg->credit_mode_enable_w1s,
            (unsigned long)(&reg->credit_mode_enable_w1s) -
                   (unsigned long)(reg));
#endif
    }

    return 0;

fail_wb:
    return rv;
}

static int engine_init(struct anlogic_dma_engine *engine, struct anlogic_dev *sgdev,
               int offset, enum dma_data_direction dir, int channel)
{
    int rv;
    u32 val;

    dbg_init("channel %d, offset 0x%x, dir %d.\n", channel, offset, dir);

    /* set magic */
    engine->magic = MAGIC_ENGINE;

    engine->channel = channel;

    /* engine interrupt request bit */
    engine->irq_bitmask = (1 << SGDMA_ENG_IRQ_NUM) - 1;
    engine->irq_bitmask <<= (sgdev->engines_num * SGDMA_ENG_IRQ_NUM);
    engine->bypass_offset = sgdev->engines_num * BYPASS_MODE_SPACING;

    /* parent */
    engine->sgdev = sgdev;
    /* register address */
    engine->regs = (sgdev->bar[sgdev->config_bar_idx] + offset);
    engine->sgdma_regs = sgdev->bar[sgdev->config_bar_idx] + offset +
                 SGDMA_OFFSET_FROM_CHANNEL;
    val = read_register(&engine->regs->identifier);
    if (val & 0x8000U)
        engine->streaming = 1;

    /* remember SG DMA direction */
    engine->dir = dir;
    sprintf(engine->name, "%d-%s%d-%s", sgdev->idx,
        (dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel,
        engine->streaming ? "ST" : "MM");

    /* initialize the deferred work for transfer completion */
    INIT_WORK(&engine->work, engine_service_work);

    if (enable_st_c2h_credit && engine->streaming &&
        engine->dir == DMA_FROM_DEVICE)
            engine->desc_max = SGDMA_ENGINE_CREDIT_XFER_MAX_DESC;
    else
            engine->desc_max = SGDMA_ENGINE_XFER_MAX_DESC;

    dbg_init("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
         (int)engine->irq_bitmask);

    if (dir == DMA_TO_DEVICE)
        sgdev->mask_irq_h2c |= engine->irq_bitmask;
    else
        sgdev->mask_irq_c2h |= engine->irq_bitmask;
    sgdev->engines_num++;

    rv = engine_alloc_resource(engine);
    if (rv)
        return rv;

    rv = engine_init_regs(engine);
    if (rv)
        return rv;

    if (poll_mode)
        sgdma_thread_add_work(engine);

    return 0;
}

static int probe_for_engine(struct anlogic_dev *sgdev, enum dma_data_direction dir,
                int channel)
{
    int offset = channel * CHANNEL_SPACING;
    u32 engine_id_expected;
    struct anlogic_dma_engine *engine;
    int rv;

    /* register offset for the engine */
    /* read channels at 0x0000, write channels at 0x1000,
     * channels at 0x100 interval
     */
    if (dir == DMA_TO_DEVICE) {
        engine_id_expected = SGDMA_ID_H2C;
        engine = &sgdev->engine_h2c[channel];
    } else {
        offset += H2C_CHANNEL_OFFSET;
        engine_id_expected = SGDMA_ID_C2H;
        engine = &sgdev->engine_c2h[channel];
    }

    dbg_init("found AXI %s %d engine, reg. off 0x%x.\n",
         dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset);

    /* allocate and initialize engine */
    rv = engine_init(engine, sgdev, offset, dir, channel);
    if (rv != 0) {
        pr_info("failed to create AXI %s %d engine.\n",
            dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel);
        return rv;
    }

    return 0;
}

static int engine_destroy(struct anlogic_dev *sgdev, struct anlogic_dma_engine *engine)
{
    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    dbg_sg("Shutting down engine %s%d", engine->name, engine->channel);

    engine_service_shutdown(engine);

    /* Disable interrupts to stop processing new events during shutdown */
    write_register(0x0, &engine->regs->interrupt_enable_mask,
               (unsigned long)(&engine->regs->interrupt_enable_mask) -
                   (unsigned long)(engine->regs));

    if (enable_st_c2h_credit && engine->streaming &&
        engine->dir == DMA_FROM_DEVICE) {
        u32 reg_value = (0x1 << engine->channel) << 16;
        struct sgdma_common_regs *reg =
            (struct sgdma_common_regs
                 *)(sgdev->bar[sgdev->config_bar_idx] +
                    (0x6 * TARGET_SPACING));

#if 0 /* XXX FIXME */
        write_register(reg_value, &reg->credit_mode_enable_w1c, 
               (unsigned long)(&reg->credit_mode_enable_w1c) -
                   (unsigned long)(reg));
#else
        reg_value = 0;

        write_register(reg_value, &reg->credit_mode_enable,
               (unsigned long)(&reg->credit_mode_enable) -
                   (unsigned long)(reg));
#endif
    }

    if (poll_mode)
        sgdma_thread_remove_work(engine);

    /* Release memory use for descriptor writebacks */
    engine_free_resource(engine);

    memset(engine, 0, sizeof(struct anlogic_dma_engine));
    /* Decrement the number of engines available */
    sgdev->engines_num--;

    return 0;
}

static void remove_engines(struct anlogic_dev *sgdev)
{
    struct anlogic_dma_engine *engine;
    int i;
    int rv;

    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return;
    }

    /* iterate over channels */
    for (i = 0; i < sgdev->h2c_channel_max; i++) {
        engine = &sgdev->engine_h2c[i];
        if (engine->magic == MAGIC_ENGINE) {
            dbg_sg("Remove %s, %d", engine->name, i);
            rv = engine_destroy(sgdev, engine);
            if (rv < 0)
                pr_err("Failed to destroy H2C engine %d\n", i);
            dbg_sg("%s, %d removed", engine->name, i);
        }
    }

    for (i = 0; i < sgdev->c2h_channel_max; i++) {
        engine = &sgdev->engine_c2h[i];
        if (engine->magic == MAGIC_ENGINE) {
            dbg_sg("Remove %s, %d", engine->name, i);
            rv = engine_destroy(sgdev, engine);
            if (rv < 0)
                pr_err("Failed to destroy C2H engine %d\n", i);
            dbg_sg("%s, %d removed", engine->name, i);
        }
    }
}

static int probe_engines(struct anlogic_dev *sgdev)
{
    int i;
    int rv = 0;

    if (!sgdev) {
        pr_err("Invalid sgdev\n");
        return -EINVAL;
    }

    /* iterate over channels */
    for (i = 0; i < sgdev->h2c_channel_max; i++) {
        rv = probe_for_engine(sgdev, DMA_TO_DEVICE, i);
        if (rv)
            break;
    }
    sgdev->h2c_channel_max = i;

    for (i = 0; i < sgdev->c2h_channel_max; i++) {
        rv = probe_for_engine(sgdev, DMA_FROM_DEVICE, i);
        if (rv)
            break;
    }
    sgdev->c2h_channel_max = i;

    return 0;
}

void *anlogic_device_open(const char *mname, struct pci_dev *pdev,int *user_max,int *h2c_channel_max, int *c2h_channel_max)
{
    struct anlogic_dev *sgdev = NULL;
    int rv = 0;

    pr_info("%s device %s, 0x%p.\n", mname, dev_name(&pdev->dev), pdev);
    
    /* allocate zeroed device book keeping structure */
    sgdev = alloc_dev_instance(pdev);
    if (!sgdev)
        return NULL;
    
    sgdev->mod_name = mname;
    sgdev->user_max = *user_max;
    sgdev->h2c_channel_max = *h2c_channel_max;
    sgdev->c2h_channel_max = *c2h_channel_max;

    pr_info("sgdev->h2c_channel_max%d.\n", sgdev->h2c_channel_max);
    pr_info("sgdev->c2h_channel_max%d.\n", sgdev->c2h_channel_max);

    sgdma_device_flag_set(sgdev, XDEV_FLAG_OFFLINE);

    if (sgdev->user_max == 0 || sgdev->user_max > MAX_USER_IRQ)
        sgdev->user_max = MAX_USER_IRQ;
    if (sgdev->h2c_channel_max == 0 || sgdev->h2c_channel_max > SGDMA_CHANNEL_NUM_MAX)
        sgdev->h2c_channel_max = SGDMA_CHANNEL_NUM_MAX;
    if (sgdev->c2h_channel_max == 0 || sgdev->c2h_channel_max > SGDMA_CHANNEL_NUM_MAX)
        sgdev->c2h_channel_max = SGDMA_CHANNEL_NUM_MAX;

    rv = al_dev_list_add(sgdev);
    if (rv < 0)
        goto free_dev;

    rv = pci_enable_device(pdev);
    if (rv) {
        dbg_init("pci_enable_device() failed, %d.\n", rv);
        goto err_enable;
    }
    
    /* keep INTx enabled */
    pci_check_intr_pend(pdev);

    /* enable relaxed ordering */
    pci_enable_capability(pdev, PCI_EXP_DEVCTL_RELAX_EN);

    /* enable extended tag */
    pci_enable_capability(pdev, PCI_EXP_DEVCTL_EXT_TAG);
 
#if 0
    /* force MRRS to be 512 */
    rv = pcie_set_readrq(pdev, 512);
    if (rv)
        pr_info("device %s, error set PCI_EXP_DEVCTL_READRQ: %d.\n",
            dev_name(&pdev->dev), rv);
#endif
       
    /* enable bus master capability */
    pci_set_master(pdev);
    
    rv = request_regions(sgdev, pdev);
    if (rv)
        goto err_regions;

    rv = map_bars(sgdev, pdev);
    if (rv)
        goto err_map;
    
    rv = set_dma_mask(pdev);
    if (rv)
        goto err_mask;

    channel_interrupts_disable(sgdev, ~0);
    //user_interrupts_disable(sgdev, ~0);
    read_interrupts(sgdev);

   rv = probe_engines(sgdev);
    if (rv)
        goto err_engines;

    rv = enable_msi_msix(sgdev, pdev);
    if (rv < 0)
        goto err_enable_msix;
    
    rv = irq_setup(sgdev, pdev);
    if (rv < 0)
        goto err_interrupts;

    if (!poll_mode)
        channel_interrupts_enable(sgdev, ~0);

    /* Flush writes */
    read_interrupts(sgdev);
    
    //store real parameter in xpdev
    *user_max = sgdev->user_max;
    *h2c_channel_max = sgdev->h2c_channel_max;
    *c2h_channel_max = sgdev->c2h_channel_max;
    
    sgdma_device_flag_clear(sgdev, XDEV_FLAG_OFFLINE);

    return (void *)sgdev;
err_interrupts:
    irq_teardown(sgdev); 
err_enable_msix:
    disable_msi_msix(sgdev, pdev);
    
err_engines:
    remove_engines(sgdev);
    
err_mask:
    unmap_bars(sgdev, pdev);
err_map:
    if (sgdev->got_regions)
        pci_release_regions(pdev);
err_regions:
    if (!sgdev->regions_in_use)
        pci_disable_device(pdev);
err_enable:
    al_dev_list_remove(sgdev);
free_dev:
    kfree(sgdev);

    return NULL;
}

EXPORT_SYMBOL_GPL(anlogic_device_open);

void anlogic_device_close(struct pci_dev *pdev, void *dev_hndl)
{
    struct anlogic_dev *sgdev = (struct anlogic_dev *)dev_hndl;

    dbg_init("pdev 0x%p, sgdev 0x%p.\n", pdev, dev_hndl);

    if (!dev_hndl)
        return;

    dbg_sg("remove(dev = 0x%p) where pdev->dev.driver_data = 0x%p\n", pdev,
           sgdev);
    if (sgdev->pdev != pdev) {
        dbg_sg("pci_dev(0x%lx) != pdev(0x%lx)\n",
               (unsigned long)sgdev->pdev, (unsigned long)pdev);
    }

    channel_interrupts_disable(sgdev, ~0);
    //user_interrupts_disable(sgdev, ~0);
    read_interrupts(sgdev);

    irq_teardown(sgdev);
    dbg_init("before disable_msi_msix.\n");

    disable_msi_msix(sgdev, pdev);
    dbg_init("before unmap_bars.\n");
    remove_engines(sgdev);

    unmap_bars(sgdev, pdev);

    remove_engines(sgdev);

    if (sgdev->got_regions) {
        dbg_init("pci_release_regions 0x%p.\n", pdev);
        pci_release_regions(pdev);
    }

    if (!sgdev->regions_in_use) {
        dbg_init("pci_disable_device 0x%p.\n", pdev);
        pci_disable_device(pdev);
    }

    al_dev_list_remove(sgdev);

    dbg_init("before kfree.\n");

    kfree(sgdev);
}
EXPORT_SYMBOL_GPL(anlogic_device_close);

