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

#ifndef __ANLOGIC_KTHREAD_H__
#define __ANLOGIC_KTHREAD_H__
/**
 * @file
 * @brief This file contains the declarations for sgdma kernel threads
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
#include "anlogic_pci_lib.h"

#ifdef DEBUG_THREADS
#define lock_thread(thp)    \
    do { \
        pr_debug("locking thp %s ...\n", (thp)->name); \
        spin_lock(&(thp)->lock); \
    } while (0)

#define unlock_thread(thp)  \
    do { \
        pr_debug("unlock thp %s ...\n", (thp)->name); \
        spin_unlock(&(thp)->lock); \
    } while (0)

#define sgdma_kthread_wakeup(thp)    \
    do { \
        pr_info("signaling thp %s ...\n", (thp)->name); \
        wake_up_process((thp)->task); \
    } while (0)

#define pr_debug_thread(fmt, ...) pr_info(fmt, __VA_ARGS__)

#else
/** lock thread macro */
#define lock_thread(thp)        spin_lock(&(thp)->lock)
/** un lock thread macro */
#define unlock_thread(thp)      spin_unlock(&(thp)->lock)
#define sgdma_kthread_wakeup(thp) \
    do { \
        thp->schedule = 1; \
        wake_up_interruptible(&thp->waitq); \
    } while (0)
/** pr_debug_thread */
#define pr_debug_thread(fmt, ...)
#endif

/**
 * @struct - sgdma_kthread
 * @brief   sgdma thread book keeping parameters
 */
struct sgdma_kthread {
    /**  thread lock*/
    spinlock_t lock;
    /**  name of the thread */
    char name[16];
    /**  cpu number for which the thread associated with */
    unsigned short cpu;
    /**  thread id */
    unsigned short id;
    /**  thread sleep timeout value */
    unsigned int timeout;
    /**  flags for thread */
    unsigned long flag;
    /**  thread wait queue */
    wait_queue_head_t waitq;
    /* flag to indicate scheduling of thread */
    unsigned int schedule;
    /**  kernel task structure associated with thread*/
    struct task_struct *task;
    /**  thread work list count */
    unsigned int work_cnt;
    /**  thread work list count */
    struct list_head work_list;
    /**  thread initialization handler */
    int (*finit)(struct sgdma_kthread *);
    /**  thread pending handler */
    int (*fpending)(struct list_head *);
    /**  thread peocessing handler */
    int (*fproc)(struct list_head *);
    /**  thread done handler */
    int (*fdone)(struct sgdma_kthread *);
};


/*****************************************************************************/
/**
 * sgdma_threads_create() - create sgdma threads
*********/
int sgdma_threads_create(unsigned int num_threads);

/*****************************************************************************/
/**
 * sgdma_threads_destroy() - destroy all the sgdma threads created
 *                          during system initialization
 *
 * @return  none
 *****************************************************************************/
void sgdma_threads_destroy(void);

/*****************************************************************************/
/**
 * sgdma_thread_remove_work() - handler to remove the attached work thread
 *
 * @param[in]   engine: pointer to sgdma_engine
 *
 * @return  none
 *****************************************************************************/
void sgdma_thread_remove_work(struct anlogic_dma_engine *engine);

/*****************************************************************************/
/**
 * sgdma_thread_add_work() - handler to add a work thread
 *
 * @param[in]   engine: pointer to sgdma_engine
 *
 * @return  none
 *****************************************************************************/
void sgdma_thread_add_work(struct anlogic_dma_engine *engine);

#endif /* #ifndef __SGDMA_KTHREAD_H__ */