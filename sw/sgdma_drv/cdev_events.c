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
#include "anlogic_pci_drv.h"

#include "anlogic_pci_cdev.h"

/*
 * character device file operations for events
 */
static ssize_t char_events_read(struct file *file, char __user *buf,
        size_t count, loff_t *pos)
{
    int rv;
    struct anlogic_user_irq *user_irq;
    struct anlogic_cdev *sgcdev = (struct anlogic_cdev *)file->private_data;
    u32 events_user;
    unsigned long flags;

    rv = sgcdev_check(__func__, sgcdev, 0);
    if (rv < 0)
        return rv;
    user_irq = sgcdev->user_irq;
    if (!user_irq) {
        pr_info("sgcdev 0x%p, user_irq NULL.\n", sgcdev);
        return -EINVAL;
    }

    if (count != 4)
        return -EPROTO;

    if (*pos & 3)
        return -EPROTO;

    /*
     * sleep until any interrupt events have occurred,
     * or a signal arrived
     */
    rv = wait_event_interruptible(user_irq->events_wq,
            user_irq->events_irq != 0);
    if (rv)
        dbg_sg("wait_event_interruptible=%d\n", rv);

    /* wait_event_interruptible() was interrupted by a signal */
    if (rv == -ERESTARTSYS)
        return -ERESTARTSYS;

    /* atomically decide which events are passed to the user */
    spin_lock_irqsave(&user_irq->events_lock, flags);
    events_user = user_irq->events_irq;
    user_irq->events_irq = 0;
    spin_unlock_irqrestore(&user_irq->events_lock, flags);

    rv = copy_to_user(buf, &events_user, 4);
    if (rv)
        dbg_sg("Copy to user failed but continuing\n");

    return 4;
}

static unsigned int char_events_poll(struct file *file, poll_table *wait)
{
    struct anlogic_user_irq *user_irq;
    struct anlogic_cdev *sgcdev = (struct anlogic_cdev *)file->private_data;
    unsigned long flags;
    unsigned int mask = 0;
    int rv;

    rv = sgcdev_check(__func__, sgcdev, 0);
    if (rv < 0)
        return rv;
    user_irq = sgcdev->user_irq;
    if (!user_irq) {
        pr_info("sgcdev 0x%p, user_irq NULL.\n", sgcdev);
        return -EINVAL;
    }

    poll_wait(file, &user_irq->events_wq,  wait);

    spin_lock_irqsave(&user_irq->events_lock, flags);
    if (user_irq->events_irq)
        mask = POLLIN | POLLRDNORM; /* readable */

    spin_unlock_irqrestore(&user_irq->events_lock, flags);

    return mask;
}

/*
 * character device file operations for the irq events
 */
static const struct file_operations events_fops = {
    .owner = THIS_MODULE,
    .open = char_open,
    .release = char_close,
    .read = char_events_read,
    .poll = char_events_poll,
};

void cdev_event_init(struct anlogic_cdev *sgcdev)
{
    sgcdev->user_irq = &(sgcdev->sgdev->user_irq[sgcdev->bar]);
    cdev_init(&sgcdev->cdev, &events_fops);
}
