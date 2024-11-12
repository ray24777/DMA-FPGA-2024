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
#ifndef __ANLOGIC_CHRDEV_H__
#define __ANLOGIC_CHRDEV_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include "anlogic_pci_drv.h"

#define ANLOGIC_NODE_NAME	"ANLOGIC-PCI"
#define ANLOGIC_MINOR_BASE (0)
#define ANLOGIC_MINOR_COUNT (255)

void anlogic_cdev_cleanup(void);
int  anlogic_cdev_init(void);

int char_open(struct inode *inode, struct file *file);
int char_close(struct inode *inode, struct file *file);
int sgcdev_check(const char *fname, struct anlogic_cdev *sgcdev, bool check_engine);
void cdev_ctrl_init(struct anlogic_cdev *sgcdev);
//void cdev_xvc_init(struct anlogic_cdev *sgcdev);
void cdev_event_init(struct anlogic_cdev *sgcdev);
void cdev_sgdma_init(struct anlogic_cdev *sgcdev);
//void cdev_bypass_init(struct anlogic_cdev *sgcdev);
long char_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

void xpdev_destroy_interfaces(struct anlogic_pci_dev *xpdev);
int xpdev_create_interfaces(struct anlogic_pci_dev *xpdev);

int bridge_mmap(struct file *file, struct vm_area_struct *vma);

#endif /* __SGDMA_CHRDEV_H__ */
