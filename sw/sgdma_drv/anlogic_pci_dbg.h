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
#ifndef __ANLOGIC_PCI_DBG_H__
#define __ANLOGIC_PCI_DBG_H__

int al_pci_drv_dbg_init(struct device *dev);

void al_pci_drv_dbg_exit(void);

void dump_desc(struct sgdma_desc *desc_virt);

void transfer_dump(struct sgdma_transfer *transfer);

void sgt_dump(struct sg_table *sgt);

void sgdma_request_cb_dump(struct sgdma_request_cb *req);

#endif /* ifndef __ANLOGIC_PCI_DBG_H__ */
