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
#ifndef __ANLOGIC_VERSION_H__
#define __ANLOGIC_VERSION_H__

#define DRV_MOD_MAJOR		2020
#define DRV_MOD_MINOR		8
#define DRV_MOD_PATCHLEVEL	21

#define DRV_MODULE_VERSION      \
	__stringify(DRV_MOD_MAJOR) "." \
	__stringify(DRV_MOD_MINOR) "." \
	__stringify(DRV_MOD_PATCHLEVEL)

#define DRV_MOD_VERSION_NUMBER  \
	((DRV_MOD_MAJOR)*1000 + (DRV_MOD_MINOR)*100 + DRV_MOD_PATCHLEVEL)

#endif /* ifndef __SGDMA_VERSION_H__ */
