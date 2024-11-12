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
#ifndef _ANLOGIC_IOCALLS_POSIX_H_
#define _ANLOGIC_IOCALLS_POSIX_H_

#include <linux/ioctl.h>

/* Use 'x' as magic number */
#define ANLOGIC_IOC_MAGIC   'x'
/* XL OpenCL X->58(ASCII), L->6C(ASCII), O->0 C->C L->6C(ASCII); */
#define ANLOGIC_XCL_MAGIC 0X586C0C6C

#ifndef VM_RESERVED
    #define VMEM_FLAGS (VM_IO | VM_DONTEXPAND | VM_DONTDUMP)
#else
    #define VMEM_FLAGS (VM_IO | VM_RESERVED)
#endif

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 *
 * _IO(type,nr)         no arguments
 * _IOR(type,nr,datatype)   read data from driver
 * _IOW(type,nr.datatype)   write data to driver
 * _IORW(type,nr,datatype)  read/write data
 *
 * _IOC_DIR(nr)         returns direction
 * _IOC_TYPE(nr)        returns magic
 * _IOC_NR(nr)          returns number
 * _IOC_SIZE(nr)        returns size
 */

enum ANLOGIC_IOC_TYPES {
    ANLOGIC_IOC_NOP,
    ANLOGIC_IOC_INFO,
    ANLOGIC_IOC_OFFLINE,
    ANLOGIC_IOC_ONLINE,
    ANLOGIC_IOC_MAX
};

struct anlogic_ioc_base {
    unsigned int magic;
    unsigned int command;
};

struct anlogic_ioc_info {
    struct anlogic_ioc_base base;
    unsigned short      vendor;
    unsigned short      device;
    unsigned short      subsystem_vendor;
    unsigned short      subsystem_device;
    unsigned int        dma_engine_version;
    unsigned int        driver_version;
    unsigned long long  feature_id;
    unsigned short      domain;
    unsigned char       bus;
    unsigned char       dev;
    unsigned char       func;
};

struct anlogic_ioc_bar {
    unsigned short    bar_id;
    unsigned short    bar_type;
    unsigned int      bar_offaddr;
    unsigned int      wdata;   // write data
    unsigned int      rdata;   //read data
};

#define MAX_DATA_SIZE   100

struct anlogic_ioc_bar_bulk {
    unsigned short    bar_id;
    unsigned short    bar_type;
    unsigned int      bar_offaddr;
    unsigned int      bar_len;
    unsigned int      wdata[MAX_DATA_SIZE];   // write data
    unsigned int      rdata[MAX_DATA_SIZE];   //read data
};

// Struct Used for Writing CFG Register.  Holds value and register to be written
typedef  struct cfgwrite {
	int reg;
	int value;
}cfgwr;

/* IOCTL codes */

#define ANLOGIC_IOCINFO         _IOWR(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_INFO, struct anlogic_ioc_info)
#define ANLOGIC_IOCOFFLINE      _IO(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_OFFLINE)
#define ANLOGIC_IOCONLINE       _IO(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_ONLINE)
/*
#define IOCTL_ANLOGIC_ADDRMODE_SET  _IOW('q', 4, int)
#define IOCTL_ANLOGIC_ADDRMODE_GET  _IOR('q', 5, int)
#define IOCTL_ANLOGIC_ALIGN_GET _IOR('q', 6, int)
*/

#define ANLOGIC_IOCW            _IOW(ANLOGIC_IOC_MAGIC,4,struct anlogic_ioc_bar)
#define ANLOGIC_IOCR            _IOR(ANLOGIC_IOC_MAGIC,5, struct anlogic_ioc_bar)
#define ANLOGIC_IOCBULKW         _IOW(ANLOGIC_IOC_MAGIC,6, struct anlogic_ioc_bar_bulk)
#define ANLOGIC_IOCBULKR         _IOR(ANLOGIC_IOC_MAGIC,7, struct anlogic_ioc_bar_bulk)

#define ANLOGIC_IOCCFGW            _IOW(ANLOGIC_IOC_MAGIC,8,struct cfgwrite)
#define ANLOGIC_IOCCFGR            _IOR(ANLOGIC_IOC_MAGIC,9, unsigned int)



#endif /* _ANLOGIC_IOCALLS_POSIX_H_ */
