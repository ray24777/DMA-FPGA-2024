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
#ifndef __ANLOGIC_DBG_H__
#define __ANLOGIC_DBG_H__

#include <linux/hwmon-sysfs.h>

#define AL_DBG_ARGC_MAX     20

#define AL_DBG_ATTR_DEF(_attr) \
    static struct al_dbg_attr _attr = { \
        .name = # _attr, \
    }

#define AL_DBG_NODE_CREATE(_attr, _dbg_main, _priv) \
    ({\
        (_attr)->dbg_main = _dbg_main; \
        (_attr)->priv = _priv; \
        al_dbg_node_create(_attr); \
    })

#define AL_DBG_NODE_DESTORY(_attr) \
    al_dbg_node_destory(_attr)

#define al_dbg_print_buf(buf, size) al_dbg_write(0, buf, size)

/**
 * struct al_dbg_attr - driver's dbg node attr:
 *
 * @dev: dbg related device;
 * @priv: dbg resv;
 * @hwmon: hwmon device;
 * @dbg_attr: sysfs attributes array;
 * @dbg_dev_attr: sysfs sensor device attribute array;
 * @group: sysfs attribute group;
 * @groups: list of sysfs attribute group for hwmon registration;
 * @name: dbg node name;
 * @dbg_main: size of a register value;
 */
struct al_dbg_attr {
    struct device *dev;
    void *priv;
    struct device *hwmon;
    struct attribute *dbg_attr[2];
    struct sensor_device_attribute dbg_dev_attr;
    struct attribute_group group;
    const struct attribute_group *groups[2];
    const char name[32];
    int (*dbg_main)(int argc, char *argv[]);
};

int al_dbg_node_create(struct al_dbg_attr *node);

void al_dbg_node_destory(struct al_dbg_attr *node);

int al_dbg_printk(const char *fmt, ...);

long al_dbg_write(unsigned int fd, const char __user *buf, size_t count);

#endif /* ifndef __ANLOGIC_DBG_H__ */
