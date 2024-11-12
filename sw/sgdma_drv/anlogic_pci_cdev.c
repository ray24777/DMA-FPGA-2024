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
#include "anlogic_pci_lib.h"
#include "anlogic_pci_cdev.h"

static struct class *g_anlogic_class;

struct kmem_cache *cdev_cache;

enum cdev_type {
    CHAR_USER,
    CHAR_CTRL,
    CHAR_XVC,
    CHAR_EVENTS,
    CHAR_SGDMA_H2C,
    CHAR_SGDMA_C2H,
    CHAR_BYPASS_H2C,
    CHAR_BYPASS_C2H,
    CHAR_BYPASS,
};

static const char * const devnode_names[] = {
    ANLOGIC_NODE_NAME "%d_user",
    ANLOGIC_NODE_NAME "%d_control",
    ANLOGIC_NODE_NAME "%d_xvc",
    ANLOGIC_NODE_NAME "%d_events_%d",
    ANLOGIC_NODE_NAME "%d_h2c_%d",
    ANLOGIC_NODE_NAME "%d_c2h_%d",
    ANLOGIC_NODE_NAME "%d_bypass_h2c_%d",
    ANLOGIC_NODE_NAME "%d_bypass_c2h_%d",
    ANLOGIC_NODE_NAME "%d_bypass",
};

enum xpdev_flags_bits {
    XDF_CDEV_USER,
    XDF_CDEV_CTRL,
    XDF_CDEV_XVC,
    XDF_CDEV_EVENT,
    XDF_CDEV_SG,
    XDF_CDEV_BYPASS,
};

static inline void xpdev_flag_set(struct anlogic_pci_dev *xpdev,
                enum xpdev_flags_bits fbit)
{
    xpdev->flags |= 1 << fbit;
}

static inline void sgcdev_flag_clear(struct anlogic_pci_dev *xpdev,
                enum xpdev_flags_bits fbit)
{
    xpdev->flags &= ~(1 << fbit);
}

static inline int xpdev_flag_test(struct anlogic_pci_dev *xpdev,
                enum xpdev_flags_bits fbit)
{
    return xpdev->flags & (1 << fbit);
}

#ifdef __SGDMA_SYSFS__
ssize_t anlogic_dev_instance_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct anlogic_pci_dev *xpdev =
        (struct anlogic_pci_dev *)dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\t%d\nn",
            xpdev->major, xpdev->sgdev->idx);
}

static DEVICE_ATTR_RO(anlogic_dev_instance);
#endif

static int config_kobject(struct anlogic_cdev *sgcdev, enum cdev_type type)
{
    int rv = -EINVAL;
    struct anlogic_dev *sgdev = sgcdev->sgdev;
    struct anlogic_dma_engine *engine = sgcdev->engine;

    switch (type) {
    case CHAR_SGDMA_H2C:
    case CHAR_SGDMA_C2H:
    case CHAR_BYPASS_H2C:
    case CHAR_BYPASS_C2H:
        if (!engine) {
            pr_err("Invalid DMA engine\n");
            return rv;
        }

        rv = kobject_set_name(&sgcdev->cdev.kobj, devnode_names[type],
            sgdev->idx, engine->channel);

        break;
    case CHAR_BYPASS:
    case CHAR_USER:
    case CHAR_CTRL:
    case CHAR_XVC:
        rv = kobject_set_name(&sgcdev->cdev.kobj, devnode_names[type],
            sgdev->idx);
        break;
    case CHAR_EVENTS:
        rv = kobject_set_name(&sgcdev->cdev.kobj, devnode_names[type],
            sgdev->idx, sgcdev->bar);
        break;
    default:
        pr_warn("%s: UNKNOWN type 0x%x.\n", __func__, type);
        break;
    }

    if (rv)
        pr_err("%s: type 0x%x, failed %d.\n", __func__, type, rv);
    return rv;
}

int sgcdev_check(const char *fname, struct anlogic_cdev *sgcdev, bool check_engine)
{
    struct anlogic_dev *sgdev;

    if (!sgcdev || sgcdev->magic != MAGIC_CHAR) {
        pr_info("%s, sgcdev 0x%p, magic 0x%lx.\n",
            fname, sgcdev, sgcdev ? sgcdev->magic : 0xFFFFFFFF);
        return -EINVAL;
    }

    sgdev = sgcdev->sgdev;
    if (!sgdev || sgdev->magic != MAGIC_DEVICE) {
        pr_info("%s, sgdev 0x%p, magic 0x%lx.\n",
            fname, sgdev, sgdev ? sgdev->magic : 0xFFFFFFFF);
        return -EINVAL;
    }

    if (check_engine) {
        struct anlogic_dma_engine *engine = sgcdev->engine;

        if (!engine || engine->magic != MAGIC_ENGINE) {
            pr_info("%s, engine 0x%p, magic 0x%lx.\n", fname,
                engine, engine ? engine->magic : 0xFFFFFFFF);
            return -EINVAL;
        }
    }

    return 0;
}

int char_open(struct inode *inode, struct file *file)
{
    struct anlogic_cdev *sgcdev;

    /* pointer to containing structure of the character device inode */
    sgcdev = container_of(inode->i_cdev, struct anlogic_cdev, cdev);
    
    if (sgcdev->magic != MAGIC_CHAR) {
        pr_err("sgcdev 0x%p inode 0x%lx magic mismatch 0x%lx\n",
            sgcdev, inode->i_ino, sgcdev->magic);
        return -EINVAL;
    }
    /* create a reference to our char device in the opened file */
    file->private_data = sgcdev;

    return 0;
}

/*
 * Called when the device goes from used to unused.
 */
int char_close(struct inode *inode, struct file *file)
{
    struct anlogic_dev  *sgdev;
    struct anlogic_cdev *sgcdev = (struct anlogic_cdev *)file->private_data;

    if (!sgcdev) {
        pr_err("char device with inode 0x%lx sgcdev NULL\n",
            inode->i_ino);
        return -EINVAL;
    }

    if (sgcdev->magic != MAGIC_CHAR) {
        pr_err("sgcdev 0x%p magic mismatch 0x%lx\n",
                sgcdev, sgcdev->magic);
        return -EINVAL;
    }

    /* fetch device specific data stored earlier during open */
    sgdev = sgcdev->sgdev;
    if (!sgdev) {
        pr_err("char device with inode 0x%lx sgdev NULL\n",
            inode->i_ino);
        return -EINVAL;
    }

    if (sgdev->magic != MAGIC_DEVICE) {
        pr_err("sgdev 0x%p magic mismatch 0x%lx\n", sgdev, sgdev->magic);
        return -EINVAL;
    }

    return 0;
}

/* create_sgcdev() -- create a character device interface to data or control bus
 *
 * If at least one SG DMA engine is specified, the character device interface
 * is coupled to the SG DMA file operations which operate on the data bus. If
 * no engines are specified, the interface is coupled with the control bus.
 */

static int create_sys_device(struct anlogic_cdev *sgcdev, enum cdev_type type)
{
    struct anlogic_dev *sgdev = sgcdev->sgdev;
    struct anlogic_dma_engine *engine = sgcdev->engine;
    int last_param;

    if (type == CHAR_EVENTS)
        last_param = sgcdev->bar;
    else
        last_param = engine ? engine->channel : 0;

    sgcdev->sys_device = device_create(g_anlogic_class, &sgdev->pdev->dev,
        sgcdev->cdevno, NULL, devnode_names[type], sgdev->idx,
        last_param);

    if (!sgcdev->sys_device) {
        pr_err("device_create(%s) failed\n", devnode_names[type]);
        return -1;
    }

    return 0;
}

static int destroy_sgcdev(struct anlogic_cdev *cdev)
{
    if (!cdev) {
        pr_warn("cdev NULL.\n");
        return -EINVAL;
    }
    if (cdev->magic != MAGIC_CHAR) {
        pr_warn("cdev 0x%p magic mismatch 0x%lx\n", cdev, cdev->magic);
        return -EINVAL;
    }

    if (!cdev->sgdev) {
        pr_err("sgdev NULL\n");
        return -EINVAL;
    }

    if (!223) {
        pr_err("g_anlogic_class NULL\n");
        return -EINVAL;
    }

    if (!cdev->sys_device) {
        pr_err("cdev sys_device NULL\n");
        return -EINVAL;
    }

    if (cdev->sys_device)
        device_destroy(g_anlogic_class, cdev->cdevno);

    cdev_del(&cdev->cdev);

    return 0;
}

static int create_sgcdev(struct anlogic_pci_dev *xpdev, struct anlogic_cdev *sgcdev,
            int bar, struct anlogic_dma_engine *engine, enum cdev_type type)
{
    int rv;
    int minor;
    struct anlogic_dev *sgdev = xpdev->sgdev;
    dev_t dev;

    spin_lock_init(&sgcdev->lock);
    /* new instance? */
    if (!xpdev->major) {
        /* allocate a dynamically allocated char device node */
        rv = alloc_chrdev_region(&dev, ANLOGIC_MINOR_BASE, ANLOGIC_MINOR_COUNT, ANLOGIC_NODE_NAME);

        if (rv) {
            pr_err("unable to allocate cdev region %d.\n", rv);
            return rv;
        } else{
            printk("allocate cdev region OK\n");
        }
        xpdev->major = MAJOR(dev);
        minor = MINOR(dev);
        
        printk(" cdev region MAJOR: %d, MINOR:%d\n", xpdev->major, minor);
    }

    /*
     * do not register yet, create kobjects and name them,
     */
    sgcdev->magic = MAGIC_CHAR;
    sgcdev->cdev.owner = THIS_MODULE;
    sgcdev->xpdev = xpdev;
    sgcdev->sgdev = sgdev;
    sgcdev->engine = engine;
    sgcdev->bar = bar;
    sgcdev->base = 0;
    rv = config_kobject(sgcdev, type);
    if (rv < 0)
        return rv;

    switch (type) {
    case CHAR_USER:
        sgcdev->base = SGDMA_USER_BAR_BASE_OFFSET;
        minor = type;
        cdev_ctrl_init(sgcdev);
        break;
    case CHAR_CTRL:
        /* minor number is type index for non-SGDMA interfaces */
        minor = type;
        cdev_ctrl_init(sgcdev);
        break;
    /*
    case CHAR_XVC:
        // minor number is type index for non-SGDMA interfaces 
        minor = type;
        cdev_xvc_init(sgcdev);
        break;
    */
    case CHAR_SGDMA_H2C:
        minor = 32 + engine->channel;
        cdev_sgdma_init(sgcdev);
        break;
    case CHAR_SGDMA_C2H:
        minor = 36 + engine->channel;
        cdev_sgdma_init(sgcdev);
        break;
    case CHAR_EVENTS:
        minor = 10 + bar;
        cdev_event_init(sgcdev);
        break;
    /*
    case CHAR_BYPASS_H2C:
        minor = 64 + engine->channel;
        cdev_bypass_init(sgcdev);
        break;
    case CHAR_BYPASS_C2H:
        minor = 68 + engine->channel;
        cdev_bypass_init(sgcdev);
        break;
    case CHAR_BYPASS:
        minor = 100;
        cdev_bypass_init(sgcdev);
        break;
    */
    default:
        pr_info("type 0x%x NOT supported.\n", type);
        return -EINVAL;
    }
    sgcdev->cdevno = MKDEV(xpdev->major, minor);

    /* bring character device live */
    
    rv = cdev_add(&sgcdev->cdev, sgcdev->cdevno, 1);
    if (rv < 0) {
        pr_err("cdev_add failed %d, type 0x%x.\n", rv, type);
        goto unregister_region;
    }

    dbg_init("sgcdev 0x%p, %u:%u, %s, type 0x%x.\n",
        sgcdev, xpdev->major, minor, sgcdev->cdev.kobj.name, type);

    /* create device on our class */
    if (g_anlogic_class) {
        rv = create_sys_device(sgcdev, type);
        if (rv < 0)
            goto del_cdev;
    }

    return 0;

del_cdev:
    cdev_del(&sgcdev->cdev);
unregister_region:
    unregister_chrdev_region(sgcdev->cdevno, ANLOGIC_MINOR_COUNT);
    return rv;
}

void xpdev_destroy_interfaces(struct anlogic_pci_dev *xpdev)
{
    int i = 0;
    int rv;
#ifdef __SGDMA_SYSFS__
    device_remove_file(&xpdev->pdev->dev, &dev_attr_sgdma_dev_instance);
#endif

    if (xpdev_flag_test(xpdev, XDF_CDEV_SG)) {
        // iterate over channels 
        for (i = 0; i < xpdev->h2c_channel_max; i++) {
            // remove SG DMA character device 
            rv = destroy_sgcdev(&xpdev->sgdma_h2c_cdev[i]);
            if (rv < 0)
                pr_err("Failed to destroy h2c sgcdev %d error :0x%x\n",
                        i, rv);
        }
        for (i = 0; i < xpdev->c2h_channel_max; i++) {
            rv = destroy_sgcdev(&xpdev->sgdma_c2h_cdev[i]);
            if (rv < 0)
                pr_err("Failed to destroy c2h sgcdev %d error 0x%x\n",
                        i, rv);
        }
    }

    if (xpdev_flag_test(xpdev, XDF_CDEV_EVENT)) {
        for (i = 0; i < xpdev->user_max; i++) {
            rv = destroy_sgcdev(&xpdev->events_cdev[i]);
            if (rv < 0)
                pr_err("Failed to destroy cdev event %d error 0x%x\n",
                    i, rv);
        }
    }

    /* remove control character device */
    if (xpdev_flag_test(xpdev, XDF_CDEV_CTRL)) {
        rv = destroy_sgcdev(&xpdev->ctrl_cdev);
        if (rv < 0)
            pr_err("Failed to destroy cdev ctrl event %d error 0x%x\n",
                i, rv);
    }

    /* remove user character device */
    if (xpdev_flag_test(xpdev, XDF_CDEV_USER)) {
        rv = destroy_sgcdev(&xpdev->user_cdev);
        if (rv < 0)
            pr_err("Failed to destroy user cdev %d error 0x%x\n",
                i, rv);
    }

    if (xpdev_flag_test(xpdev, XDF_CDEV_XVC)) {
        rv = destroy_sgcdev(&xpdev->xvc_cdev);
        if (rv < 0)
            pr_err("Failed to destroy xvc cdev %d error 0x%x\n",
                i, rv);
    }

    if (xpdev->major)
        unregister_chrdev_region(
                MKDEV(xpdev->major, ANLOGIC_MINOR_BASE),
                ANLOGIC_MINOR_COUNT);
}

int xpdev_create_interfaces(struct anlogic_pci_dev *xpdev)
{
    struct anlogic_dev *sgdev = xpdev->sgdev;
    struct anlogic_dma_engine *engine;
    int i;
    int rv = 0;

    /* initialize control character device */
    printk("xpdev_create_interfaces config bar:%d\n",sgdev->config_bar_idx);
    rv = create_sgcdev(xpdev, &xpdev->ctrl_cdev, sgdev->config_bar_idx, NULL, CHAR_CTRL);
    if (rv < 0) {
        pr_err("create_char(ctrl_cdev) failed\n");
        goto fail;
    }
    else
    {
        printk("xpdev_create_interfaces ctrl_cdev OK\n");
    }
    xpdev_flag_set(xpdev, XDF_CDEV_CTRL);

    /* iterate over channels */

    for (i = 0; i < xpdev->h2c_channel_max; i++) {
        engine = &sgdev->engine_h2c[i];

        if (engine->magic != MAGIC_ENGINE)
            continue;

        rv = create_sgcdev(xpdev, &xpdev->sgdma_h2c_cdev[i], i, engine,
                 CHAR_SGDMA_H2C);
        if (rv < 0) {
            pr_err("create char h2c %d failed, %d.\n", i, rv);
            goto fail;
        }
    }

    for (i = 0; i < xpdev->c2h_channel_max; i++) {
        engine = &sgdev->engine_c2h[i];

        if (engine->magic != MAGIC_ENGINE)
            continue;

        rv = create_sgcdev(xpdev, &xpdev->sgdma_c2h_cdev[i], i, engine,
                 CHAR_SGDMA_C2H);
        if (rv < 0) {
            pr_err("create char c2h %d failed, %d.\n", i, rv);
            goto fail;
        }
    }
    xpdev_flag_set(xpdev, XDF_CDEV_SG);

    /* initialize user character device */
    printk("xpdev_create_interfaces user bar:%d\n", sgdev->user_bar_idx);
    if (sgdev->user_bar_idx >= 0) {
        rv = create_sgcdev(xpdev, &xpdev->user_cdev, sgdev->user_bar_idx, NULL, CHAR_USER);
        if (rv < 0) {
            pr_err("create_char(user_cdev) failed\n");
            goto fail;
        } else {
            printk("xpdev_create_interfaces user_cdev OK\n");
        }
        xpdev_flag_set(xpdev, XDF_CDEV_USER);
    }

#ifdef __SGDMA_SYSFS__
    /* sys file */
    rv = device_create_file(&xpdev->pdev->dev,
                &dev_attr_sgdma_dev_instance);
    if (rv) {
        pr_err("Failed to create device file\n");
        goto fail;
    }
#endif

    return 0;

fail:
    rv = -1;
    xpdev_destroy_interfaces(xpdev);
    return rv;
}


int anlogic_cdev_init(void)
{
    g_anlogic_class = class_create(THIS_MODULE, ANLOGIC_NODE_NAME);
    if (IS_ERR(g_anlogic_class)) {
        dbg_init(ANLOGIC_NODE_NAME ": failed to create class");
        return -1;
    }

    /* using kmem_cache_create to enable sequential cleanup */
    cdev_cache = kmem_cache_create("cdev_cache",
                                   sizeof(struct cdev_async_io),
                                   0,
                                   SLAB_HWCACHE_ALIGN,
                                   NULL);
    if (!cdev_cache) {
        pr_info("memory allocation for cdev_cache failed. OOM\n");
        return -ENOMEM;
    }

    return 0;
}

void anlogic_cdev_cleanup(void)
{
    if (cdev_cache)
        kmem_cache_destroy(cdev_cache);

    if (g_anlogic_class)
        class_destroy(g_anlogic_class);
}
