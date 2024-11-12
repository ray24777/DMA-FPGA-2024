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
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <linux/interrupt.h>

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/aer.h>


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,2,0)

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#include <linux/pci-aspm.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,37)
#include <linux/prefetch.h>
#endif

#include "anlogic_pci_drv.h"
#include "anlogic_pci_lib.h"
#include "anlogic_pci_cdev.h"
#include "anlogic_pci_dbg.h"

MODULE_LICENSE("Dual BSD/GPL");

#define DRV_MODULE_NAME "Anlogic-pcie"

#define PCI_VENDOR_ID_ANLOGIC       0x1edb
#define PCI_DEVICE_ID_ANLOGIC       0xabcd

extern void anlogic_device_close(struct pci_dev *pdev, void *dev_hndl);
extern void *anlogic_device_open(const char *mname, struct pci_dev *pdev,int *user_max,int *h2c_channel_max, int *c2h_channel_max);

static const struct pci_device_id anlogic_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_ANLOGIC, PCI_DEVICE_ID_ANLOGIC), },
    {0,}
};

MODULE_DEVICE_TABLE(pci, anlogic_pci_tbl);

/* SECTION: Module global variables */
static int xpdev_cnt;

static void xpdev_free(struct anlogic_pci_dev *xpdev)
{
    struct anlogic_dev *sgdev = xpdev->sgdev;

    pr_info("xpdev 0x%p, destroy_interfaces, sgdev 0x%p.\n", xpdev, sgdev);
    xpdev_destroy_interfaces(xpdev);  //destroy all device
    xpdev->sgdev = NULL;
    pr_info("xpdev 0x%p, sgdev 0x%p sgdma_device_close.\n", xpdev, sgdev);
    anlogic_device_close(xpdev->pdev, sgdev);
    xpdev_cnt--;

    kfree(xpdev);
}

static struct anlogic_pci_dev *xpdev_alloc(struct pci_dev *pdev)
{
    struct anlogic_pci_dev *xpdev = kmalloc(sizeof(*xpdev), GFP_KERNEL);

    if (!xpdev)
        return NULL;
    memset(xpdev, 0, sizeof(*xpdev));
    
    // store some parameter in xpdev
    xpdev->magic = MAGIC_DEVICE;
    xpdev->pdev = pdev;
    xpdev->user_max = MAX_USER_IRQ;
    xpdev->h2c_channel_max = SGDMA_CHANNEL_NUM_MAX;
    xpdev->c2h_channel_max = SGDMA_CHANNEL_NUM_MAX;

    xpdev_cnt++;
    return xpdev;
}

static int anlogic_pci_probe(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
    void __iomem *bar0_base;
    u16 pci_vendor_id = 0;
    u16 pci_device_id = 0;
    int rv = 0;
    struct anlogic_pci_dev *xpdev = NULL;
    void *hndl;

    xpdev = xpdev_alloc(pdev);
    if (!xpdev)
        return -ENOMEM;
    pr_info("xpdev->h2c_channel_max%d.\n",xpdev->h2c_channel_max);
    pr_info("xpdev->c2h_channel_max%d.\n", xpdev->c2h_channel_max);

    hndl = anlogic_device_open(DRV_MODULE_NAME, pdev, &xpdev->user_max,
            &xpdev->h2c_channel_max, &xpdev->c2h_channel_max);
    if (!hndl) {
        rv = -EINVAL;
        goto err_out;
    }
    xpdev->sgdev = hndl;

    rv = xpdev_create_interfaces(xpdev);  //create all device
    if (rv)
        goto err_out;
    
    dev_set_drvdata(&pdev->dev, xpdev);

    rv = al_pci_drv_dbg_init(&pdev->dev);
    if (rv < 0)
        pr_info("anlogic driver dbg init falied.\n");

    return 0;
    
err_out:

    pr_err("pdev 0x%p, err %d.\n", pdev, rv);
    al_pci_drv_dbg_exit();
    xpdev_free(xpdev);

    return rv;
}

static void  anlogic_pci_remove(struct pci_dev *pdev)
{
  struct anlogic_pci_dev *xpdev;

    if (!pdev)
        return;

    xpdev = dev_get_drvdata(&pdev->dev);
    if (!xpdev)
        return;

    pr_info("pdev 0x%p, sgdev 0x%p, 0x%p.\n",
        pdev, xpdev, xpdev->sgdev);

    xpdev_free(xpdev);

    dev_set_drvdata(&pdev->dev, NULL);

    al_pci_drv_dbg_exit();
}

static pci_ers_result_t anlogic_error_detected(struct pci_dev *pdev,
                    pci_channel_state_t state)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    switch (state) {
    case pci_channel_io_normal:
        return PCI_ERS_RESULT_CAN_RECOVER;
    case pci_channel_io_frozen:
        pr_warn("dev 0x%p,0x%p, frozen state error, reset controller\n",
            pdev, xpdev);
            
        pci_disable_device(pdev);
        return PCI_ERS_RESULT_NEED_RESET;
    case pci_channel_io_perm_failure:
        pr_warn("dev 0x%p,0x%p, failure state error, req. disconnect\n",
            pdev, xpdev);
        return PCI_ERS_RESULT_DISCONNECT;
    }
    return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t anlogic_slot_reset(struct pci_dev *pdev)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("0x%p restart after slot reset\n", xpdev);
    if (pci_enable_device_mem(pdev)) {
        pr_info("0x%p failed to renable after slot reset\n", xpdev);
        return PCI_ERS_RESULT_DISCONNECT;
    }

    pci_set_master(pdev);
    pci_restore_state(pdev);
    pci_save_state(pdev);

    return PCI_ERS_RESULT_RECOVERED;
}

static void anlogic_error_resume(struct pci_dev *pdev)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 16, 0) /* XXX FIX ME */
    pci_cleanup_aer_uncorrect_error_status(pdev);
#endif
}

#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
static void anlogic_reset_prepare(struct pci_dev *pdev)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);

}

static void anlogic_reset_done(struct pci_dev *pdev)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);

}

#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static void anlogic_reset_notify(struct pci_dev *pdev, bool prepare)
{
    struct anlogic_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p, prepare %d.\n", pdev, xpdev, prepare);

}
#endif

static const struct pci_error_handlers anlogic_err_handler = {
    .error_detected = anlogic_error_detected,
    .slot_reset = anlogic_slot_reset,
    .resume     = anlogic_error_resume,
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
    .reset_prepare  = anlogic_reset_prepare,
    .reset_done = anlogic_reset_done,
#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    .reset_notify   = anlogic_reset_notify,
#endif
};

static struct pci_driver anlogic_pci_driver = {
    .name       = DRV_MODULE_NAME,
    .id_table   = anlogic_pci_tbl,
    .probe      = anlogic_pci_probe,
    .remove     = anlogic_pci_remove,
    .err_handler = &anlogic_err_handler,
};

static __init int anlogic_pci_init_module(void)
{
    int rv;
    printk("Hello,Anlogic-pcie\n");

    rv = anlogic_cdev_init(); //create class
    if (rv < 0)
        return rv;

    if (pci_register_driver(&anlogic_pci_driver)) {
        printk("no found pcie\n");
        return -ENODEV;
    }

    return 0;
}

static __exit void anlogic_pci_cleanup_module(void)
{
    pci_unregister_driver (&anlogic_pci_driver);

    anlogic_cdev_cleanup();

    printk("Goodbye,Anlogic-pcie\n");
}

module_init(anlogic_pci_init_module);
module_exit(anlogic_pci_cleanup_module);

