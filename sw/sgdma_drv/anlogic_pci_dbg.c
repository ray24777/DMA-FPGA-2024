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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#define OPTPARSE_IMPLEMENTATION
#define OPTPARSE_API static
#include "optparse.h"

#include "anlogic_dbg.h"
#include "anlogic_pci_lib.h"
#include "anlogic_pci_drv.h"
#include "anlogic_pci_test.h"
#include "anlogic_ring.h"
#include "anlogic_thread.h"

#define AL_DBG_VERSION          "v1.0"
#define AL_DBG_COPYRIGHT_YEARS  "2022"
#define AL_DBG_NAME             "Anlogic Driver Debuger"

#define printf al_dbg_printk

#define optget_args(options, argc, argv, arg_max) \
    do { \
        char *last; \
        argv[argc++] = options->optarg; \
        while (argc < arg_max && (argv[argc++] = optparse_arg(options))); \
        argc--; \
        last = argv[argc - 1]; \
        last[strlen(last) - 1] = '\0'; \
    } while(0)

#define DBG_FORMART_INT_ARG(arg, val, base) \
    do { \
        if (kstrtoul(arg, base, &val) != 0) { \
            printf("Invalid arg %s\n", arg); \
            return; \
        } \
    } while (0)

#define AL_GET_DMA_ENGINE(engine, mode, ch) \
    do { \
        engine = al_pci_get_engine(mode, ch); \
        if (engine == NULL) { \
            printf("Engine %s-%d is not ready.\n", ((mode) == c2h) ? "C2H" : "H2C", ch); \
            return -ENXIO; \
        } \
    } while(0)

enum dma_mode {
    c2h = 0,
    h2c,
};

struct al_pci_dbg {
    struct optparse options;
    struct anlogic_dma_engine *engine;
    int mode;
    int ch;
};

AL_DBG_ATTR_DEF(al_dbger);

extern void __engine_status_dump(struct anlogic_dma_engine *engine, int (*printf)(const char *fmt, ...));

extern int engine_reg_dump(struct anlogic_dma_engine *engine);

extern void dump_desc(struct sgdma_desc *desc_virt);

extern void transfer_dump(struct sgdma_transfer *transfer);

extern void sgt_dump(struct sg_table *sgt);

extern void sgdma_request_cb_dump(struct sgdma_request_cb *req);

static void engine_transfer_list_dump(struct anlogic_dma_engine *engine)
{
    struct sgdma_transfer *transfer;
    unsigned long flags;

    if (engine == NULL) {
        printf("Invalid engine.\n");
        return;
    }

    spin_lock_irqsave(&engine->lock, flags);

    if (list_empty(&engine->transfer_list)) {
        printf("Engine %s current transfer list is empty.\n", engine->name);
        spin_unlock_irqrestore(&engine->lock, flags);
        return;
    }

    printf("Engine %s current transfer list:\n", engine->name);

    list_for_each_entry(transfer, &engine->transfer_list, entry) {
        printf("Tansfer (%p):\n", transfer);
        transfer_dump(transfer);
    }

    spin_unlock_irqrestore(&engine->lock, flags);
}

static void engine_info_dump(struct anlogic_dma_engine *engine)
{
    struct sgdma_transfer *transfer;
    unsigned long flags;

    if (engine == NULL) {
        printf("Invalid engine.\n");
        return;
    }

    printf("------------------------------------------\n");
    printf("name:                   %-20s\n", engine->name);
    printf("magic:                  0x%-20lx\n", engine->magic);
    printf("version:                0x%-20lx\n", engine->version);
    printf("channel:                %-20d\n", engine->channel);
    printf("dir:                    %-20s\n", (engine->dir == DMA_FROM_DEVICE) ? "C2H" : "H2C");
    printf("running:                %-20s\n", (engine->running == 1) ? "running" : "stop");
    printf("streaming:              %-20s\n", (engine->streaming == 1) ? "true" : "false");
    printf("addr_align:             %-20d\n", engine->addr_align);
    printf("desc_dequeued:          %-20d\n", engine->desc_dequeued);
    printf("desc_max:               %-20d\n", engine->desc_max);
    printf("eop_flush:              %-20d\n", engine->eop_flush);
    printf("cyclic_result:          0x%-20pK\n", engine->cyclic_result);
    printf("cyclic_result_bus:      0x%-20lx\n", engine->cyclic_result_bus);
    printf("cyclic_req:             0x%-20pK\n", engine->cyclic_req);
    printf("rx_tail:                %-20d\n", engine->rx_tail);
    printf("rx_head:                %-20d\n", engine->rx_head);
    printf("rx_overrun:             %-20d\n", engine->rx_overrun);
    printf("poll_mode_addr_virt:    0x%-20pK\n", engine->poll_mode_addr_virt);
    printf("poll_mode_bus:          0x%-20lx\n", engine->poll_mode_bus);
    printf("desc:                   0x%-20pK\n", engine->desc);
    printf("desc_bus:               0x%-20lx\n", engine->desc_bus);
    printf("desc_idx:               %-20d\n", engine->desc_idx);
    printf("desc_used:              %-20d\n", engine->desc_used);
    printf("cmplthp:                0x%-20pK\n", engine->cmplthp);
    printf("cmplthp name:           %-20s\n", engine->cmplthp ? engine->cmplthp->name : "null");
    printf("------------------------------------------\n");
}

static struct anlogic_dma_engine *al_pci_get_engine(int mode, int ch)
{
    struct anlogic_dev *sgdev = ((struct anlogic_pci_dev *)dev_get_drvdata(al_dbger.dev)) -> sgdev;

    if (ch >= SGDMA_CHANNEL_NUM_MAX || ch < 0) {
        return NULL;
    }

    return (mode == c2h)? &sgdev->engine_c2h[ch] : &sgdev->engine_h2c[ch];
}

static const struct optparse_long longopts[] = {
    /* Display usage help */
    { "help", 'h', OPTPARSE_NONE },

    /* Engine related options */
    { "dump",    'd', OPTPARSE_REQUIRED },
    { "mode",    'm', OPTPARSE_REQUIRED },
    { "channel", 'c', OPTPARSE_REQUIRED },
    { "run",     'r', OPTPARSE_NONE },
    { "stop",    's', OPTPARSE_NONE },

    /* Transfer related options */
    { "build", 'b', OPTPARSE_REQUIRED },

    /* Test case related options */
    { "test", 't', OPTPARSE_REQUIRED },

    {0},
};

static const char *option_help[] = {
    ['h'] = "Display this help",
    ['m'] = "Set dma engine mode, 0 - c2h, 1 - h2c",
    ['c'] = "Set dma engine channel, 0 ~ 3",
    ['r'] = "Start dma engine transfer.",
    ['s'] = "Stop dma engine transfer.",
    ['b'] = "Build a new transfer.\n\t\t"
            "Example: echo -b [ring | list] [addr] > /sys/class/hwmon/hwmon?/al_dbger",
    ['t'] = "Exec a test case.\n\t\t"
            "Example: echo -t [list] / [index] / [all] > /sys/class/hwmon/hwmon?/al_dbger",
    ['d'] = "Dump dma engine info.\n\t\t"
            "Example: echo -d [engine | reg | sgreg | tl | desc | rcb | sgt] [addr] > /sys/class/hwmon/hwmon?/al_dbger",
};

static void al_pci_dbg_help(char *app_name)
{
    int i;

    printf("%s %s (c) %s, "
           "Anlogic Bsp\n", app_name,
           AL_DBG_VERSION, AL_DBG_COPYRIGHT_YEARS);
    printf("\n"
           "Usage: %s [options] <filename>\n"
           "\n"
           "Options:\n\n", app_name ? : AL_DBG_NAME);
    for (i = 0; i < ARRAY_SIZE(longopts) - 1; i++) {
        printf("\t-%c, --%s%s\n" /* "\t-%c%s\n" */,
               longopts[i].shortname,
               longopts[i].longname,
               (longopts[i].argtype != OPTPARSE_NONE) ? " <argument>" : "");
        printf("\t\t%s.\n\n", option_help[longopts[i].shortname]);
    }
}

static void al_pci_dbg_setup_transfer(struct optparse *options)
{
    char *argv[AL_DBG_ARGC_MAX] = {NULL};
    int argc = 0, ret;
    struct anlogic_dma_engine *engine = 
        ((struct al_pci_dbg *)container_of(options, struct al_pci_dbg, options))->engine;

    optget_args(options, argc, argv, AL_DBG_ARGC_MAX);

    if (strcmp(argv[0], "ring") == 0) {
        ret = sgdma_cyclic_transfer_setup(engine);
        if (ret == 0) {
            printf("Engine %s setup rx ring transfer %p success.\n", engine->name, &engine->cyclic_req->tfer[0]);
        } else {
            printf("Engine %s setup rx ring transfer failed.\n", engine->name);
        }
        return;
    }
}

static void al_pci_dbg_dump(struct optparse *options)
{
    char *argv[AL_DBG_ARGC_MAX] = {NULL};
    int argc = 0;
    unsigned long addr = 0;
    struct anlogic_dma_engine *engine = 
        ((struct al_pci_dbg *)container_of(options, struct al_pci_dbg, options))->engine;

    optget_args(options, argc, argv, AL_DBG_ARGC_MAX);

    if (strcmp(argv[0], "status") == 0) {
        __engine_status_dump(engine, al_dbg_printk);
        return;
    }

    if (strcmp(argv[0], "tl") == 0) {
        engine_transfer_list_dump(engine);
        return;
    }

    if (strcmp(argv[0], "transfer") == 0) {
        DBG_FORMART_INT_ARG(argv[1], addr, 16);
        transfer_dump((struct sgdma_transfer *)addr);
        return;
    }

    if (strcmp(argv[0], "desc") == 0) {
        DBG_FORMART_INT_ARG(argv[1], addr, 16);
        dump_desc((struct sgdma_desc *)addr);
        return;
    }

    if (strcmp(argv[0], "rcb") == 0) {
        DBG_FORMART_INT_ARG(argv[1], addr, 16);
        sgdma_request_cb_dump((struct sgdma_request_cb *)addr);
        return;
    }

    if (strcmp(argv[0], "sgt") == 0) {
        DBG_FORMART_INT_ARG(argv[1], addr, 16);
        sgt_dump((struct sg_table *)addr);
        return;
    }

    if (strcmp(argv[0], "reg") == 0) {
        engine_reg_dump(engine);
        return;
    }

    if (strcmp(argv[0], "sgreg") == 0) {
        __engine_status_dump(engine, al_dbg_printk);
        return;
    }

    if (strcmp(argv[0], "engine") == 0) {
        engine_info_dump(engine);
        return;
    }
}

static void al_pci_dbg_test(struct optparse *options)
{
    char *argv[AL_DBG_ARGC_MAX] = {NULL};
    int argc = 0;
    unsigned long idx = 0;

    optget_args(options, argc, argv, AL_DBG_ARGC_MAX);

    if (strcmp(argv[0], "list") == 0) {
        al_pci_test_case_show();
        return;
    }

    if (strcmp(argv[0], "all") == 0) {
        al_pci_test_exec_all();
        return;
    }

    /* test a case */
    DBG_FORMART_INT_ARG(argv[0], idx, 10);

    al_pci_test_exec(idx);

    return;
}

static int al_pci_dbg_main(int argc, char *argv[])
{
    int option, ret;
    static struct al_pci_dbg pci_dbg = {
        .mode = 0,
        .ch = 0,
        .engine = NULL,
    };

    /* init a start engine. */
    AL_GET_DMA_ENGINE(pci_dbg.engine, pci_dbg.mode, pci_dbg.ch);

    optparse_init(&pci_dbg.options, argv);
    while ((option = optparse_long(&pci_dbg.options, longopts, NULL)) != -1) {
        switch (option) {
        case 'm':
            pci_dbg.mode = pci_dbg.options.optarg ? simple_strtol(pci_dbg.options.optarg, NULL, 10) : 1;
            AL_GET_DMA_ENGINE(pci_dbg.engine, pci_dbg.mode, pci_dbg.ch);
            break;
        case 'c':
            pci_dbg.ch = pci_dbg.options.optarg ? simple_strtol(pci_dbg.options.optarg, NULL, 10) : 1;
            AL_GET_DMA_ENGINE(pci_dbg.engine, pci_dbg.mode, pci_dbg.ch);
            break;
        case 'r':
            if (NULL == engine_start(pci_dbg.engine)) {
                printf("engine start failed\n");
            }
            break;
        case 's':
            ret = engine_service_shutdown(pci_dbg.engine);
            if (ret < 0) {
                printf("engine stop failed with %d\n", ret);
            }
            break;
        case 'd':
            al_pci_dbg_dump(&pci_dbg.options);
            break;
        case 'b':
            al_pci_dbg_setup_transfer(&pci_dbg.options);
            break;
        case 't':
            al_pci_dbg_test(&pci_dbg.options);
            break;
        case 'h': // help
            al_pci_dbg_help(argv[0]);
            break;
        }
    }

    return ret;
}

int al_pci_drv_dbg_init(struct device *dev)
{
    al_dbger.dev = dev;
    
    al_pci_test_main(al_dbger.dev);

    return AL_DBG_NODE_CREATE(&al_dbger, al_pci_dbg_main, NULL);
}

void al_pci_drv_dbg_exit(void)
{
    al_dbger.dev = NULL;

    AL_DBG_NODE_DESTORY(&al_dbger);
}

