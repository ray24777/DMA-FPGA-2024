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

#include "anlogic_dbg.h"
#include "anlogic_pci_lib.h"
#include "anlogic_pci_drv.h"
#include "anlogic_pci_dbg.h"
#include "anlogic_pci_test.h"
#include "anlogic_ring.h"

static struct anlogic_dma_engine *h2c_test_engine;

/**
 * test sgdma h2c pollMode
 */
H2C_TEST_CASE("sgdma h2c pollMode test", al_sgdma_h2c_test0)
{
#undef TEST_DATA_SIZE
#undef TEST_DESC_NUM
#undef TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE       0x100
#define TEST_DESC_NUM        5
#define TEST_DATA_DUMP_SIZE 0x200

    struct anlogic_dev *sgdev = h2c_test_engine->sgdev;
    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test0;
    dma_addr_t desc_bus, data_bus;
    size_t data_size;
    uint32_t control = 0, status, next_adj, w;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], err_info[256] = {0};
    bool ret = true;
    int i, rv;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 0);
    if (rv < 0) {
        ret = false;
        goto out1;
    }

    desc_virt = curr_test_case->desc_virt;
    data_virt = curr_test_case->data_virt;
    desc_bus = curr_test_case->desc_bus;
    data_bus = curr_test_case->data_bus;

    /* step2: run engine do transfer */
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);
    if (rv < 0) {
        ret = false;
    }

    /* step3: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "register comp desc cnt: %d\n", w);

    if (wb_data->completed_desc_count != TEST_DESC_NUM || w  != TEST_DESC_NUM) {
        ret = false;
    }

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * sgdma c2h stream random desc length
 */
H2C_TEST_CASE("sgdma h2c stream random desc length, desc with eop", al_sgdma_h2c_test1)
{
    return al_random_length_xfer(&al_sgdma_h2c_test1, h2c_test_engine, 1);
}

/**
 * test sgdma h2c stream random desc stop
 */
H2C_TEST_CASE("sgdma h2c stream random desc stop", al_sgdma_h2c_test2)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0x1000
#define TEST_DESC_NUM       500
#define TEST_DATA_DUMP_SIZE 0x80

    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct anlogic_dev *sgdev = engine->sgdev;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test2;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, extra_adj, w;
    void *data_virt;
    char prefix[64], desc_str[256], err_info[256] = {0};
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 0);
    if (rv < 0) {
        ret = false;
        goto out1;
    }

    desc_virt = curr_test_case->desc_virt;
    res_virt = curr_test_case->res_virt;
    desc_bus = curr_test_case->desc_bus;
    res_bus = curr_test_case->res_bus;

    /* step2: add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    write_register(cpu_to_le32(PCI_DMA_L(curr_test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_lo, 
            (unsigned long)(&engine->sgdma_regs->first_desc_lo) -
                (unsigned long)(engine->sgdma_regs));

    write_register(cpu_to_le32(PCI_DMA_H(curr_test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_hi, 
            (unsigned long)(&engine->sgdma_regs->first_desc_hi) -
                (unsigned long)(engine->sgdma_regs));

    extra_adj = count;

    if (extra_adj > 0) {
        extra_adj = extra_adj - 1;
        if (extra_adj > MAX_EXTRA_ADJ)
            extra_adj = MAX_EXTRA_ADJ;
    }

    write_register(cpu_to_le32(extra_adj),
            &engine->sgdma_regs->first_desc_adjacent, 
            (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
                (unsigned long)(engine->sgdma_regs));

    /* clear poll addr wb count */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    wb_data->completed_desc_count = 0;

    /* step4: start engine */
    engine_start_mode_config(engine);
    engine->running = 1;

    /* wait some times */
    mdelay(al_random_num_gen(1, 4));

    /* stop transfer */
    sgdma_engine_stop(engine);
    engine->running = 0;

    status = read_register(&engine->regs->status);
    TEST_DBG(curr_test_case, "engine %s status = 0x%08x\n", engine->name, status);

    /* check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    /* must keep minimal restart delay */
    mdelay(1);
    
    /* start a new ok transfer */
    ret = al_stream_transfer_test(curr_test_case);

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test sgdma h2c stream bad desc magic test
 */
H2C_TEST_CASE("sgdma h2c stream bad desc magic test", al_sgdma_h2c_test3)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       5
#define TEST_DATA_DUMP_SIZE 0x100

    struct anlogic_dev *sgdev = h2c_test_engine->sgdev;
    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test3;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, w, desc_wb;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], desc_str[256];
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 0);
    if (rv < 0) {
        ret = false;
        goto out1;
    }

    desc_virt = curr_test_case->desc_virt;
    data_virt = curr_test_case->data_virt;
    res_virt = curr_test_case->res_virt;
    desc_bus = curr_test_case->desc_bus;
    data_bus = curr_test_case->data_bus;
    res_bus = curr_test_case->res_bus;

    /* set bad control magic on desc[2] */
    desc_virt[2].control &= 0xFFFFUL;
    desc_virt[2].control |= 0xAD4C0000UL;

    /* add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    rv = al_pci_test_engine_do_transfer_with_no_stop(curr_test_case, engine, TEST_DESC_NUM, 0);
    if (rv < 0) {
        ret = false;
    }

    /* step4: check write back data */
    status = read_register(&engine->regs->status);
    TEST_DBG(curr_test_case, "engine %s status = 0x%08x\n", engine->name, status);

    /* stop transfer */
    sgdma_engine_stop(engine);
    engine->running = 0;

    if (status & SGDMA_STAT_MAGIC_STOPPED) {
        TEST_DBG(curr_test_case, "engine %s status get MAGIC_STOPPED bit\n", engine->name);
    } else {
        TEST_DBG(curr_test_case, "engine %s status not get MAGIC_STOPPED bit\n", engine->name);
        w = read_register(&engine->regs->control);
        TEST_DBG(curr_test_case, "engine %s control = 0x%08x, MAGIC_STOP flag is %s\n",
            engine->name, w, (w & SGDMA_CTRL_IE_MAGIC_STOPPED) ? "1" : "0");
        if (w & SGDMA_CTRL_IE_MAGIC_STOPPED) {
            TEST_RET(curr_test_case, false, "sw set bad magic control, but status not get err bit\n");
        }
    }

    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    w = wb_data->completed_desc_count;
    desc_wb = w & WB_COUNT_MASK;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d, raw: 0x%08x\n", desc_wb, w);

    if (w & WB_ERR_MASK) {
        TEST_DBG(curr_test_case, "hw write back get err bit\n");
    } else {
        TEST_DBG(curr_test_case, "hw write back not get err bit\n");
        w = read_register(&engine->regs->control);
        TEST_DBG(curr_test_case, "engine %s control = 0x%08x, MAGIC_STOP flag is %s\n",
            engine->name, w, (w & SGDMA_CTRL_IE_MAGIC_STOPPED) ? "1" : "0");
        if (w & SGDMA_CTRL_IE_MAGIC_STOPPED) {
            TEST_RET(curr_test_case, false, "sw set bad magic control, but not get write back err bit\n");
        }
    }

#if 0
    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (desc_wb < TEST_DESC_NUM) ? desc_wb : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }
#endif

    al_pci_test_free_desc(curr_test_case);

    /* step5: start a new ok transfer */
    ret = al_stream_transfer_test(curr_test_case);

out1:
    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test sgdma h2c credit mode
 */
H2C_TEST_CASE("sgdma h2c credit mode test", al_sgdma_h2c_test4)
{
#undef TEST_DATA_SIZE
#undef TEST_DESC_NUM
#define TEST_DATA_SIZE       0x100
#define TEST_DESC_NUM        20
#define TEST_CREDIT_MAX_NUM  7

    struct anlogic_dev *sgdev = h2c_test_engine->sgdev;
    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test4;
    struct sgdma_common_regs *reg;
    dma_addr_t desc_bus, data_bus;
    size_t data_size;
    uint32_t w;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], err_info[256] = {0};
    bool ret = true;
    int random_credit_num;
    int i, rv;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 0);
    if (rv < 0) {
        ret = false;
        goto out1;
    }

    desc_virt = curr_test_case->desc_virt;
    data_virt = curr_test_case->data_virt;
    desc_bus = curr_test_case->desc_bus;
    data_bus = curr_test_case->data_bus;

    /* step2: init credit mode */
    w = 0x1 << engine->channel; /* h2c: bit0 ~ 3 */
    reg = (struct sgdma_common_regs
             *)(sgdev->bar[sgdev->config_bar_idx] +
                (0x6 * TARGET_SPACING));

#if 0 /* XXX FIXME */
    write_register(w, &reg->credit_mode_enable,
        (unsigned long)(&reg->credit_mode_enable) -
               (unsigned long)(reg));
#else
    write_register(w, &reg->credit_mode_enable_w1s,
        (unsigned long)(&reg->credit_mode_enable_w1s) -
               (unsigned long)(reg));
#endif

    random_credit_num = al_random_num_gen(0, TEST_CREDIT_MAX_NUM);

    write_register(random_credit_num,
                &engine->sgdma_regs->credits, 
           (unsigned long)(&engine->sgdma_regs->credits) -
               (unsigned long)(engine->sgdma_regs));

    /* step3: run engine do transfer */
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);
    if (rv < 0) {
        ret = false;
    }

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    /* step5: check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "comp desc cnt: %d\n", w);

    if (wb_data->completed_desc_count != 0 || w  != random_credit_num) {
        ret = false;
        sprintf(err_info, "data write back count check failed, dst %d, real %d",
            random_credit_num, wb_data->completed_desc_count);
    }

    /* disable credit mode */
    w = 0x1 << engine->channel; /* h2c: bit0 ~ 3 */
    write_register(w, &reg->credit_mode_enable_w1c,
        (unsigned long)(&reg->credit_mode_enable_w1c) -
               (unsigned long)(reg));

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * sgdma h2c stream random desc length, desc without eop
 */
H2C_TEST_CASE("sgdma h2c stream random desc length, desc without eop", al_sgdma_h2c_test5)
{
    return al_random_length_xfer(&al_sgdma_h2c_test5, h2c_test_engine, 0);
}

/**
 * sgdma h2c stream random desc length, desc without eop
 */
H2C_TEST_CASE("sgdma h2c multiple tlp, desc length 64 bytes aligned, src addr byte aligned", al_sgdma_h2c_test6)
{
#undef TEST_DATA_SIZE
#undef TEST_DESC_NUM
#undef TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE       448
#define TEST_DESC_NUM        20
#define DATA_ALIGN_BYTES     64
#define DATA_ALIGN_MASK      (DATA_ALIGN_BYTES - 1)
#define SRC_ADDR_ALIGN_BYTES 0Xf
#define SRC_ADDR_ALIGN_MASK  (SRC_ADDR_ALIGN_BYTES - 1)

    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct anlogic_dev *sgdev = engine->sgdev;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test6;
    dma_addr_t desc_bus, data_bus;
    size_t data_size;
    struct sg_table test_sgt;
    struct scatterlist *sg;
    uint32_t control = 0, status, next_adj, w;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], err_info[256] = {0};
    bool ret = true;
    int i, rv, mod;
    int buf_size[TEST_DESC_NUM];
    int buf_offset_bytes[TEST_DESC_NUM];

    /* step1: alloc transfer desc */
    curr_test_case->desc_size = sizeof(struct sgdma_desc) * TEST_DESC_NUM;
    curr_test_case->desc_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->desc_size, &curr_test_case->desc_bus);
    if (curr_test_case->desc_virt == NULL) {
        TEST_DBG(curr_test_case, "pci desc alloc consistent failed.\n");
        ret = false;
        goto out1;
    }
    memset(curr_test_case->desc_virt, 0, curr_test_case->desc_size);

    /* gen random data length */
    for (i = 0; i < ARRAY_SIZE(buf_size); i++) {
        buf_size[i] = al_random_num_gen(400, PAGE_SIZE);
        buf_size[i] &= ~DATA_ALIGN_MASK;
        data_size += buf_size[i];
        TEST_DBG(curr_test_case, "buf_size[%d]: 0x%08x\n", i, buf_size[i]);
    }

    for (i = 0; i < ARRAY_SIZE(buf_offset_bytes); i++) {
        if (i > 9) {
            buf_offset_bytes[i] = al_random_num_gen(1, 9);
        } else {
            buf_offset_bytes[i] = i;
        }
        TEST_DBG(curr_test_case, "buf_offset_bytes[%d]: %d\n", i, buf_offset_bytes[i]);
    }

    al_pci_test_desc_link(curr_test_case->desc_virt, curr_test_case->desc_bus, TEST_DESC_NUM);

    /* build 0 ~ 9 bytes aligned src addr */
    rv = sgt_alloc_with_pages(&test_sgt, TEST_DESC_NUM, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_DESC_NUM, rv);
        ret = false;
        goto out2;
    }

    /* make page offset for src addr align */
    for (i = 0, sg = test_sgt.sgl; i < TEST_DESC_NUM; i++, sg = sg_next(sg)) {
        sgdma_desc_set(&curr_test_case->desc_virt[i],
            ((sg_dma_address(sg) + SRC_ADDR_ALIGN_BYTES) & ~SRC_ADDR_ALIGN_MASK) + buf_offset_bytes[i],
            0, buf_size[i], curr_test_case->dir);
        sgdma_desc_control_clear(&curr_test_case->desc_virt[i], LS_BYTE_MASK);
    }

    /* stop engine, EOP for AXI ST, req IRQ on last descriptor */
    control = SGDMA_DESC_STOPPED;
    control |= SGDMA_DESC_EOP;
    control |= SGDMA_DESC_COMPLETED;
    sgdma_desc_control_set(&curr_test_case->desc_virt[TEST_DESC_NUM - 1], control);

    al_pci_test_fill_adj_num(curr_test_case->desc_virt, TEST_DESC_NUM);

    /* step2: run engine do transfer */
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);
    if (rv < 0) {
        ret = false;
    }

    /* step3: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "register comp desc cnt: %d\n", w);

    if (wb_data->completed_desc_count != TEST_DESC_NUM || w != TEST_DESC_NUM) {
        ret = false;
    }

    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out2:
    pci_free_consistent(sgdev->pdev, curr_test_case->desc_size, curr_test_case->desc_virt, curr_test_case->desc_bus);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * sgdma h2c stream src addr cross 4k bound
 */
H2C_TEST_CASE("sgdma h2c src addr cross 4k bound", al_sgdma_h2c_test7)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DESC_NUM       (10)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_ALIGN_OFFSET   64
#define SIZE_4K             (0x1000UL)
#define BUF_4K_MASK         (SIZE_4K - 1)
#define TEST_DATA_SIZE      SIZE_4K

    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct anlogic_dev *sgdev = engine->sgdev;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test7;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, extra_adj, desc_align, w, align_offset;
    void *data_virt;
    char prefix[64], desc_str[256], err_info[256];
    bool ret = true;
    int i, rv, num, count;
    struct sg_table test_sgt;
    struct scatterlist *sg;

    /* step1: build desc list */

    /* alloc some data buf */
    count = TEST_DESC_NUM;
    curr_test_case->data_size = count * TEST_DATA_SIZE + TEST_ALIGN_OFFSET;

    curr_test_case->data_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->data_size, &curr_test_case->data_bus);
    if (curr_test_case->data_virt == NULL) {
        TEST_DBG(curr_test_case, "pci data alloc consistent failed.\n");
        ret = false;
        goto out1;
    }

    /* build cross 4k bound data buf */
    if ((curr_test_case->data_bus & BUF_4K_MASK) == 0) {
        TEST_DBG(curr_test_case, "data buf start addr(0x%llx) is 4k aligned, to make offset 0x%x\n",
            curr_test_case->data_bus, TEST_ALIGN_OFFSET);
        align_offset = TEST_ALIGN_OFFSET;
        curr_test_case->data_virt += align_offset;
        curr_test_case->data_bus += align_offset;
        curr_test_case->data_size -= align_offset;
    } else {
        TEST_DBG(curr_test_case, "data buf start addr(0x%llx) is not 4k aligned, offset 0x%llx\n",
            curr_test_case->data_bus, curr_test_case->data_bus & BUF_4K_MASK);
        align_offset = 0;
    }

    /* alloc some desc */
    rv = sgt_alloc_with_pages(&test_sgt, 2, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, 2, rv);
        ret = false;
        goto out2;
    }

    /* first page used to desc */
    sg = test_sgt.sgl;

    if (count > (sg_dma_len(sg) / sizeof(struct sgdma_desc))) {
        count = sg_dma_len(sg) / sizeof(struct sgdma_desc);
    }

    desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));

    curr_test_case->desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));
    curr_test_case->desc_bus = sg_dma_address(sg);

    /* 2th page used to desc res */
    sg = sg_next(sg);

    if (count > (sg_dma_len(sg) / sizeof(struct sgdma_result))) {
        count = sg_dma_len(sg) / sizeof(struct sgdma_result);
    }

    curr_test_case->res_virt = (struct sgdma_result *)page_address(sg_page(sg));
    curr_test_case->res_bus = sg_dma_address(sg);

    al_pci_test_build_desc_no_alloc(curr_test_case, count, TEST_DATA_SIZE, 0);

    desc_bus = curr_test_case->desc_bus;
    desc_virt = curr_test_case->desc_virt;
    res_bus = curr_test_case->res_bus;
    res_virt = curr_test_case->res_virt;
    data_bus = curr_test_case->data_bus;
    data_virt = curr_test_case->data_virt;

    /* step2: add all desc comp flag */
    for (i = 0; i < count - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, count);
    if (rv < 0) {
        ret = false;
    }
    
    /* step4: check write back data cnt */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "reg comp desc cnt: %d\n", w);

    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    w = wb_data->completed_desc_count;
    num = w & WB_COUNT_MASK;

    if (num != count) {
        sprintf(err_info, "%s", "recive cnt check failed.\n");
        ret = false;
    }

    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out2:
    pci_free_consistent(sgdev->pdev, curr_test_case->data_size + align_offset,
            curr_test_case->data_virt - align_offset, curr_test_case->data_bus - align_offset);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma h2c ring desc
 */
H2C_TEST_CASE("sgdma h2c ring desc test", al_sgdma_h2c_test8)
{
#define TEST_TX_PAGE_NUM        5//50
#define TEST_POLL_CNT           100
#define TEST_EXPECT_DESC_NUM    20//200

    struct anlogic_dma_engine *engine = h2c_test_engine;
    struct anlogic_dev *sgdev = engine->sgdev;
    struct al_test_case *curr_test_case = &al_sgdma_h2c_test8;
    struct sgdma_transfer *xfer;
    struct scatterlist *sg;
    struct sgdma_common_regs *reg;
    uint32_t w, fill_desc_num;
    unsigned long flags;
    int i, j, pos, rc, tx_cyclic_comp_cnt;
    char prefix[64], err_info[64] = {0};
    bool ret = true;

    if (engine->cyclic_req) {
        sprintf(err_info, "%s: exclusive access already taken.\n",
            engine->name);
        TEST_RET(curr_test_case, false, "%s\n", err_info);
    }

    engine->tx_tail = 0;
    engine->tx_head = 0;
    engine->tx_overrun = 0;
    engine->eop_found = 0;
    engine->tx_desc_cnt = 0;
    engine->tx_ring_desc_num = TEST_TX_PAGE_NUM;

    /* init credit mode */
    w = 0x1 << engine->channel; /* h2c: bit0 ~ 3 */
    reg = (struct sgdma_common_regs
             *)(sgdev->bar[sgdev->config_bar_idx] +
                (0x6 * TARGET_SPACING));

    write_register(w, &reg->credit_mode_enable_w1s,
        (unsigned long)(&reg->credit_mode_enable_w1s) -
               (unsigned long)(reg));

    /* init credit num */
    write_register(TEST_TX_PAGE_NUM / 2, /* half num */
                &engine->sgdma_regs->credits, 
           (unsigned long)(&engine->sgdma_regs->credits) -
               (unsigned long)(engine->sgdma_regs));

    rc = sgt_alloc_with_pages(&engine->cyclic_sgt, engine->tx_ring_desc_num,
                engine->dir, sgdev->pdev);
    if (rc < 0) {
        sprintf(err_info, "%s cyclic pages %u OOM.\n",
            engine->name, engine->tx_ring_desc_num);
        goto err_out;
    }

    engine->cyclic_req = sgdma_init_request(&engine->cyclic_sgt, 0);
    if (!engine->cyclic_req) {
        sprintf(err_info, "%s cyclic request OOM.\n", engine->name);
        rc = -ENOMEM;
        goto err_out;
    }

    xfer = &engine->cyclic_req->tfer[0];

    rc = transfer_init(engine, engine->cyclic_req, xfer);
    if (rc < 0) {
        sprintf(err_info, "%s transfer init failed.\n", engine->name);
        goto err_out;
    }
    
    /* set control of all descriptors, clear stop flag for last desc. */
    for (i = 0; i < xfer->desc_num; i++) {
        sgdma_desc_control_clear(xfer->desc_virt + i, LS_BYTE_MASK);
        sgdma_desc_control_set(xfer->desc_virt + i,
         SGDMA_DESC_EOP | SGDMA_DESC_COMPLETED);
    }

    /*
     * make this a cyclic transfer
     * link last descriptor to first descriptor
     */
    sgdma_desc_link(xfer->desc_virt + xfer->desc_num - 1, xfer->desc_virt, xfer->desc_bus);

    /* remember transfer is cyclic */
    xfer->cyclic = 1;

    if (enable_st_h2c_credit) {
        write_register(engine->tx_ring_desc_num >> 1, &engine->sgdma_regs->credits,
               (unsigned long)(&engine->sgdma_regs->credits) -
                   (unsigned long)(engine->sgdma_regs));
    }

    /* fill sgt data */
    sg = engine->cyclic_sgt.sgl;
    pos = 0;
    for (i = 0; i < engine->cyclic_sgt.nents; i++, sg = sg_next(sg)) {
        pos += al_do_h2c_data_fill(page_address(sg_page(sg)), PAGE_SIZE, AL_H2C_DATA_WIDTH, pos);
    }

    transfer_dump(xfer);

    /* start cyclic transfer */
    transfer_queue(engine, xfer);
    tx_cyclic_comp_cnt = 0;
    for (i = 0; i < TEST_POLL_CNT; i++) {
        rc = engine_service_poll(engine, (i == 0) ? (TEST_EXPECT_DESC_NUM / 2) : (TEST_EXPECT_DESC_NUM - engine->tx_desc_cnt)); /* XXX FIXME */
        if (rc) {
            sprintf(err_info, "%s service_poll failed %d.\n",
                engine->name, rc);
        } else {
            //for (j = 0; j < 1; j++) {//(engine->tx_desc_cnt - tx_cyclic_comp_cnt)
            fill_desc_num = engine->tx_desc_cnt - tx_cyclic_comp_cnt;
            
            for (j = 0; j < fill_desc_num; j++) {
                /* fill h2c data buf */
                pos += al_do_h2c_data_fill(
                    __va(((uint64_t)xfer->desc_virt[engine->tx_head].src_addr_hi << 32) | xfer->desc_virt[engine->tx_head].src_addr_lo),
                    xfer->desc_virt[engine->tx_head].bytes,
                    AL_H2C_DATA_WIDTH, pos);

                /* increment head index */
                engine->tx_head = (engine->tx_head + 1) % engine->tx_ring_desc_num;
            }
            
            tx_cyclic_comp_cnt = engine->tx_desc_cnt;
            
            write_register(fill_desc_num,
                &engine->sgdma_regs->credits, 
               (unsigned long)(&engine->sgdma_regs->credits) - (unsigned long)(engine->sgdma_regs));
        }
        
        if (engine->tx_desc_cnt >= TEST_EXPECT_DESC_NUM) {
            break;
        }
    }

    ret = al_do_h2c_data_check(curr_test_case);

    if (engine->tx_desc_cnt < TEST_EXPECT_DESC_NUM) {
        ret = false;
        sprintf(curr_test_case->err_info, "tx desc num(%d) less than expect(%d).\n", engine->tx_desc_cnt, TEST_EXPECT_DESC_NUM);
    }

#if 0
    /* stop tx ring engine */
    engine_status_read(engine, 1, 0);
    if ((engine->running) && !(engine->status & SGDMA_STAT_BUSY)) {
        /* transfers on queue? */
        if (!list_empty(&engine->transfer_list))
            engine_transfer_dequeue(engine);

        engine_service_shutdown(engine);
    }
#else
    /* stop engine right now */
    if (engine->running) {
        /* transfers on queue? */
        if (!list_empty(&engine->transfer_list))
            engine_transfer_dequeue(engine);

        engine_service_shutdown(engine);
    }
#endif

    /* unwind on errors */
err_out:
    /* disable credit mode */
    w = 0x1 << engine->channel; /* h2c: bit0 ~ 3 */
    write_register(w, &reg->credit_mode_enable_w1c,
        (unsigned long)(&reg->credit_mode_enable_w1c) -
               (unsigned long)(reg));

    if (engine->cyclic_req) {
        sgdma_request_free(engine->cyclic_req);
        engine->cyclic_req = NULL;
    }
    
    if (engine->cyclic_sgt.orig_nents) {
        sgt_free_with_pages(&engine->cyclic_sgt, engine->dir,
                sgdev->pdev);
        engine->cyclic_sgt.orig_nents = 0;
        engine->cyclic_sgt.nents = 0;
        engine->cyclic_sgt.sgl = NULL;
    }

    if (rc < 0)
        ret = false;

    engine->tx_ring_desc_num = 0;

    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

#include <linux/sched.h>
#include <linux/tty.h>
//#include <linux/sched/signal.h>
/**
 * test sgdma h2c credit mode
 */
H2C_TEST_CASE("kernel printk test", al_sgdma_printk_test)
{
    struct tty_struct *cur_tty;
    struct al_test_case *curr_test_case = &al_sgdma_printk_test;

    cur_tty = current->signal->tty;

    if (cur_tty) {
        cur_tty->ops->write(cur_tty, "123\r\n456711223344\r\n", 19);
    }

    TEST_RET(curr_test_case, true, "\n");
}

void al_pci_h2c_case_init(struct al_tester *tester)
{
    struct anlogic_dev *sgdev = to_anlogic_sgdev(tester->dev);

    h2c_test_engine = &sgdev->engine_h2c[0];

    AL_PCIE_DBG_ADD(al_sgdma_h2c_test0);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test1);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test2);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test3);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test4);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test5);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test6);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test7);
    AL_PCIE_DBG_ADD(al_sgdma_h2c_test8);
    //AL_PCIE_DBG_ADD(al_sgdma_printk_test);
}

