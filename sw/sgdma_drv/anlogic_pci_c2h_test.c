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

static struct anlogic_dma_engine *c2h_test_engine;

/**
 * test sgdma bar0 user register read/write
 */
C2H_TEST_CASE("user register read/write test", al_sgdma_test0)
{
#define BAR_USER_MEM_OFFSET (1UL << 19)
#define BAR_USER_MEM_SIZE   0x800

    void *user_addr;
    uint32_t *pmem;
    uint32_t i, j, data;
    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct al_test_case *curr_test_case = &al_sgdma_test0;
    char prefix[64];
    bool ret = true;
    uint32_t skip_reg_addr[] = {
        0x0,
        0x4
    };

    user_addr = sgdev->bar[0] + BAR_USER_MEM_OFFSET;
    pmem = (uint32_t *)user_addr;

    /* write first */
    for (i = 0; i < BAR_USER_MEM_SIZE; i += sizeof(uint32_t), pmem++) {
        for (j = 0; j < ARRAY_SIZE(skip_reg_addr); j++)
            if (i == skip_reg_addr[j])
                continue;

        iowrite32(i, pmem);
    }

    pmem = (uint32_t *)user_addr;

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 4, user_addr, BAR_USER_MEM_SIZE, true);

    /* read and check. */
    for (i = 0; i < BAR_USER_MEM_SIZE; i += sizeof(uint32_t), pmem++) {
        for (j = 0; j < ARRAY_SIZE(skip_reg_addr); j++)
            if (i == skip_reg_addr[j])
                continue;

        data = ioread32(pmem);
        if (data != i) {
            TEST_DBG(curr_test_case, "user mem 0x%pK, bar offset 0x%llx, read value 0x%08x, write value 0x%08x\n", 
                    pmem, (uint64_t)((void *)pmem - sgdev->bar[0]), data, i);
            ret = false;
        }
    }

    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test sgdma bar0 user register read/write
 */
C2H_TEST_CASE("sgdma c2h poll mode test", al_sgdma_test1)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0x100
#define TEST_DESC_NUM       5
#define TEST_DATA_DUMP_SIZE 0x200

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test1;
    uint32_t *user_reg = sgdev->bar[0] + BAR_USER_MEM_OFFSET;
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
    al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);

    /* step3: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }

    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "comp desc cnt: %d\n", w);

#if 0
    w = read_register(&engine->regs->perf_dat_hi);
    TEST_DBG(curr_test_case, "performance data cnt hi: %d\n", w);

    w = read_register(&engine->regs->perf_dat_lo);
    TEST_DBG(curr_test_case, "performance data cnt lo: %d\n", w);

    //ret = al_do_data_check(curr_test_case, curr_test_case->data_virt, curr_test_case->data_size);
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, TEST_DESC_NUM);
    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }
#endif

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt, TEST_DATA_DUMP_SIZE, true);

    snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8,
        data_virt + curr_test_case->data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream mode write back
 */
C2H_TEST_CASE("sgdma c2h stream mode test", al_sgdma_test2)
{
    struct al_test_case *curr_test_case = &al_sgdma_test2;
    bool ret;
    char err_info[256] = {0};

    ret = al_stream_transfer_test(curr_test_case);

    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream bad desc magic test
 */
C2H_TEST_CASE("sgdma c2h stream bad desc magic test", al_sgdma_test3)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       5
#define TEST_DATA_DUMP_SIZE 0x100

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test3;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, next_adj, w, desc_wb;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], desc_str[256];
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 1);
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
    al_pci_test_engine_do_transfer_with_no_stop(curr_test_case, engine, TEST_DESC_NUM, 0);

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

#if 0
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
#endif

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (desc_wb < TEST_DESC_NUM) ? desc_wb : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt,
        min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[0].length), true);

    snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE,
        min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[num - 1].length), true);

    al_pci_test_free_desc(curr_test_case);

    /* step5: start a new ok transfer */
    ret = al_stream_transfer_test(curr_test_case);

out1:
    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test sgdma c2h stream mode write back
 */
C2H_TEST_CASE("sgdma submit desc test", al_sgdma_test4)
{
#define TEST_PAGES_NUM      10

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test4;
    ssize_t comp_size;
    char *data_virt;
    char prefix[64];
    bool ret = true;
    int i, rv, num;
    struct sg_table test_sgt;
    struct scatterlist *sg;

    /* step1: alloc sgt */
    rv = sgt_alloc_with_pages(&test_sgt, TEST_PAGES_NUM, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out1;
    }

    sg = test_sgt.sgl;

    /* step2: submit transfer */
    comp_size = sgdma_xfer_submit(sgdev, engine->channel, false, 0, &test_sgt, true, 5 * 1000);
    if (comp_size < 0) {
        TEST_DBG(curr_test_case, "%s xfer submit failed, ret %zd.\n",
            engine->name, comp_size);
        ret = false;
        goto out2;
    }

    TEST_DBG(curr_test_case, "sgdma_xfer_submit result: %zu(0x%zx)\n", comp_size, comp_size);

    for (i = 0; i < test_sgt.orig_nents; i++, sg = sg_next(sg)) {
        snprintf(prefix, 64, "[%s sg %d]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, page_address(sg_page(sg)), 2048, true);
    }

out2:
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    TEST_RET(curr_test_case, ret, "\n");
}

void buf_hex_dump(uint8_t *buf, int length)
{
	int i;
	char linestr[256] = {0};
	int offset = 0;
#define BYTES_PER_LINE 16
        for (i = 0; i < length; i++) {
          if ((i % BYTES_PER_LINE) == 0) {
            printk("%s\n", linestr);
            offset = 0;
            offset += sprintf(&linestr[offset], "\n%08x  ", i);
          }

            offset += sprintf(&linestr[offset], " %02x", buf[i]);
        }

	if (length < BYTES_PER_LINE)
            printk("%s\n", linestr);
		
        printk("\n");
}

bool data_buf_check(int idx, uint8_t *buf, int length)
{
	int i, pattern;
	for (i = 0, pattern = 1; i < length; i++) {
	    pattern &= 0xff;
	    if (buf[i] != pattern) {
	        printk("desc %d, rx: %02x, pattern: %02x, offset: %d\n", idx, buf[i], pattern, i);
	        return false;
	    }
	    pattern++;
	    //if (pattern == 256)
	    //    pattern = 1;
	}
	return true;
}

/**
 * sgdma c2h stream random desc length
 */
C2H_TEST_CASE("sgdma c2h stream random desc length", al_sgdma_test5)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       (4096)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1
#define DATA_ALIGN_BYTES    64
#define DATA_ALIGN_MASK     (DATA_ALIGN_BYTES - 1)

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test5;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, extra_adj, w;
    void *data_virt;
    char prefix[64], desc_str[256], err_info[256] = {0};
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;
    struct sg_table test_sgt;
    struct scatterlist *sg;
#if 1
    int *buf_size = (int *)kzalloc(sizeof(int) * TEST_DESC_NUM, GFP_KERNEL);
    if (buf_size == NULL) {
        TEST_DBG(curr_test_case, "%s desc_virt %lu OOM\n",
            engine->name, sizeof(int) * TEST_DESC_NUM);
        ret = false;
        goto out2;
    }
#else
    int buf_size[TEST_DESC_NUM] = {
        [0] = 512,
        [1] = 4096,
        [2] = 4096,
        [3] = 4096,
        [4] = 133,
        [5] = 4096,
        [6] = 64,
        [7] = 333,
        [8] = 96,
        [9] = 160,
        [10] = 192,
        [11] = 387,
        [12] = 497,
        [13] = 699,
        [14] = 147,
        [15] = 833,
        [16] = 743,
        [17] = 996,
        [18] = 0xc0,
        [19] = 64,
    };
#endif
    count = TEST_DESC_NUM;

    /* gen random data length */
    for (i = 0; i < count; i++) {
        //buf_size[i] = al_random_num_gen(64, 4096);
        buf_size[i] = 0x1000;
    }

#if 0
    /* step1: build 64bit addr desc */
    rv = sgt_alloc_with_pages(&test_sgt, 2, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out2;
    }

    /* first page used to desc */
    sg = test_sgt.sgl;

    count = sg_dma_len(sg) / sizeof(struct sgdma_desc);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));
    desc_bus = sg_dma_address(sg);

    curr_test_case->desc_virt = desc_virt;
    curr_test_case->desc_bus = desc_bus;

    /* 2th page used to desc res */
    sg = sg_next(sg);

    count = sg_dma_len(sg) / sizeof(struct sgdma_result);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    res_virt = (struct sgdma_result *)page_address(sg_page(sg));
    res_bus = sg_dma_address(sg);

    curr_test_case->res_virt = res_virt;
    curr_test_case->res_bus = res_bus;
#else
    desc_virt = pci_alloc_consistent(sgdev->pdev, count * sizeof(struct sgdma_desc), &desc_bus);
    if (desc_virt == NULL) {
        TEST_DBG(curr_test_case, "%s desc_virt %lu OOM\n",
            engine->name, count * sizeof(struct sgdma_desc));
        ret = false;
        goto out2;
    }

    res_virt = pci_alloc_consistent(sgdev->pdev, count * sizeof(struct sgdma_result), &res_bus);
    if (res_virt == NULL) {
        pci_free_consistent(sgdev->pdev, count * sizeof(struct sgdma_desc), desc_virt, desc_bus);
        TEST_DBG(curr_test_case, "%s res_virt %lu OOM\n",
            engine->name, count * sizeof(struct sgdma_result));
        ret = false;
        goto out2;
    }

    curr_test_case->desc_virt = desc_virt;
    curr_test_case->desc_bus = desc_bus;
    curr_test_case->res_virt = res_virt;
    curr_test_case->res_bus = res_bus;
#endif


    /* alloc some data buf */
    /* calc desc total size */
    curr_test_case->data_size = 0;
    for (i = 0; i < count; i++) {
        /* must keep 64 byte align. */
        buf_size[i] &= ~DATA_ALIGN_MASK;
        //buf_size[i] &= 0xff;
        curr_test_case->data_size += buf_size[i];
    }

#if 0
    curr_test_case->data_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->data_size, &curr_test_case->data_bus);
    if (curr_test_case->data_virt == NULL) {
        TEST_DBG(curr_test_case, "pci desc alloc consistent failed.\n");
        ret = false;
        goto out1;
    }
    memset(curr_test_case->data_virt, 0, curr_test_case->data_size);
#else
    /* step1: build 64bit data buf */
    rv = sgt_alloc_with_pages(&test_sgt, count, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, count, rv);
        ret = false;
        goto out1;
    }
#endif

    al_pci_test_desc_link(desc_virt, desc_bus, count);

#if 0
    /* step4: config desc data addr and control filed */
    for (data_bus = curr_test_case->data_bus, i = 0; i < count; i++) {
        sgdma_desc_set(&desc_virt[i], data_bus, 0, buf_size[i], curr_test_case->dir);
        sgdma_desc_control_clear(&desc_virt[i], LS_BYTE_MASK);
        data_bus += buf_size[i];
    }
#else
    for (sg = test_sgt.sgl, i = 0; i < count; i++, sg = sg_next(sg)) {
        sgdma_desc_set(&desc_virt[i], sg_dma_address(sg), 0, buf_size[i], curr_test_case->dir);
        sgdma_desc_control_clear(&desc_virt[i], LS_BYTE_MASK);
    }
#endif

    /* stop engine, EOP for AXI ST, req IRQ on last descriptor */
    control = SGDMA_DESC_STOPPED;
    control |= SGDMA_DESC_EOP;
    control |= SGDMA_DESC_COMPLETED;
    sgdma_desc_control_set(&desc_virt[count - 1], control);

    if (TEST_IS_STREAM_MODE) {
        /* replace source addresses with result write-back addresses */
        for (i = 0; i < count; i++) {
            desc_virt[i].src_addr_lo = cpu_to_le32(PCI_DMA_L(res_bus));
            desc_virt[i].src_addr_hi = cpu_to_le32(PCI_DMA_H(res_bus));
            res_bus += sizeof(struct sgdma_result);
        }
    }

    al_pci_test_fill_adj_num(desc_virt, count);

    /* step2: add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        ;//sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    al_pci_test_engine_do_transfer(curr_test_case, engine, count);

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    //TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    TEST_DBG(curr_test_case, "hw stream write back result: %d\n", num);


    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

#if 0
    //ret = al_do_data_check(curr_test_case, curr_test_case->data_virt, curr_test_case->data_size);
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, count);

    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    data_virt = curr_test_case->data_virt;
    for (i = 0; i < count; i++) {
        snprintf(prefix, 64, "[%s desc%d]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt, min((u32)buf_size[i], curr_test_case->res_virt[i].length), true);
        printk("\n");
        data_virt += buf_size[i];
    }
#else
    for (sg = test_sgt.sgl, i = 0; i < count; i++, sg = sg_next(sg)) {
        snprintf(prefix, 64, "[%s desc%d]", curr_test_case->alias, i);
        //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, page_address(sg_page(sg)), min((u32)buf_size[i], curr_test_case->res_virt[i].length), true);
        //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, page_address(sg_page(sg)), i + 1, true);
        //buf_hex_dump(page_address(sg_page(sg)), 16);
        ret = data_buf_check(i, page_address(sg_page(sg)), i + 1);
	if (ret == false) {
	    sprintf(err_info, "data check failed.");
	}
    }
#endif

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);
out3:
    //pci_free_consistent(sgdev->pdev, curr_test_case->data_size, curr_test_case->data_virt, curr_test_case->data_bus);
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    kfree(buf_size);
    //sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
    pci_free_consistent(sgdev->pdev, count * sizeof(struct sgdma_desc), curr_test_case->desc_virt, curr_test_case->desc_bus);
    pci_free_consistent(sgdev->pdev, count * sizeof(struct sgdma_result), curr_test_case->res_virt, curr_test_case->res_bus);
out2:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * sgdma c2h stream random desc length
 */
C2H_TEST_CASE("sgdma c2h stream random data length/addr align byte (1 ~ 8)", al_sgdma_test5_1)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       (20)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1
#define DATA_ALIGN_BYTES    64
#define DATA_ALIGN_MASK     (DATA_ALIGN_BYTES - 1)
#define DATA_ADDR_ALIGN_BYTES(n, align) (((n) / (align) * (align) == (n)) ? (n) : (((n) / (align) + 1) * (align)))

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test5_1;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size, buf_total_size = 0;
    uint32_t control = 0, status, extra_adj, desc_align, w;
    void *data_virt;
    char prefix[64], desc_str[256], err_info[256] = {0};
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;
    struct sg_table test_sgt;
    struct scatterlist *sg;

    int buf_size[] = {
        [0] = 512,
        [1] = 64,
        [2] = 128,
        [3] = 256,
        [4] = 512,
        [5] = 1024,
        [6] = 2048,
        [7] = 4096,
        [8] = 96,
        [9] = 160,
        [10] = 192,
        [11] = 387,
        [12] = 497,
        [13] = 699,
        [14] = 147,
        [15] = 833,
        [16] = 743,
        [17] = 996,
        [18] = 1994,
        [19] = 3753,
    };

    int data_align_array[] = {
        [0] = 1,
        [1] = 2,
        [2] = 3,
        [3] = 4,
        [4] = 5,
        [5] = 6,
        [6] = 7,
        [7] = 8,
    };

    count = (count < ARRAY_SIZE(buf_size)) ? count : ARRAY_SIZE(buf_size);

    /* gen random data length */
    for (i = 0; i < ARRAY_SIZE(buf_size); i++) {
        buf_size[i] = al_random_num_gen(64, 4096);
    }

    /* gen random data_align_array */
    for (i = 0; i < ARRAY_SIZE(data_align_array); i++) {
        data_align_array[i] = al_random_num_gen(1, 9);
    }

    /* step1: build 64bit addr desc */
    rv = sgt_alloc_with_pages(&test_sgt, 2, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out2;
    }

    /* first page used to desc */
    sg = test_sgt.sgl;

    count = sg_dma_len(sg) / sizeof(struct sgdma_desc);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));
    desc_bus = sg_dma_address(sg);

    curr_test_case->desc_virt = desc_virt;
    curr_test_case->desc_bus = desc_bus;

    /* 2th page used to desc res */
    sg = sg_next(sg);

    count = sg_dma_len(sg) / sizeof(struct sgdma_result);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    res_virt = (struct sgdma_result *)page_address(sg_page(sg));
    res_bus = sg_dma_address(sg);

    curr_test_case->res_virt = res_virt;
    curr_test_case->res_bus = res_bus;

    /* alloc some data buf */
    /* calc desc total size */
    curr_test_case->data_size = 0;
    for (i = 0; i < count; i++) {
        /* must keep 64 byte align. */
        buf_size[i] &= ~DATA_ALIGN_MASK;
        curr_test_case->data_size += buf_size[i];
        buf_total_size += buf_size[i];
    }

    curr_test_case->data_size += 256; /* do data align (0 ~ 8 bytes) test */

    curr_test_case->data_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->data_size, &curr_test_case->data_bus);
    if (curr_test_case->data_virt == NULL) {
        TEST_DBG(curr_test_case, "pci desc alloc consistent failed.\n");
        ret = false;
        goto out1;
    }
    memset(curr_test_case->data_virt, 0, curr_test_case->data_size);

    al_pci_test_desc_link(desc_virt, desc_bus, count);

    /* step4: config desc data addr and control filed */
    for (data_bus = curr_test_case->data_bus, i = 0; i < count; i++) {

        TEST_DBG(curr_test_case, "desc[%d](%d): before align: 0x%llx\n", i, data_align_array[i % 8], data_bus);
        data_bus = DATA_ADDR_ALIGN_BYTES(data_bus, data_align_array[i % 8]);
        TEST_DBG(curr_test_case, "desc[%d](%d): after  align: 0x%llx\n", i, data_align_array[i % 8], data_bus);

        sgdma_desc_set(&desc_virt[i], data_bus, 0, buf_size[i], curr_test_case->dir);
        sgdma_desc_control_clear(&desc_virt[i], LS_BYTE_MASK);
        data_bus += buf_size[i];
    }

    /* stop engine, EOP for AXI ST, req IRQ on last descriptor */
    control = SGDMA_DESC_STOPPED;
    control |= SGDMA_DESC_EOP;
    control |= SGDMA_DESC_COMPLETED;
    sgdma_desc_control_set(&desc_virt[count - 1], control);

    if (TEST_IS_STREAM_MODE) {
        /* replace source addresses with result write-back addresses */
        for (i = 0; i < count; i++) {
            desc_virt[i].src_addr_lo = cpu_to_le32(PCI_DMA_L(res_bus));
            desc_virt[i].src_addr_hi = cpu_to_le32(PCI_DMA_H(res_bus));
            res_bus += sizeof(struct sgdma_result);
        }
    }

    al_pci_test_fill_adj_num(desc_virt, count);

    /* step2: add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "hw stream write back result, except total size %zu :\n", buf_total_size);
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    
    buf_total_size = 0;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
        buf_total_size += res_virt[i].length;
    }
    
    TEST_DBG(curr_test_case, "receive total size %zu\n", buf_total_size);

    //ret = al_do_data_check(curr_test_case, curr_test_case->data_virt, curr_test_case->data_size);
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, count);

    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    for (i = 0; i < count; i++) {
        snprintf(prefix, 64, "[%s desc%d]", curr_test_case->alias, i);
		
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8,
			__va(((uint64_t)curr_test_case->desc_virt[i].dst_addr_hi << 32) | 
				curr_test_case->desc_virt[i].dst_addr_lo),
			min((u32)buf_size[i], curr_test_case->res_virt[i].length), true);
			
        printk("\n");
    }

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);
out3:
    pci_free_consistent(sgdev->pdev, curr_test_case->data_size, curr_test_case->data_virt, curr_test_case->data_bus);
out1:
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out2:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream random desc stop
 */
C2H_TEST_CASE("sgdma c2h stream random desc stop", al_sgdma_test6)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0x1000
#define TEST_DESC_NUM       500
#define TEST_DATA_DUMP_SIZE 0x80

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test6;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, extra_adj, w;
    void *data_virt;
    char prefix[64], desc_str[256];
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 1);
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
#if 0
    /* dump desc */
    for (i = 0; i < count; i++) {
        dump_desc(&curr_test_case->desc_virt[i]);
    }

    for (i = 0; i < count; i++) {
        format_desc(desc_str, 256, &curr_test_case->desc_virt[i]);
        printf("%s", desc_str);
    }
#endif

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

    /* step5: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    data_virt = curr_test_case->data_virt;
    for (i = 0; i < 10; i++) {
        
        snprintf(prefix, 64, "[%s desc%d start]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt, min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
    
        snprintf(prefix, 64, "[%s desc%d   end]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8,
            data_virt + TEST_DATA_SIZE - min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length),
            min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
        
        printk("\n");
        data_virt += TEST_DATA_SIZE;
    }

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test sgdma c2h stream random 32/64 bits data buf addr
 */
C2H_TEST_CASE("sgdma c2h stream data addr random 32/64 bits", al_sgdma_test7_0)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       (20)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test7_0;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, extra_adj, w;
    void *data_virt_32, *dst_virt;
    char prefix[64], desc_str[256], err_info[256] = {0};
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;
    struct sg_table test_sgt;
    struct scatterlist *sg;

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, TEST_IS_STREAM_MODE);
    if (rv < 0) {
        ret = false;
        TEST_DBG(curr_test_case, "alloc test build desc failed, %d\n", rv);
        goto out2;
    }

    rv = sgt_alloc_with_pages(&test_sgt, TEST_DESC_NUM, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out1;
    }

    desc_virt = curr_test_case->desc_virt;
    res_virt = curr_test_case->res_virt;
    desc_bus = curr_test_case->desc_bus;
    res_bus = curr_test_case->res_bus;

    data_bus = curr_test_case->data_bus;
    data_virt_32 = curr_test_case->data_virt;

    /* modify data buf to 32 or 64 bit addr */
    do {
        int data_pos[] = {
            [0] = 32,
            [1] = 64,
            [2] = 32,
            [3] = 64,
            [4] = 32,
            [5] = 64,
            [6] = 32,
            [7] = 64,
            [8] = 32,
            [9] = 64,
            [10] = 64,
            [11] = 64,
            [12] = 32,
            [13] = 32,
            [14] = 64,
            [15] = 64,
            [16] = 32,
            [17] = 32,
            [18] = 64,
            [19] = 32,
        };

        /* gen random data bits */
        for (i = 0; i < ARRAY_SIZE(data_pos); i++) {
            data_pos[i] = (al_random_num_gen(1, 10) < 5) ? 32 : 64;
        }

        /* step4: config desc data addr and control filed */
        for (sg = test_sgt.sgl, i = 0; i < count; i++) {
            if (data_pos[i] == 64) {
                sgdma_desc_set(&desc_virt[i], sg_dma_address(sg), 0, sg_dma_len(sg), curr_test_case->dir);
                sg = sg_next(sg);
            } else {
                sgdma_desc_set(&desc_virt[i], data_bus, 0, TEST_DATA_SIZE, curr_test_case->dir);
                data_bus += TEST_DATA_SIZE;
            }
        }

        if (TEST_IS_STREAM_MODE) {
            /* replace source addresses with result write-back addresses */
            for (i = 0; i < count; i++) {
                desc_virt[i].src_addr_lo = cpu_to_le32(PCI_DMA_L(res_bus));
                desc_virt[i].src_addr_hi = cpu_to_le32(PCI_DMA_H(res_bus));
                res_bus += sizeof(struct sgdma_result);
            }
        }
    } while (0);

    /* step2: add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step3: run engine do transfer */
    al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, TEST_DESC_NUM);

    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    for (i = 0; i < TEST_DESC_NUM; i++) {
        dst_virt = __va(((uint64_t)desc_virt[i].dst_addr_hi << 32) | desc_virt[i].dst_addr_lo);
        
        snprintf(prefix, 64, "[%s desc%d(0x%x) start]", curr_test_case->alias, i, desc_virt[i].bytes);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, dst_virt, min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
        
        snprintf(prefix, 64, "[%s desc%d(0x%x)   end]", curr_test_case->alias, i, desc_virt[i].bytes);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8,
            dst_virt + desc_virt[i].bytes - min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length),
            min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
        
        printk("\n");
    }

    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    al_pci_test_free_desc(curr_test_case);
out2:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream 64 bits desc buf addr
 */
C2H_TEST_CASE("sgdma c2h stream 64 bits desc buf addr", al_sgdma_test7_1)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0x100
#define TEST_DESC_NUM       (20)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test7_1;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, extra_adj, w;
    void *data_virt;
    char prefix[64], desc_str[256], err_info[256] = {0};
    bool ret = true;
    int i, rv, num, count;
    struct sg_table test_sgt;
    struct scatterlist *sg;

    /* step1: build 64bit addr desc */

    rv = sgt_alloc_with_pages(&test_sgt, 2, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out1;
    }

    /* first page used to desc */
    sg = test_sgt.sgl;

    count = sg_dma_len(sg) / sizeof(struct sgdma_desc);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));

    curr_test_case->desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));
    curr_test_case->desc_bus = sg_dma_address(sg);

    /* 2th page used to desc res */
    sg = sg_next(sg);

    count = sg_dma_len(sg) / sizeof(struct sgdma_result);
    count = (count < TEST_DESC_NUM) ? count : TEST_DESC_NUM;

    curr_test_case->res_virt = (struct sgdma_result *)page_address(sg_page(sg));
    curr_test_case->res_bus = sg_dma_address(sg);

    /* alloc some data buf */
    curr_test_case->data_size = count * TEST_DATA_SIZE;

    curr_test_case->data_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->data_size, &curr_test_case->data_bus);
    if (curr_test_case->data_virt == NULL) {
        TEST_DBG(curr_test_case, "pci data alloc consistent failed.\n");
        ret = false;
        goto out2;
    }

    al_pci_test_build_desc_no_alloc(curr_test_case, count, TEST_DATA_SIZE, TEST_IS_STREAM_MODE);

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
    al_pci_test_engine_do_transfer(curr_test_case, engine, count);

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < count) ? wb_data->completed_desc_count : count;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    //ret = al_do_data_check(curr_test_case, curr_test_case->data_virt, curr_test_case->data_size);
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, count);

    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    //snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt, TEST_DATA_DUMP_SIZE, true);

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);
    pci_free_consistent(sgdev->pdev, curr_test_case->data_size, curr_test_case->data_virt, curr_test_case->data_bus);
out2:
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream dst addr cross 4k bound
 */
C2H_TEST_CASE("sgdma c2h stream dst addr cross 4k bound", al_sgdma_test8_0)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DESC_NUM       (10)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1
#define TEST_ALIGN_OFFSET   64
#define SIZE_4K             (0x1000UL)
#define BUF_4K_MASK         (SIZE_4K - 1)
#define TEST_DATA_SIZE      SIZE_4K

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test8_0;
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

    al_pci_test_build_desc_no_alloc(curr_test_case, count, TEST_DATA_SIZE, TEST_IS_STREAM_MODE);

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
    al_pci_test_engine_do_transfer(curr_test_case, engine, count);

    /* step4: check write back data */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "reg comp desc cnt: %d\n", w);

    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    w = wb_data->completed_desc_count;
    num = w & WB_COUNT_MASK;

    if (num != count) {
        sprintf(err_info, "%s", "recive cnt check failed.\n");
        ret = false;
    }

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (num < count) ? num : count;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    /* check data */
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, count);
    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt, min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[0].length), true);

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out2:
    pci_free_consistent(sgdev->pdev, curr_test_case->data_size + align_offset,
            curr_test_case->data_virt - align_offset, curr_test_case->data_bus - align_offset);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream result wb addr cross 4k bound
 */
C2H_TEST_CASE("sgdma c2h stream result wb addr cross 4k bound", al_sgdma_test8_1)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DESC_NUM       (10)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 1
#define TEST_ALIGN_OFFSET   64
#define SIZE_4K             (0x1000UL)
#define BUF_4K_MASK         (SIZE_4K - 1)
#define TEST_DATA_SIZE      SIZE_4K

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test8_1;
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

    /* alloc desc result */
    count = TEST_DESC_NUM;
    curr_test_case->res_size = count * sizeof(struct sgdma_result) + SIZE_4K;

    curr_test_case->res_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->res_size, &curr_test_case->res_bus);
    if (curr_test_case->res_virt == NULL) {
        TEST_DBG(curr_test_case, "pci data alloc consistent failed.\n");
        ret = false;
        goto out1;
    }

    /*
     * build cross 4k bound res wb
     * make half desc wb to cross 4k bound
     */
    if ((curr_test_case->data_bus & BUF_4K_MASK) != 0) {
        align_offset = (SIZE_4K - (curr_test_case->data_bus & BUF_4K_MASK)) - (sizeof(struct sgdma_result) >> 1);

        TEST_DBG(curr_test_case, "data buf start addr(0x%llx) is not 4k aligned, offset 0x%x\n",
            curr_test_case->data_bus, align_offset);
    } else {
        align_offset = SIZE_4K - (sizeof(struct sgdma_result) >> 1);

        TEST_DBG(curr_test_case, "res wb start addr(0x%llx) is 4k aligned, to make offset 0x%x\n",
            curr_test_case->res_bus, align_offset);
    }

    //align_offset = 0;

    curr_test_case->res_virt += align_offset;
    curr_test_case->res_bus += align_offset;
    curr_test_case->res_size -= align_offset;

    /* alloc some desc */
    rv = sgt_alloc_with_pages(&test_sgt, 1, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, 1, rv);
        ret = false;
        goto out2;
    }

    sg = test_sgt.sgl;

    if (count > (sg_dma_len(sg) / sizeof(struct sgdma_desc))) {
        count = sg_dma_len(sg) / sizeof(struct sgdma_desc);
    }

    curr_test_case->desc_virt = (struct sgdma_desc *)page_address(sg_page(sg));
    curr_test_case->desc_bus = sg_dma_address(sg);

    /* alloc some data buf */
    curr_test_case->data_size = count * TEST_DATA_SIZE;

    curr_test_case->data_virt = pci_alloc_consistent(sgdev->pdev, curr_test_case->data_size, &curr_test_case->data_bus);
    if (curr_test_case->data_virt == NULL) {
        sprintf(err_info, "%s", "pci data alloc consistent failed.\n");
        ret = false;
        goto out3;
    }

    al_pci_test_build_desc_no_alloc(curr_test_case, count, TEST_DATA_SIZE, TEST_IS_STREAM_MODE);

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
    al_pci_test_engine_do_transfer(curr_test_case, engine, count);

    /* step4: check write back data */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "reg comp desc cnt: %d\n", w);

    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    w = wb_data->completed_desc_count;
    num = w & WB_COUNT_MASK;

    if (num != count) {
        sprintf(err_info, "%s", "recive cnt check failed.\n");
        ret = false;
    }

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (num < count) ? num : count;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    /* check data */
    ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, count);
    if (ret == false) {
        sprintf(err_info, "%s", "data check failed.");
    }

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt, min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[0].length), true);

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);

    pci_free_consistent(sgdev->pdev, curr_test_case->data_size, curr_test_case->data_virt, curr_test_case->data_bus);
out3:
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out2:
    pci_free_consistent(sgdev->pdev, curr_test_case->res_size + align_offset,
            curr_test_case->res_virt - align_offset, curr_test_case->res_bus - align_offset);
out1:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/**
 * test sgdma c2h stream mode write back
 */
C2H_TEST_CASE("do fpga sgdma desc test", al_sgdma_test55)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DESC_NUM 5

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    volatile struct sgdma_poll_wb *wb_data;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test55;
    char *data_virt;
    char prefix[64];
    bool ret = false;
    int i, rv, next_adj, count = TEST_DESC_NUM;
    struct sg_table test_sgt;
    struct scatterlist *sg;
    size_t data_size, res_size;
    u32 w;
    volatile u32 *poll_val;

    write_register(0xcf83e06, &engine->regs->control,
               (unsigned long)(&engine->regs->control) -
                   (unsigned long)(engine->regs));
    
    write_register(0x06, &engine->regs->interrupt_enable_mask,
               (unsigned long)(&engine->regs->interrupt_enable_mask) -
                   (unsigned long)(engine->regs));

    TEST_DBG(curr_test_case, "poll addr hi: 0x%08llux\n", PCI_DMA_H(engine->poll_mode_bus));
    TEST_DBG(curr_test_case, "poll addr ho: 0x%08llux\n", PCI_DMA_L(engine->poll_mode_bus));
    TEST_DBG(curr_test_case, "poll addr virt: 0x%pK\n", engine->poll_mode_addr_virt);
    memset(engine->poll_mode_addr_virt, 0, sizeof(struct sgdma_poll_wb));
    poll_val = (u32 *)(engine->poll_mode_addr_virt);
    for (i = 0; i < sizeof(struct sgdma_poll_wb); i += 4, poll_val++) {
        TEST_DBG(curr_test_case, "poll addr[%d]: 0x%08x\n", i >> 2, *poll_val);
    }

    w = cpu_to_le32(PCI_DMA_L(engine->poll_mode_bus));
    write_register(w, &engine->regs->poll_mode_wb_lo,
               (unsigned long)(&engine->regs->poll_mode_wb_lo) -
                   (unsigned long)(engine->regs));
    w = cpu_to_le32(PCI_DMA_H(engine->poll_mode_bus));
    write_register(w, &engine->regs->poll_mode_wb_hi,
               (unsigned long)(&engine->regs->poll_mode_wb_hi) -
                   (unsigned long)(engine->regs));

    /* step2: build transfer desc. */
    al_pci_test_build_desc(curr_test_case, count, 256, 0);

    /* step3: fill in adjacent numbers */
    for (i = 0; i < count; i++) {
        next_adj = sgdma_get_next_adj(count - i - 1,
                        (curr_test_case->desc_virt + i)->next_lo);

        dbg_desc("set next adj at index %d to %u\n", i, next_adj);
        sgdma_desc_adjacent(curr_test_case->desc_virt + i, next_adj);
    }

    /* DBG: dump desc */
    for (i = 0; i < count; i++) {
        dump_desc(&curr_test_case->desc_virt[i]);
    }

    /* step4: config sgdma regs */
    write_register(cpu_to_le32(PCI_DMA_L(curr_test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_lo, 
            (unsigned long)(&engine->sgdma_regs->first_desc_lo) -
                (unsigned long)(engine->sgdma_regs));

    write_register(cpu_to_le32(PCI_DMA_H(curr_test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_hi, 
            (unsigned long)(&engine->sgdma_regs->first_desc_hi) -
                (unsigned long)(engine->sgdma_regs));

    next_adj = sgdma_get_next_adj(count, cpu_to_le32(PCI_DMA_L(curr_test_case->desc_bus)));
    write_register(cpu_to_le32(next_adj - 1),
            &engine->sgdma_regs->first_desc_adjacent, 
            (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
                (unsigned long)(engine->sgdma_regs));

    /* statrt engine. */
    write_register(0xcf83e07, &engine->regs->control,
               (unsigned long)(&engine->regs->control) -
                   (unsigned long)(engine->regs));

    w = read_register(&engine->regs->control);
    TEST_DBG(curr_test_case, "control reg: 0x%08x\n", w);

    msleep(5000);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "comp desc cnt: %d\n", w);

    w = read_register(&engine->regs->perf_dat_hi);
    TEST_DBG(curr_test_case, "performance data cnt hi: %d\n", w);

    w = read_register(&engine->regs->perf_dat_lo);
    TEST_DBG(curr_test_case, "performance data cnt lo: %d\n", w);

    /* This memory barrier is needed to keep us from reading
     * any other fields out of the rx_desc until we know the
     * descriptor has been written back
     */
    dma_rmb();

    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "poll addr virt: 0x%pK\n", engine->poll_mode_addr_virt);
    poll_val = (volatile u32 *)(engine->poll_mode_addr_virt);
    for (i = 0; i < sizeof(struct sgdma_poll_wb); i += 4, poll_val++) {
        TEST_DBG(curr_test_case, "poll addr[%d]: 0x%08x\n", i >> 2, *poll_val);
    }

    w = read_register(&engine->regs->poll_mode_wb_lo);
    TEST_DBG(curr_test_case, "poll_mode_wb_lo addr: 0x%08x\n", w);

    w = read_register(&engine->regs->poll_mode_wb_hi);
    TEST_DBG(curr_test_case, "poll_mode_wb_hi addr: 0x%08x\n", w);

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, curr_test_case->data_virt, 0x100, true);

    //snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, data_virt + data_size - 1 - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);

    /* stop engine. */
    write_register(0x0, &engine->regs->control,
               (unsigned long)(&engine->regs->control) -
                   (unsigned long)(engine->regs));

    al_pci_test_free_desc(curr_test_case);

    TEST_RET(curr_test_case, true, "\n");
err_out:
    TEST_RET(curr_test_case, false, "\n");
}

/**
 * test sgdma c2h stream mode write back
 */
C2H_TEST_CASE("sgdma submit desc test", al_sgdma_test77)
{
#define TEST_PAGES_NUM    10

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test77;
    ssize_t comp_size;
    char *data_virt;
    char prefix[64];
    bool ret = true;
    int i, rv, num;
    struct sg_table test_sgt;
    struct scatterlist *sg;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;

    /* step1: alloc sgt */
    rv = sgt_alloc_with_pages(&test_sgt, TEST_PAGES_NUM, engine->dir, sgdev->pdev);
    if (rv < 0) {
        TEST_DBG(curr_test_case, "%s cyclic pages %u OOM, ret %d\n",
            engine->name, TEST_PAGES_NUM, rv);
        ret = false;
        goto out1;
    }

    sg = test_sgt.sgl;

    /* step2: submit transfer */
    comp_size = sgdma_xfer_submit(sgdev, engine->channel, false, 0, &test_sgt, true, 5 * 1000);
    if (comp_size < 0) {
        TEST_DBG(curr_test_case, "%s xfer submit failed, ret %zd.\n",
            engine->name, comp_size);
        ret = false;
        goto out2;
    }

    TEST_DBG(curr_test_case, "sgdma_xfer_submit result: %zd(0x%zx)\n", comp_size, comp_size);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 16, 0) /* XXX FIX ME */
    fp = filp_open("/home/free/pcie_test/AL_PCI_TEST_KERNEL_FILE", O_RDWR | O_CREAT,0644);

    if (IS_ERR(fp)){
        printk("create file error/n");
        return -1;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);

    pos =0;

    for (i = 0; i < test_sgt.orig_nents; i++, sg = sg_next(sg)) {
        snprintf(prefix, 64, "[%s sg %d]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, page_address(sg_page(sg)), 0x100, true);
        vfs_write(fp, page_address(sg_page(sg)), PAGE_SIZE, &pos);
        msleep(500);
    }
    filp_close(fp,NULL);

    set_fs(fs);
#else
    for (i = 0; i < test_sgt.orig_nents; i++, sg = sg_next(sg)) {
        snprintf(prefix, 64, "[%s sg %d]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, page_address(sg_page(sg)), 0x100, true);
    }
#endif

out2:
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    TEST_RET(curr_test_case, ret, "\n");
}


/**
 * test random completed flag desc list interrupt
 */
C2H_TEST_CASE("sgdma c2h random completed flag desc list interrupt", al_sgdma_test9)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0X1000
#define TEST_DESC_NUM       30
#define TEST_DATA_DUMP_SIZE 0X100

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    struct al_test_case *curr_test_case = &al_sgdma_test9;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, next_adj, w;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64], desc_str[256];
    bool ret = true;
    int i, rv, num, count = TEST_DESC_NUM;
    int random_index[] = {2, 8, 16, 27, 28};

    /* step1: build desc */
    rv = al_pci_test_build_desc(curr_test_case, TEST_DESC_NUM, TEST_DATA_SIZE, 1);
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

    /* step2: set random desc comp flag  */
    for (i = 0; i < ARRAY_SIZE(random_index); i++) {
        if (random_index[i] < TEST_DESC_NUM - 1) {
            sgdma_desc_control_set(&desc_virt[random_index[i]], SGDMA_DESC_COMPLETED);
        }
    }

    /* step3: run engine do transfer */
    al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;

    if (wb_data->completed_desc_count & WB_ERR_MASK) {
        ret = false;
    }
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %lu, err status: 0x%lx\n",
        wb_data->completed_desc_count & WB_COUNT_MASK, wb_data->completed_desc_count & WB_ERR_MASK);

    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }

    data_virt = curr_test_case->data_virt;
    for (i = 0; i < TEST_DESC_NUM; i++) {
        
        snprintf(prefix, 64, "[%s desc%d start]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt, min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
    
        snprintf(prefix, 64, "[%s desc%d   end]", curr_test_case->alias, i);
        print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8,
            data_virt + TEST_DATA_SIZE - min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length),
            min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);
        
        printk("\n");
        data_virt += TEST_DATA_SIZE;
    }

    al_pci_test_free_desc(curr_test_case);
out1:
    TEST_RET(curr_test_case, ret, "\n");
}

/**
 * test custom msi interrupt
 */
C2H_TEST_CASE("sgdma custom msi interrupt test", al_sgdma_test10)
{
#define BAR_MSI_INTERRUPT_OFFSET (0x20a8)

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct al_test_case *curr_test_case = &al_sgdma_test10;

    channel_interrupts_enable(sgdev, ~0);

    iowrite32(0x0, sgdev->bar[0] + BAR_MSI_INTERRUPT_OFFSET);
    iowrite32(0x1, sgdev->bar[0] + BAR_MSI_INTERRUPT_OFFSET);
    mdelay(5);
    iowrite32(0x0, sgdev->bar[0] + BAR_MSI_INTERRUPT_OFFSET);

    channel_interrupts_disable(sgdev, ~0);

    TEST_DBG(curr_test_case, "please check dmesg msi interrupt log.\n");
    TEST_RET(curr_test_case, true, "\n");
}

/**
 * test sgdma c2h ring desc
 */
C2H_TEST_CASE("sgdma c2h ring desc test", al_sgdma_test11)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      4096
#define TEST_DATA_DUMP_SIZE 512
#define TEST_READ_CNT       100
#define TEST_TX_PAGE_NUM    CYCLIC_RX_PAGES_MAX

    struct anlogic_dev *sgdev = c2h_test_engine->sgdev;
    struct anlogic_dma_engine *engine = c2h_test_engine;
    struct al_test_case *curr_test_case = &al_sgdma_test11;
    struct sgdma_common_regs *reg;
    char prefix[64], err_info[64] = {0};
    uint32_t w;
    uint8_t *buf;
    int count = TEST_DATA_SIZE, rv, read_bytes = 0, read_cnt = TEST_READ_CNT;
    bool ret = true;

    /* init credit mode */
    enable_st_c2h_credit = 1;

    w = (0x1 << engine->channel) << 16; /* c2h: bit16 ~ 19 */
    reg = (struct sgdma_common_regs
             *)(sgdev->bar[sgdev->config_bar_idx] +
                (0x6 * TARGET_SPACING));

    write_register(w, &reg->credit_mode_enable_w1s,
        (unsigned long)(&reg->credit_mode_enable_w1s) -
               (unsigned long)(reg));

    /* init credit num */
    TEST_DBG(curr_test_case, "write credits num %d.\n", TEST_TX_PAGE_NUM / 2);
    write_register(TEST_TX_PAGE_NUM / 2, /* half num */
                &engine->sgdma_regs->credits, 
           (unsigned long)(&engine->sgdma_regs->credits) -
               (unsigned long)(engine->sgdma_regs));

    buf = kzalloc(TEST_DATA_SIZE, GFP_KERNEL);
    if (buf == NULL) {
        ret = false;
        sprintf(err_info, "%s", "buf alloc failed.");
        goto out1;
    }

    rv = sgdma_cyclic_transfer_setup(engine);
    if (rv < 0 && rv != -EBUSY) {
        ret = false;
        sprintf(err_info, "cyclic setup failed, ret %d", rv);
        goto out2;
    }

    /* 5 sec. timeout */
    while (count > 0 && read_cnt--) {
        rv = sgdma_engine_read_cyclic(engine, &buf[read_bytes], count, 5000);
        if (rv < 0) {
            ret = false;
            sprintf(err_info, "cyclic expected read bytes %d, real read bytes %d, ret %d", TEST_DATA_SIZE, read_bytes, rv);
            break;
        }

        count -= rv;
        read_bytes += rv;
    }

    if (read_bytes <= TEST_DATA_SIZE && read_cnt == 0) {
        ret = false;
        sprintf(err_info, "cyclic expected read bytes %d, real read bytes %d at poll %d times", TEST_DATA_SIZE, read_bytes, TEST_READ_CNT); 
    }

    snprintf(prefix, 64, "[%s begin]", curr_test_case->alias);
    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, buf, TEST_DATA_SIZE, true);
    // snprintf(prefix, 64, "[%s   end]", curr_test_case->alias);
    // print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, buf + TEST_DATA_SIZE - TEST_DATA_DUMP_SIZE, TEST_DATA_DUMP_SIZE, true);

out2:
    kfree(buf);
out1:

    /* disable credit mode */
    w = (0x1 << engine->channel) << 16; /* c2h: bit16 ~ 19 */
    write_register(w, &reg->credit_mode_enable_w1c,
        (unsigned long)(&reg->credit_mode_enable_w1c) -
               (unsigned long)(reg));
 
    enable_st_c2h_credit = 0;

    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

void al_pci_c2h_case_init(struct al_tester *tester)
{
    struct anlogic_dev *sgdev = to_anlogic_sgdev(tester->dev);

    c2h_test_engine = &sgdev->engine_c2h[0];

    AL_PCIE_DBG_ADD(al_sgdma_test0);
    AL_PCIE_DBG_ADD(al_sgdma_test1);
    AL_PCIE_DBG_ADD(al_sgdma_test2);
    AL_PCIE_DBG_ADD(al_sgdma_test3);
    AL_PCIE_DBG_ADD(al_sgdma_test5);
    AL_PCIE_DBG_ADD(al_sgdma_test5_1);
    AL_PCIE_DBG_ADD(al_sgdma_test6);
    AL_PCIE_DBG_ADD(al_sgdma_test7_0);
    AL_PCIE_DBG_ADD(al_sgdma_test7_1);
    AL_PCIE_DBG_ADD(al_sgdma_test8_0);
    AL_PCIE_DBG_ADD(al_sgdma_test8_1);
    AL_PCIE_DBG_ADD(al_sgdma_test9);
    AL_PCIE_DBG_ADD(al_sgdma_test10);
    AL_PCIE_DBG_ADD(al_sgdma_test11);
}

