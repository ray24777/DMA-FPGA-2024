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

#define AL_TEST_VERSION             "v1.0"
#define AL_TEST_COPYRIGHT_YEARS     "2022"
#define AL_TEST_NAME                "Anlogic Driver Tester"

#define AL_DBG_PRINT_SIM_DESC 0

static struct al_tester al_sgdma_tester;

int format_desc(char *buf, int len, struct sgdma_desc *desc)
{
    int offset = 0;

    memset(buf, 0, len);
    offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc->bytes)) << 32) | desc->control);
    offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc->src_addr_hi)) << 32) | desc->src_addr_lo);
    offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc->dst_addr_hi)) << 32) | desc->dst_addr_lo);
    offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc->next_hi)) << 32) | desc->next_lo);

    return offset;
}

void format_desc_to_sim(struct sgdma_desc *desc, int cnt, enum dma_data_direction dir)
{
#define AL_SIM_DST_ADDR_START     0x1c001000ULL
#define AL_SIM_SRC_ADDR_START     0x0ULL
#define AL_SIM_DESC_ADDR_START    0x18000000ULL
#define AL_DESC_STR_LEN           256

    struct sgdma_desc desc_sim;
    int offset = 0, i, len = AL_DESC_STR_LEN;
    char buf[AL_DESC_STR_LEN];
    uint64_t src_addr, dst_addr, next_addr;

    if (dir == DMA_FROM_DEVICE) {
        src_addr = AL_SIM_SRC_ADDR_START;
        dst_addr = AL_SIM_DST_ADDR_START;
    } else {
        src_addr = AL_SIM_DST_ADDR_START;
        dst_addr = AL_SIM_SRC_ADDR_START;
    }

    next_addr = AL_SIM_DESC_ADDR_START + sizeof(struct sgdma_desc);

    for (i = 0; i < cnt; i++) {
        memcpy(&desc_sim, &desc[i], sizeof(struct sgdma_desc));
        memset(buf, 0, len);

        desc_sim.src_addr_hi = (src_addr & 0xffffffff00000000) >> 32;
        desc_sim.src_addr_lo = src_addr & 0xffffffff;
        desc_sim.dst_addr_hi = (dst_addr & 0xffffffff00000000) >> 32;
        desc_sim.dst_addr_lo = dst_addr & 0xffffffff;

        /* end do not fix. */
        if (i < cnt - 1) {
            desc_sim.next_hi = (next_addr & 0xffffffff00000000) >> 32;
            desc_sim.next_lo = next_addr & 0xffffffff;
        }

        offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc_sim.bytes)) << 32) | desc_sim.control);
        offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc_sim.src_addr_hi)) << 32) | desc_sim.src_addr_lo);
        offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc_sim.dst_addr_hi)) << 32) | desc_sim.dst_addr_lo);
        offset += snprintf(&buf[offset], len - offset, "%016llx\n", (((uint64_t)(desc_sim.next_hi)) << 32) | desc_sim.next_lo);

        printk("%s", buf);

        /* update next addr. */
        offset = 0;
        dst_addr += desc[i].bytes;
        next_addr += sizeof(struct sgdma_desc);
    }
}

void get_random_bytes(void *buf, int nbytes);

int al_random_num_gen(int min, int max)
{
    uint64_t num;

    get_random_bytes(&num, sizeof(num));

    return  min + (num % (max - min + 1));
}

#define ZERO_DATA_LEN 1

bool al_do_data_check(struct al_test_case *cur_case, void *buf, int len)
{
    uint64_t *pmem = buf;
    uint64_t except, i;

    for (i = 0, except = 0; i < ZERO_DATA_LEN * sizeof(uint64_t); i += sizeof(uint64_t), pmem++) {
        if (*pmem != 0) {
            TEST_DBG(cur_case, "recive: 0x%016llx expect: 0x%016llx, offset: 0x%016llx\n", *pmem, 0ULL, i);
            return false;
        }
    }

    for (except = 1; i < len; i += sizeof(uint64_t), except++, pmem++) {
        if (*pmem != ((except << 32) | except)) {
            TEST_DBG(cur_case, "recive: 0x%016llx expect: 0x%016llx, offset: 0x%016llx\n", *pmem, ((except << 32) | except), i);
            return false;
        }
    }

    return true;
}

bool al_do_desc_data_check(struct al_test_case *cur_case, struct sgdma_desc *desc, int cnt)
{
    int desc_idx = 0;
    uint64_t except = 0, recive, total_bytes = 0, i;
    uint64_t *dst_virt;

    dst_virt = __va(((uint64_t)desc[desc_idx].dst_addr_hi << 32) | desc[desc_idx].dst_addr_lo);
    total_bytes = (cur_case->is_stream) ? cur_case->res_virt[desc_idx].length : desc[desc_idx].bytes;

    if (total_bytes == 0) {
        return false;
    }

    for (i = 0, except = 0; i < ZERO_DATA_LEN * sizeof(uint64_t); i += sizeof(uint64_t), dst_virt++) {

        recive = *dst_virt;
        if (recive != 0) {
            TEST_DBG(cur_case, "recive: 0x%016llx expect: 0x%016llx, offset: 0x%016llx, desc: %d\n", recive, 0ULL, i, desc_idx);
            return false;
        }

        if (i >= total_bytes) {
            desc_idx++;

            if (desc_idx >= cnt) {
                return true;
            }

            dst_virt = __va(((uint64_t)desc[desc_idx].dst_addr_hi << 32) | desc[desc_idx].dst_addr_lo);
            total_bytes += (cur_case->is_stream) ? cur_case->res_virt[desc_idx].length : desc[desc_idx].bytes;
        }
    }

    except = 1;

    while (desc_idx < cnt) {

        recive = *dst_virt;

        if (recive != ((except << 32) | except)) {
            TEST_DBG(cur_case, "recive: 0x%016llx expect: 0x%016llx, offset: 0x%016llx, desc: %d\n",
                    recive, ((except << 32) | except), i, desc_idx);
            return false;
        }

        i += sizeof(uint64_t);
        except++;
        dst_virt++;

        if (i >= total_bytes) {
            desc_idx++;

            if (desc_idx >= cnt) {
                return true;
            }

            dst_virt = __va(((uint64_t)desc[desc_idx].dst_addr_hi << 32) | desc[desc_idx].dst_addr_lo);
            total_bytes += (cur_case->is_stream) ? cur_case->res_virt[desc_idx].length : desc[desc_idx].bytes;
        }
    }

    return true;
}

bool al_do_h2c_data_check(struct al_test_case *curr_test_case)
{
#define H2C_ERR_NUM_CNT_REG 0x80004

    struct anlogic_dev *sgdev = to_anlogic_sgdev(al_sgdma_tester.dev);
    uint32_t w;

    w = read_register(sgdev->bar[0] + H2C_ERR_NUM_CNT_REG);

    //al_dbg_printk("h2c err data count: %u\n", w);
    sprintf(curr_test_case->err_info, "h2c err data count: %u\n", w);

    return (w == 0) ? true : false;
}

inline void al_pci_test_desc_link(struct sgdma_desc *desc_virt, dma_addr_t desc_bus, int desc_num)
{
    int i;

    /* create singly-linked list for SG DMA controller */
    for (i = 0; i < desc_num - 1; i++) {
        /* increment bus address to next in array */
        desc_bus += sizeof(struct sgdma_desc);
    
        /* singly-linked list uses bus addresses */
        desc_virt[i].next_lo = cpu_to_le32(PCI_DMA_L(desc_bus));
        desc_virt[i].next_hi = cpu_to_le32(PCI_DMA_H(desc_bus));
        desc_virt[i].bytes = cpu_to_le32(0);
        desc_virt[i].control = cpu_to_le32(DESC_MAGIC);
    }
    /* { i = number - 1 } */
    /* zero the last descriptor next pointer */
    desc_virt[i].next_lo = cpu_to_le32(0);
    desc_virt[i].next_hi = cpu_to_le32(0);
    desc_virt[i].bytes = cpu_to_le32(0);
    desc_virt[i].control = cpu_to_le32(DESC_MAGIC);
}

inline void al_pci_test_fill_adj_num(struct sgdma_desc *desc_virt, int desc_num)
{
    int i, ret;

    /* step5: fill in adjacent numbers */
#ifdef AL_IP_VER0
    for (i = 0; i < desc_num; i++) {
        u32 next_adj = desc_num - i - 1;
        sgdma_desc_adjacent(desc_virt + i, next_adj);
    }
#else
    for (i = 0; i < desc_num; i++) {
        u32 next_adj = sgdma_get_next_adj(desc_num - i - 1,
                        (desc_virt + i)->next_lo);

        dbg_desc("set next adj at index %d to %u\n", i, next_adj);
        sgdma_desc_adjacent(desc_virt + i, next_adj);
    }
#endif
}

void al_pci_test_build_desc_no_alloc(struct al_test_case *test_case, int count, size_t single_size, int is_stream)
{
    struct anlogic_dev *sgdev = to_anlogic_sgdev(al_sgdma_tester.dev);
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    dma_addr_t data_bus, res_bus;
    void *data_virt;
    size_t data_size, res_size;
    uint32_t control = 0, next_adj;
    int i, ret;

    memset(test_case->desc_virt, 0, test_case->desc_size);
    memset(test_case->res_virt, 0, test_case->res_size);
    memset(test_case->data_virt, 0, test_case->data_size);

    res_virt = test_case->res_virt;
    res_bus = test_case->res_bus;
    res_size = test_case->res_size;
    data_virt = test_case->data_virt;
    data_bus = test_case->data_bus;
    data_size = test_case->data_size;
    desc_virt = test_case->desc_virt;

    TEST_DBG(test_case, "pci alloc resource success:\n"
        "desc virt 0x%pK bus 0x%016llx\n"
        "data virt 0x%pK bus 0x%016llx\n"
        "res  virt 0x%pK bus 0x%016llx\n",
        desc_virt, test_case->desc_bus,
        data_virt, data_bus,
        res_virt, res_bus);

    al_pci_test_desc_link(desc_virt, test_case->desc_bus, count);

    /* step4: config desc data addr and control filed */
    for (i = 0; i < count; i++) {
        sgdma_desc_set(&desc_virt[i], data_bus + single_size * i, 0, single_size, test_case->dir);
        sgdma_desc_control_clear(&desc_virt[i], LS_BYTE_MASK);
        //sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }
    /* stop engine, EOP for AXI ST, req IRQ on last descriptor */
    control = SGDMA_DESC_STOPPED;
    control |= SGDMA_DESC_EOP;
    control |= SGDMA_DESC_COMPLETED;
    sgdma_desc_control_set(&desc_virt[count - 1], control);

    if (is_stream && test_case->dir == DMA_FROM_DEVICE) {
        /* replace source addresses with result write-back addresses */
        for (i = 0; i < count; i++) {
            desc_virt[i].src_addr_lo = cpu_to_le32(PCI_DMA_L(res_bus));
            desc_virt[i].src_addr_hi = cpu_to_le32(PCI_DMA_H(res_bus));
            res_bus += sizeof(struct sgdma_result);
        }
    }

    al_pci_test_fill_adj_num(desc_virt, count);

    if (test_case->dir == DMA_TO_DEVICE)
        al_do_h2c_data_fill(test_case->data_virt, test_case->data_size, AL_H2C_DATA_WIDTH, 0);
}

int al_pci_test_build_desc(struct al_test_case *test_case, int count, size_t single_size, int is_stream)
{
    struct anlogic_dev *sgdev = to_anlogic_sgdev(al_sgdma_tester.dev);
    int ret;

    /* step1: alloc transfer desc */
    test_case->desc_size = sizeof(struct sgdma_desc) * count;
    test_case->desc_virt = pci_alloc_consistent(sgdev->pdev, test_case->desc_size, &test_case->desc_bus);
    if (test_case->desc_virt == NULL) {
        TEST_DBG(test_case, "pci desc alloc consistent failed.\n");
        return -ENOMEM;
    }
    memset(test_case->desc_virt, 0, test_case->desc_size);

    /* step2: alloc transfer data buf */
    test_case->data_size = single_size * count;
    test_case->data_virt = pci_alloc_consistent(sgdev->pdev, test_case->data_size, &test_case->data_bus);
    if (test_case->data_virt == NULL) {
        TEST_DBG(test_case, "pci data alloc consistent failed.\n");
        ret = false;
        goto out1;
    }
    memset(test_case->data_virt, 0, test_case->data_size);

    /* step3: alloc stream wb res */
    test_case->res_size = sizeof(struct sgdma_result) * count;
    test_case->res_virt = pci_alloc_consistent(sgdev->pdev, test_case->res_size, &test_case->res_bus);
    if (test_case->res_virt == NULL) {
        TEST_DBG(test_case, "pci result alloc consistent failed.\n");
        ret = false;
        goto out2;
    }
    memset(test_case->res_virt, 0, test_case->res_size);

    al_pci_test_build_desc_no_alloc(test_case, count, single_size, is_stream);

    return 0;
out2:
    pci_free_consistent(sgdev->pdev, test_case->desc_size, test_case->desc_virt, test_case->desc_bus);
out1:
    pci_free_consistent(sgdev->pdev, test_case->data_size, test_case->data_virt, test_case->data_bus);

    return -ENOMEM;
}

void al_pci_test_free_desc(struct al_test_case *test_case)
{
    struct anlogic_dev *sgdev = to_anlogic_sgdev(al_sgdma_tester.dev);

    pci_free_consistent(sgdev->pdev, test_case->desc_size, test_case->desc_virt, test_case->desc_bus);
    pci_free_consistent(sgdev->pdev, test_case->res_size, test_case->res_virt, test_case->res_bus);
    pci_free_consistent(sgdev->pdev, test_case->data_size, test_case->data_virt, test_case->data_bus);

    test_case->desc_virt = NULL;
    test_case->desc_bus = 0;
    test_case->desc_size = 0;

    test_case->res_virt = NULL;
    test_case->res_bus = 0;
    test_case->res_size = 0;

    test_case->data_virt = NULL;
    test_case->data_bus = 0;
    test_case->data_size = 0;
}

int al_pci_test_engine_do_transfer_with_no_stop(struct al_test_case *test_case, 
                                                    struct anlogic_dma_engine *engine, int count, int is_stop)
{
    int i, status, rv = 0;
    uint32_t next_adj, extra_adj;
    char desc_str[256];
    struct sgdma_poll_wb *wb_data;

    /* clear pool addr cnt */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    wb_data->completed_desc_count = 0;

    /* step4: calc adjacent numbers */
    for (i = 0; i < count; i++) {
        next_adj = sgdma_get_next_adj(count - i - 1,
                        (test_case->desc_virt + i)->next_lo);
    }

    /* step5: config sgdma regs */
    write_register(cpu_to_le32(PCI_DMA_L(test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_lo, 
            (unsigned long)(&engine->sgdma_regs->first_desc_lo) -
                (unsigned long)(engine->sgdma_regs));

    write_register(cpu_to_le32(PCI_DMA_H(test_case->desc_bus)),
            &engine->sgdma_regs->first_desc_hi, 
            (unsigned long)(&engine->sgdma_regs->first_desc_hi) -
                (unsigned long)(engine->sgdma_regs));

#ifdef AL_IP_VER0
    if (count > 2) {
        extra_adj = count - 2;
        if (extra_adj > MAX_EXTRA_ADJ) {
            extra_adj = MAX_EXTRA_ADJ;
        }
    } else {
        extra_adj = 0;
    }

    write_register(cpu_to_le32(extra_adj),
            &engine->sgdma_regs->first_desc_adjacent, 
            (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
                (unsigned long)(engine->sgdma_regs));
#else
    next_adj = sgdma_get_next_adj(count, cpu_to_le32(PCI_DMA_L(test_case->desc_bus)));
    write_register(cpu_to_le32(next_adj),
            &engine->sgdma_regs->first_desc_adjacent, 
            (unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
                (unsigned long)(engine->sgdma_regs));
#endif

    printk("%s %d %d\n", __FUNCTION__, __LINE__, count);
    /* dump desc */
    for (i = 0; i < count; i++) {
        dump_desc(&test_case->desc_virt[i]);
    }

#if AL_DBG_PRINT_SIM_DESC
    format_desc_to_sim(test_case->desc_virt, count, engine->dir);
#endif

    if ((engine->dir == DMA_FROM_DEVICE) ? enable_st_c2h_credit : enable_st_h2c_credit) {
        write_register(count,
                    &engine->sgdma_regs->credits, 
               (unsigned long)(&engine->sgdma_regs->credits) -
                   (unsigned long)(engine->sgdma_regs));
    }

    /* fill h2c data buf */
    if (test_case->dir == DMA_TO_DEVICE) {
        al_do_h2c_desc_data_fill(test_case->desc_virt, count);
        //al_do_h2c_data_fill(test_case->data_virt, test_case->data_size, AL_H2C_DATA_WIDTH, 0);
    }

    /* step6: start engine */
    engine_start_mode_config(engine);
    engine->running = 1;

    /* wait some times */
    msleep(5000);
    status = read_register(&engine->regs->status);
    TEST_DBG(test_case, "engine %s status = 0x%08x\n", engine->name, status);

    if (engine->dir == DMA_TO_DEVICE) {
        if (true != al_do_h2c_data_check(test_case)) {
            rv = -1;
        }
    }

    if (is_stop) {
        /* stop transfer */
        sgdma_engine_stop(engine);
        engine->running = 0;
    }

    return rv;
}

int al_pci_test_engine_do_transfer(struct al_test_case *test_case, 
                                                    struct anlogic_dma_engine *engine, int count)
{
    return al_pci_test_engine_do_transfer_with_no_stop(test_case, engine, count, 1);
}

bool al_stream_transfer_test(struct al_test_case *curr_test_case)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      0x1000
#define TEST_DESC_NUM       500
#define TEST_DATA_DUMP_SIZE 0x100

    struct anlogic_dev *sgdev = to_anlogic_sgdev(al_sgdma_tester.dev);
    struct anlogic_dma_engine *engine = 
        (curr_test_case->dir == DMA_FROM_DEVICE) ? &sgdev->engine_c2h[0] : &sgdev->engine_h2c[0];
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    dma_addr_t desc_bus, data_bus, res_bus;
    size_t data_size, res_size;
    uint32_t control = 0, status, next_adj, w;
    uint64_t ep_addr = 0;
    char *data_virt;
    char prefix[64];
    bool ret = true;
    int i, rv, num;

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

    /* add all desc comp flag */
    for (i = 0; i < TEST_DESC_NUM - 1; i++) {
        sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_COMPLETED);
    }

    /* step2: run engine do transfer */
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, TEST_DESC_NUM);
    if (rv < 0) {
        ret = false;
    }

    /* step3: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back addr completed cnt: %d\n", wb_data->completed_desc_count);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "reg completed desc cnt: %d\n", w);

    if (curr_test_case->dir == DMA_FROM_DEVICE) {
        TEST_DBG(curr_test_case, "hw stream write back result:\n");
        num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;

        ret = al_do_desc_data_check(curr_test_case, curr_test_case->desc_virt, TEST_DESC_NUM);

        data_virt = curr_test_case->data_virt;
        for (i = 0; i < 10; i++) {
            snprintf(prefix, 64, "[%s desc%d start]", curr_test_case->alias, i);

            print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt,
                min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);

            snprintf(prefix, 64, "[%s desc%d   end]", curr_test_case->alias, i);

            printk("\n");
            data_virt += TEST_DATA_SIZE;
        }

        data_virt = curr_test_case->data_virt + TEST_DATA_SIZE * (TEST_DESC_NUM - 10);
        for (i = TEST_DESC_NUM - 10; i < TEST_DESC_NUM - 1; i++) {
            snprintf(prefix, 64, "[%s desc%d end start]", curr_test_case->alias, i);

            print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 8, data_virt,
                min((u32)TEST_DATA_DUMP_SIZE, curr_test_case->res_virt[i].length), true);

            snprintf(prefix, 64, "[%s desc%d   end]", curr_test_case->alias, i);

            printk("\n");
            data_virt += TEST_DATA_SIZE;
        }
    }

    al_pci_test_free_desc(curr_test_case);
out1:
    return ret;
}

bool al_random_length_xfer(struct al_test_case *curr_test_case, struct anlogic_dma_engine *engine, bool set_eop)
{
#undef  TEST_DATA_SIZE
#undef  TEST_DESC_NUM
#undef  TEST_DATA_DUMP_SIZE
#define TEST_DATA_SIZE      256
#define TEST_DESC_NUM       (4096)
#define TEST_DATA_DUMP_SIZE 0x100
#define TEST_IS_STREAM_MODE 0
#define TEST_PAGES_NUM      2
#define DATA_ALIGN_BYTES    64
#define DATA_ALIGN_MASK     (DATA_ALIGN_BYTES - 1)

    struct anlogic_dev *sgdev = engine->sgdev;
    struct sgdma_poll_wb *wb_data;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
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
#endif

#if 0
    /* gen random data length */
    for (i = 0; i < ARRAY_SIZE(buf_size); i++) {
        buf_size[i] = al_random_num_gen(64, 4096);
    }

    count = (count < ARRAY_SIZE(buf_size)) ? count : ARRAY_SIZE(buf_size);
#else
    /* gen data length from 1 ~ 4096 */
    for (i = 0; i < TEST_DESC_NUM; i++) {
        buf_size[i] = i + 1;
    }

    count = TEST_DESC_NUM;
#endif

#if 0
    /* step1: build 64bit addr desc */
    rv = sgt_alloc_with_pages(&test_sgt, TEST_PAGES_NUM, engine->dir, sgdev->pdev);
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
    count = TEST_DESC_NUM;
    
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
        //buf_size[i] &= ~DATA_ALIGN_MASK;
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

        if (set_eop)
            sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_EOP); /* set eop for every desc. */

        data_bus += buf_size[i];
    }
#else
    for (sg = test_sgt.sgl, i = 0; i < count; i++, sg = sg_next(sg)) {
        sgdma_desc_set(&desc_virt[i], sg_dma_address(sg), 0, buf_size[i], curr_test_case->dir);
        sgdma_desc_control_clear(&desc_virt[i], LS_BYTE_MASK);
        
        if (set_eop)
            sgdma_desc_control_set(&desc_virt[i], SGDMA_DESC_EOP); /* set eop for every desc. */
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
    rv = al_pci_test_engine_do_transfer(curr_test_case, engine, count);
    if (rv < 0) {
        ret = false;
        goto out3;
    }

    /* step4: check write back data */
    wb_data = (struct sgdma_poll_wb *)engine->poll_mode_addr_virt;
    TEST_DBG(curr_test_case, "hw write back completed cnt: %d\n", wb_data->completed_desc_count);

    /* check completed result. */
    w = read_register(&engine->regs->completed_desc_count);
    TEST_DBG(curr_test_case, "register comp desc cnt: %d\n", w);

#if 0
    TEST_DBG(curr_test_case, "hw stream write back result:\n");
    num = (wb_data->completed_desc_count < TEST_DESC_NUM) ? wb_data->completed_desc_count : TEST_DESC_NUM;
    for (i = 0; i < num; i++) {
        TEST_DBG(curr_test_case, "result[%d]: 0x%08x, length[%d]\n",
            i, res_virt[i].status, res_virt[i].length);
    }
#endif

    if (wb_data->completed_desc_count != TEST_DESC_NUM || w != TEST_DESC_NUM) {
        ret = false;
        sprintf(err_info, "data write back count check failed, dst %d, real wb %d, reg %d", TEST_DESC_NUM, wb_data->completed_desc_count, w);
    }

out3:
    //pci_free_consistent(sgdev->pdev, curr_test_case->data_size, curr_test_case->data_virt, curr_test_case->data_bus);
    sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
out1:
    //sgt_free_with_pages(&test_sgt, engine->dir, sgdev->pdev);
    kfree(buf_size);
    pci_free_consistent(sgdev->pdev, count * sizeof(struct sgdma_desc), curr_test_case->desc_virt, curr_test_case->desc_bus);
    pci_free_consistent(sgdev->pdev, count * sizeof(struct sgdma_result), curr_test_case->res_virt, curr_test_case->res_bus);
out2:
    TEST_RET(curr_test_case, ret, "%s\n", err_info);
}

/* fill increasing data for h2c test */
int al_do_h2c_data_fill(void *buf, int size, int width, int start)
{
    uint64_t i, dat;
    int pos;
    int bytes = DIV_ROUND_UP(width, 8);
    int cnt = DIV_ROUND_UP(size, bytes);
    uint8_t *pbuf = buf;
    char *prefix = "fill data";

    for (i = 0, pos = 0; i < cnt; i++, pos += bytes) {
        dat = cpu_to_le64(i + start);
        memcpy(&pbuf[pos], &dat, bytes);
    }

//    printk("data fill buf phy addr: 0x%lx\n", __pa(buf));
//    print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, bytes, buf, size, true);
    //print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, bytes, buf, 0x100, true);

    return i;
}

void al_do_h2c_desc_data_fill(struct sgdma_desc *desc, int cnt)
{
#undef DATA_ALIGN_BYTES
#define DATA_ALIGN_BYTES     8//64

    uint64_t i, dat, idx = 0;
    int pos;
    int bytes = DIV_ROUND_UP(DATA_ALIGN_BYTES, 8);
    uint8_t *pbuf;

    pbuf = (uint8_t *)phys_to_virt((((uint64_t)desc[idx].src_addr_hi) << 32) | desc[idx].src_addr_lo);

    for (i = 0, pos = 0; idx < cnt; i++, pos += bytes) {

        if (pos >= desc[idx].bytes) {
            //printk("data fill buf phy addr: 0x%lx\n", __pa(pbuf));
            //print_hex_dump(KERN_NOTICE, "fill data ", DUMP_PREFIX_OFFSET, 16, bytes, pbuf, desc[idx].bytes, true);
            //print_hex_dump(KERN_NOTICE, "fill data ", DUMP_PREFIX_OFFSET, 16, bytes, pbuf, 64, true);
            pos = 0;
            idx++;

            if (idx >= cnt) {
                break;
            }

            pbuf = (uint8_t *)phys_to_virt((((uint64_t)desc[idx].src_addr_hi) << 32) | desc[idx].src_addr_lo);
        }

        dat = cpu_to_le64(i);
        memcpy(&pbuf[pos], &dat, bytes);
    }
}

int al_pci_test_main(struct device *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    al_sgdma_tester.dev = dev;

    INIT_LIST_HEAD(&al_sgdma_tester.case_list);

    al_pci_c2h_case_init(&al_sgdma_tester);

    al_pci_h2c_case_init(&al_sgdma_tester);

    return 0;
}

void al_pci_test_case_show(void)
{
    struct al_test_case *cur_case;
    int i = 0;

    if (al_sgdma_tester.dev == NULL) {
        al_dbg_printk("please init first.\n");
        return;
    }

    al_dbg_printk("\nselect a case to exec. (after case exec, you need to reinstall driver before use user app.)\n\n");
    list_for_each_entry(cur_case, &al_sgdma_tester.case_list, node) {
        if (cur_case) {
            al_dbg_printk("%3d - %s\n", i++, cur_case->desc);
        }
    }
    al_dbg_printk("\n");
}

int al_pci_test_exec(int index)
{
    struct al_test_case *cur_case;
    int i = 0;

    list_for_each_entry(cur_case, &al_sgdma_tester.case_list, node) {
        if (i++ == index) {
            if (cur_case && cur_case->ops) {
                cur_case->ops();
            } else {
                al_dbg_printk("%s ops is null\n", cur_case->desc);
            }
            break;
        }
    }

    return 0;
}

int al_pci_test_exec_all(void)
{
    struct al_test_case *cur_case;

    list_for_each_entry(cur_case, &al_sgdma_tester.case_list, node) {
        if (cur_case && cur_case->ops) {
            al_dbg_printk("exec %s\n", cur_case->desc);
            cur_case->ops();
        } else {
            al_dbg_printk("%s ops is null\n", cur_case->desc);
        }
    }

    return 0;
}

