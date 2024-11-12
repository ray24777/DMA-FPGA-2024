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
#ifndef __ANLOGIC_PCI_TEST_H__
#define __ANLOGIC_PCI_TEST_H__

#include <linux/list.h>

#define TEST_CASE(_des, _name, _dir) \
    bool _name##_case (void); \
    static struct al_test_case _name = { \
        .desc = _des, \
        .alias = # _name, \
        .ops = _name##_case, \
        .dir = _dir, \
        .is_stream = true, \
    }; \
    bool _name##_case (void)

#define TEST_RET(case, res, fmt, args...) \
    do { \
        al_dbg_printk("\r[%s] %s exec %s\n", (case)->alias, (case)->desc, \
                ((res) == true) ? "success" : "failed"); \
        if (res == false) { \
            al_dbg_printk("[%s] error info: %s"fmt"\n", (case)->alias, (case)->err_info, ##args); \
        } \
        al_dbg_printk("---------------------------------------------------------\n\n"); \
        return (res); \
    } while(0)

#define TEST_DBG(case, fmt, args...) \
    do { \
        al_dbg_printk("[%s] "fmt, (case)->alias, ##args); \
        printk("[%s] "fmt, (case)->alias, ##args); \
    } while(0)

#define C2H_TEST_CASE(des, name) TEST_CASE(des, name, DMA_FROM_DEVICE)

#define H2C_TEST_CASE(des, name) TEST_CASE(des, name, DMA_TO_DEVICE)

#define AL_PCIE_DBG_ADD(test_case) list_add_tail(&test_case.node, &tester->case_list);

#define AL_H2C_DATA_WIDTH   (64)

#define to_anlogic_sgdev(dev) (((struct anlogic_pci_dev *)dev_get_drvdata(dev)) -> sgdev)

typedef bool (* al_test_case_t)(void);

struct al_test_case {
    const char desc[256];
    const char alias[32];
    char err_info[256];
    bool is_stream;
    void *priv;
    struct sgdma_desc *desc_virt;
    struct sgdma_result *res_virt;
    void *data_virt;
    dma_addr_t desc_bus;
    dma_addr_t data_bus;
    dma_addr_t res_bus;
    size_t desc_size;
    size_t data_size;
    size_t res_size;
    enum dma_data_direction dir;
    al_test_case_t ops;
    struct list_head node;
};

struct al_tester {
    struct device *dev;
    struct list_head case_list;
};

extern unsigned int enable_st_c2h_credit;
extern unsigned int enable_st_h2c_credit;

extern void channel_interrupts_enable(struct anlogic_dev *sgdev, u32 mask);

extern void channel_interrupts_disable(struct anlogic_dev *sgdev, u32 mask);

int format_desc(char *buf, int len, struct sgdma_desc *desc);

int al_pci_test_build_desc(struct al_test_case *test_case, int count, size_t single_size, int is_stream);

void al_pci_test_build_desc_no_alloc(struct al_test_case *test_case, int count, size_t single_size, int is_stream);

void al_pci_test_free_desc(struct al_test_case *test_case);

int al_pci_test_engine_do_transfer(struct al_test_case *test_case, struct anlogic_dma_engine *engine, int count);

int al_pci_test_engine_do_transfer_with_no_stop(struct al_test_case *test_case, 
                                                    struct anlogic_dma_engine *engine, int count, int is_stop);

bool al_stream_transfer_test(struct al_test_case *curr_test_case);

void al_pci_test_desc_link(struct sgdma_desc *desc_virt, dma_addr_t desc_bus, int desc_num);

int al_random_num_gen(int min, int max);

bool al_do_data_check(struct al_test_case *cur_case, void *buf, int len);

bool al_do_desc_data_check(struct al_test_case *cur_case, struct sgdma_desc *desc, int cnt);

bool al_random_length_xfer(struct al_test_case *curr_test_case, struct anlogic_dma_engine *engine, bool set_eop);

bool al_do_h2c_data_check(struct al_test_case *curr_test_case);

int al_do_h2c_data_fill(void *buf, int size, int width, int start);

void al_do_h2c_desc_data_fill(struct sgdma_desc *desc, int cnt);

void al_pci_test_fill_adj_num(struct sgdma_desc *desc_virt, int desc_num);

void al_pci_h2c_case_init(struct al_tester *tester);

void al_pci_c2h_case_init(struct al_tester *tester);

/* anlogic pci dbg use */

int al_pci_test_main(struct device *dev);

void al_pci_test_case_show(void);

int al_pci_test_exec(int index);

int al_pci_test_exec_all(void);

#endif /* ifndef __ANLOGIC_PCI_TEST_H__ */

