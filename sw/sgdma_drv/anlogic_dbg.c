#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/hwmon.h>
#include <linux/sched.h>
#include <linux/tty.h>
//#include <linux/sched/signal.h>

#include "anlogic_dbg.h"

#define AL_DBG_PRINTK_MAX 512

#define to_al_dbg_attr(attr) container_of(to_sensor_dev_attr(attr), struct al_dbg_attr, dbg_dev_attr)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 16, 0) /* XXX FIX ME */
long al_dbg_write(unsigned int fd, const char __user *buf, size_t count)
{
    long ret = -EBADF;
    struct file *file = NULL;
    mm_segment_t status;
    loff_t offset;
 
    if (!buf) {
        printk("Write buffer was empty.\n");
        return ret;
    }
 
    status = get_fs();
    set_fs(KERNEL_DS);
 
    file = fget(fd);
    if (file) {
        offset = file->f_pos;
        ret = vfs_write(file, buf, count, &offset);
        file->f_pos = offset;
        fput(file);
    }
 
    set_fs(status);
 
    return ret;
}
#endif

int al_dbg_printk(const char *fmt, ...)
{
    int ret = -EBADF;
    char printf_buf[AL_DBG_PRINTK_MAX];
    va_list args;
    struct tty_struct *cur_tty;

    if (!fmt) {
        printk("Format for custom_printk was empty.\n");
        return -EINVAL;
    }

    //return vprintk(fmt, args);

    memset(printf_buf, 0, AL_DBG_PRINTK_MAX);
    va_start(args, fmt);
    //ret = vsnprintf(printf_buf, AL_DBG_PRINTK_MAX, fmt, args);
    ret = vscnprintf(printf_buf, AL_DBG_PRINTK_MAX, fmt, args);
    va_end(args);

#if 0
    if (ret > 0) {
        cur_tty = current->signal->tty;
        if (cur_tty) {
            cur_tty->ops->write(cur_tty, printf_buf, ret);
        }
    }
#else
    if (ret > 0)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 16, 0) /* XXX FIX ME */
        ret = (int)al_dbg_write(0, printf_buf, (unsigned int)ret);
#else
        printk("%s", printf_buf);
#endif
    else
        printk("Something was wrong with format or snprintf.\n");
#endif

    return 0;
}

static ssize_t al_dbg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct al_dbg_attr *dbg_attr = to_al_dbg_attr(attr);
    char *argv[AL_DBG_ARGC_MAX] = {(char *)dbg_attr->name, "-h", NULL};

    if (dbg_attr->dbg_main) {
        dbg_attr->dbg_main(2, argv); /* show help info */
    }

    return 0;
}

static ssize_t al_dbg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
#define DELIM " "
    char *str = (char *)buf, *token, *argv[AL_DBG_ARGC_MAX] = {NULL};
    int argc = 0;
    struct al_dbg_attr *dbg_attr = to_al_dbg_attr(attr);

    argv[argc++] = (char *)dbg_attr->name;

    do {
        token = strsep(&str, DELIM);

        if (token == NULL) {
            break;
        }

        if (*token == '\0') {
            continue; /* skip continue delim */
        }

        argv[argc++] = token;
    }  while (argc < AL_DBG_ARGC_MAX);

    if (dbg_attr->dbg_main) {
        dbg_attr->dbg_main(argc, argv);
    }

    return len;
}

static struct device_attribute al_dbg_devattr_rw = {
    .show   = al_dbg_show,
    .store  = al_dbg_store,
};

static int al_dbg_attr_init(struct al_dbg_attr *node)
{
    struct device *dev = node->dev;

    node->group.attrs = devm_kcalloc(dev,
                     2, /* must keep last one is null */
                     sizeof(struct attribute *),
                     GFP_KERNEL);
    if (!node->group.attrs)
        return -ENOMEM;

    node->dbg_attr[0] = &node->dbg_dev_attr.dev_attr.attr;
    memcpy(&node->dbg_dev_attr.dev_attr, &al_dbg_devattr_rw, sizeof(struct device_attribute));

    /* Set attribute name as a label. */
    node->dbg_attr[0]->name = devm_kasprintf(dev, GFP_KERNEL, node->name);

    if (!node->dbg_attr[0]->name) {
        dev_err(dev, "Memory allocation failed for sysfs attribute.\n");
        return -ENOMEM;
    }

    node->dbg_dev_attr.dev_attr.attr.mode = 0644;
    node->dbg_dev_attr.dev_attr.attr.name = node->dbg_attr[0]->name;
    node->dbg_dev_attr.index = 0;
    sysfs_attr_init(&node->dbg_dev_attr.dev_attr.attr);

    node->group.attrs = node->dbg_attr;
    node->groups[0] = &node->group;
    node->groups[1] = NULL;

    node->hwmon = devm_hwmon_device_register_with_groups(dev,
                                 "anlogic_dbg",
                                  node,
                                  node->groups);
    if (IS_ERR(node->hwmon)) {
        dev_err(dev, "Failed to register hwmon device %ld\n",
            PTR_ERR(node->hwmon));
        return PTR_ERR(node->hwmon);
    }

    return 0;
}

int al_dbg_node_create(struct al_dbg_attr *node)
{
    if (node == NULL || node->dev == NULL) {
        return -EINVAL;
    }

    return al_dbg_attr_init(node);
}

void al_dbg_node_destory(struct al_dbg_attr *node)
{
    if (node == NULL) {
        return;
    }

    //if (node->hwmon)
    //    hwmon_device_unregister(node->hwmon);

    node->dbg_main = NULL;
}

