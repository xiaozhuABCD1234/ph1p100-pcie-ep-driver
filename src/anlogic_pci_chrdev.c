// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：字符设备注册层。
 *
 * 负责 class/char 区域的申请与释放、按类型创建/销毁
 * /dev/ANLOGIC-PCI<n>_control（控制节点）与 /dev/ANLOGIC-PCI<n>_user（用户 BAR 窗口）。
 * 文件操作实现在 anlogic_pci_ctrl.c。
 */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include "anlogic_pci.h"

static struct class *anlogic_pci_class;

static const char * const anlogic_pci_node_names[ANLOGIC_PCI_CDEV_TYPE_NUM] = {
    [ANLOGIC_PCI_CDEV_USER] = ANLOGIC_PCI_NODE_NAME "%d_user",
    [ANLOGIC_PCI_CDEV_CTRL] = ANLOGIC_PCI_NODE_NAME "%d_control",
};

static inline void anlogic_pci_card_flag_set(struct anlogic_pci_card *card,
                                             enum anlogic_pci_card_flags fbit)
{
    card->flags |= 1 << fbit;
}

static inline int anlogic_pci_card_flag_test(struct anlogic_pci_card *card,
                                             enum anlogic_pci_card_flags fbit)
{
    return card->flags & (1 << fbit);
}

/*
 * 每个入口都要做的自检：字符设备与其所属实例的魔数必须对得上
 * （这是访问规约之外的防御，避免野指针走进 BAR 访问路径）。
 */
int anlogic_pci_cdev_check(const char *fname, struct anlogic_pci_cdev *cdev)
{
    struct anlogic_pci_hw *hw;

    if (!cdev || cdev->magic != ANLOGIC_PCI_MAGIC_CHAR) {
        pr_err("%s, cdev 0x%p, magic 0x%lx.\n", fname, cdev,
               cdev ? cdev->magic : 0xFFFFFFFF);
        return -EINVAL;
    }

    hw = cdev->hw;
    if (!hw || hw->magic != ANLOGIC_PCI_MAGIC_DEVICE) {
        pr_err("%s, hw 0x%p, magic 0x%lx.\n", fname, hw,
               hw ? hw->magic : 0xFFFFFFFF);
        return -EINVAL;
    }

    return 0;
}

int anlogic_pci_cdev_open(struct inode *inode, struct file *file)
{
    struct anlogic_pci_cdev *cdev;

    /* 从 inode 找回包含它的字符设备结构 */
    cdev = container_of(inode->i_cdev, struct anlogic_pci_cdev, cdev);

    if (cdev->magic != ANLOGIC_PCI_MAGIC_CHAR) {
        pr_err("cdev 0x%p inode 0x%llx magic mismatch 0x%lx\n",
               cdev, (unsigned long long)inode->i_ino, cdev->magic);
        return -EINVAL;
    }
    file->private_data = cdev;

    return 0;
}

int anlogic_pci_cdev_close(struct inode *inode, struct file *file)
{
    struct anlogic_pci_hw *hw;
    struct anlogic_pci_cdev *cdev = file->private_data;

    if (!cdev) {
        pr_err("char device with inode 0x%llx cdev NULL\n",
               (unsigned long long)inode->i_ino);
        return -EINVAL;
    }
    if (cdev->magic != ANLOGIC_PCI_MAGIC_CHAR) {
        pr_err("cdev 0x%p magic mismatch 0x%lx\n", cdev, cdev->magic);
        return -EINVAL;
    }

    hw = cdev->hw;
    if (!hw) {
        pr_err("char device with inode 0x%llx hw NULL\n",
               (unsigned long long)inode->i_ino);
        return -EINVAL;
    }
    if (hw->magic != ANLOGIC_PCI_MAGIC_DEVICE) {
        pr_err("hw 0x%p magic mismatch 0x%lx\n", hw, hw->magic);
        return -EINVAL;
    }

    return 0;
}

/* 给 cdev 的 kobject 命名，名字与设备节点保持一致 */
static int anlogic_pci_cdev_set_name(struct anlogic_pci_cdev *cdev,
                                     enum anlogic_pci_cdev_type type)
{
    struct anlogic_pci_hw *hw = cdev->hw;
    int rv;

    rv = kobject_set_name(&cdev->cdev.kobj, anlogic_pci_node_names[type],
                          hw->idx);
    if (rv)
        pr_err("%s: type 0x%x, failed %d.\n", __func__, type, rv);

    return rv;
}

static int anlogic_pci_cdev_add_sys_device(struct anlogic_pci_cdev *cdev,
                                           enum anlogic_pci_cdev_type type)
{
    struct anlogic_pci_hw *hw = cdev->hw;

    cdev->sys_device = device_create(anlogic_pci_class, &hw->pdev->dev,
                                     cdev->cdevno, NULL,
                                     anlogic_pci_node_names[type], hw->idx);
    if (!cdev->sys_device) {
        pr_err("device_create(%s) failed\n", anlogic_pci_node_names[type]);
        return -1;
    }

    return 0;
}

static int anlogic_pci_cdev_destroy(struct anlogic_pci_cdev *cdev)
{
    if (!cdev) {
        pr_warn("cdev NULL.\n");
        return -EINVAL;
    }
    if (cdev->magic != ANLOGIC_PCI_MAGIC_CHAR) {
        pr_warn("cdev 0x%p magic mismatch 0x%lx\n", cdev, cdev->magic);
        return -EINVAL;
    }
    if (!cdev->hw) {
        pr_err("hw NULL\n");
        return -EINVAL;
    }
    if (!anlogic_pci_class) {
        pr_err("anlogic_pci_class NULL\n");
        return -EINVAL;
    }
    if (!cdev->sys_device) {
        pr_err("cdev sys_device NULL\n");
        return -EINVAL;
    }

    device_destroy(anlogic_pci_class, cdev->cdevno);
    cdev_del(&cdev->cdev);

    return 0;
}

/* 建一个节点：先申请 char 区域（整卡一次），再挂这个节点 */
static int anlogic_pci_cdev_create(struct anlogic_pci_card *card,
                                   struct anlogic_pci_cdev *cdev, int bar,
                                   enum anlogic_pci_cdev_type type)
{
    struct anlogic_pci_hw *hw = card->hw;
    dev_t dev;
    int rv;

    if (!card->major) {
        rv = alloc_chrdev_region(&dev, ANLOGIC_PCI_MINOR_BASE,
                                 ANLOGIC_PCI_MINOR_COUNT,
                                 ANLOGIC_PCI_NODE_NAME);
        if (rv) {
            pr_err("unable to allocate cdev region %d.\n", rv);
            return rv;
        }
        card->major = MAJOR(dev);
        pr_info("cdev region MAJOR: %d, MINOR: %d\n", card->major,
                MINOR(dev));
    }

    cdev->magic = ANLOGIC_PCI_MAGIC_CHAR;
    cdev->cdev.owner = THIS_MODULE;
    cdev->card = card;
    cdev->hw = hw;
    cdev->bar = bar;

    rv = anlogic_pci_cdev_set_name(cdev, type);
    if (rv < 0)
        return rv;

    /* 非 SG DMA 节点用 type 当 minor：0 = _user，1 = _control */
    switch (type) {
    case ANLOGIC_PCI_CDEV_USER:
    case ANLOGIC_PCI_CDEV_CTRL:
        anlogic_pci_ctrl_init(cdev);
        break;
    default:
        pr_info("type 0x%x NOT supported.\n", type);
        return -EINVAL;
    }
    cdev->cdevno = MKDEV(card->major, type);

    rv = cdev_add(&cdev->cdev, cdev->cdevno, 1);
    if (rv < 0) {
        pr_err("cdev_add failed %d, type 0x%x.\n", rv, type);
        goto unregister_region;
    }

    if (anlogic_pci_class) {
        rv = anlogic_pci_cdev_add_sys_device(cdev, type);
        if (rv < 0)
            goto del_cdev;
    }

    return 0;

del_cdev:
    cdev_del(&cdev->cdev);
unregister_region:
    unregister_chrdev_region(cdev->cdevno, ANLOGIC_PCI_MINOR_COUNT);
    return rv;
}

int anlogic_pci_card_create_interfaces(struct anlogic_pci_card *card)
{
    struct anlogic_pci_hw *hw = card->hw;
    int rv;

    /* 控制节点绑在寄存器窗（BAR0） */
    rv = anlogic_pci_cdev_create(card, &card->ctrl_cdev, hw->config_bar_idx,
                                 ANLOGIC_PCI_CDEV_CTRL);
    if (rv < 0) {
        pr_err("create_char(ctrl_cdev) failed\n");
        goto fail;
    }
    anlogic_pci_card_flag_set(card, ANLOGIC_PCI_CARD_CDEV_CTRL);

    /* 用户 BAR 窗口节点，只有探测到 user BAR 时才建 */
    if (hw->user_bar_idx >= 0) {
        rv = anlogic_pci_cdev_create(card, &card->user_cdev,
                                     hw->user_bar_idx,
                                     ANLOGIC_PCI_CDEV_USER);
        if (rv < 0) {
            pr_err("create_char(user_cdev) failed\n");
            goto fail;
        }
        anlogic_pci_card_flag_set(card, ANLOGIC_PCI_CARD_CDEV_USER);
    }

    return 0;

fail:
    anlogic_pci_card_destroy_interfaces(card);
    return -1;
}

void anlogic_pci_card_destroy_interfaces(struct anlogic_pci_card *card)
{
    int rv;

    if (anlogic_pci_card_flag_test(card, ANLOGIC_PCI_CARD_CDEV_CTRL)) {
        rv = anlogic_pci_cdev_destroy(&card->ctrl_cdev);
        if (rv < 0)
            pr_err("Failed to destroy cdev ctrl, error 0x%x\n", rv);
    }

    if (anlogic_pci_card_flag_test(card, ANLOGIC_PCI_CARD_CDEV_USER)) {
        rv = anlogic_pci_cdev_destroy(&card->user_cdev);
        if (rv < 0)
            pr_err("Failed to destroy cdev user, error 0x%x\n", rv);
    }

    if (card->major)
        unregister_chrdev_region(MKDEV(card->major, ANLOGIC_PCI_MINOR_BASE),
                                 ANLOGIC_PCI_MINOR_COUNT);
}

int anlogic_pci_chrdev_init(void)
{
    anlogic_pci_class = class_create(ANLOGIC_PCI_NODE_NAME);
    if (IS_ERR(anlogic_pci_class)) {
        pr_err(ANLOGIC_PCI_NODE_NAME ": failed to create class");
        return -1;
    }

    return 0;
}

void anlogic_pci_chrdev_cleanup(void)
{
    if (anlogic_pci_class) {
        class_destroy(anlogic_pci_class);
        anlogic_pci_class = NULL;
    }
}
