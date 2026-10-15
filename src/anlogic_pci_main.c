// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：模块与 PCI 驱动骨架。
 *
 * 只做 PCI 驱动的注册/探测/移除与错误处理，资源与寄存器访问在 anlogic_pci_hw.c，
 * 字符设备在 anlogic_pci_chrdev.c（注册）与 anlogic_pci_ctrl.c（文件操作）。
 */
#include <linux/aer.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "anlogic_pci.h"

#define ANLOGIC_PCI_VENDOR_ID   0x1edb
#define ANLOGIC_PCI_DEVICE_ID   0xabcd

static const struct pci_device_id anlogic_pci_tbl[] = {
    { PCI_DEVICE(ANLOGIC_PCI_VENDOR_ID, ANLOGIC_PCI_DEVICE_ID), },
    { }
};

MODULE_DEVICE_TABLE(pci, anlogic_pci_tbl);

static struct anlogic_pci_card *anlogic_pci_card_alloc(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card;

    card = kzalloc(sizeof(*card), GFP_KERNEL);
    if (!card)
        return NULL;

    card->magic = ANLOGIC_PCI_MAGIC_DEVICE;
    card->pdev = pdev;
    card->user_max = ANLOGIC_PCI_MAX_USER_IRQ;
    card->h2c_channel_max = ANLOGIC_PCI_MAX_CHANNELS;
    card->c2h_channel_max = ANLOGIC_PCI_MAX_CHANNELS;

    return card;
}

static void anlogic_pci_card_free(struct anlogic_pci_card *card)
{
    struct anlogic_pci_hw *hw = card->hw;

    pr_info("card 0x%p, destroy_interfaces, hw 0x%p.\n", card, hw);
    anlogic_pci_card_destroy_interfaces(card);
    card->hw = NULL;

    if (hw)
        anlogic_pci_hw_close(card->pdev, hw);

    kfree(card);
}

static int anlogic_pci_probe(struct pci_dev *pdev,
                             const struct pci_device_id *ent)
{
    struct anlogic_pci_card *card;
    void *hw;
    int rv = 0;

    card = anlogic_pci_card_alloc(pdev);
    if (!card)
        return -ENOMEM;

    pr_info("card->h2c_channel_max %d.\n", card->h2c_channel_max);
    pr_info("card->c2h_channel_max %d.\n", card->c2h_channel_max);

    hw = anlogic_pci_hw_open(ANLOGIC_PCI_DRV_NAME, pdev, &card->user_max,
                             &card->h2c_channel_max, &card->c2h_channel_max);
    if (!hw) {
        rv = -EINVAL;
        goto err_out;
    }
    card->hw = hw;

    rv = anlogic_pci_card_create_interfaces(card);
    if (rv)
        goto err_out;

    dev_set_drvdata(&pdev->dev, card);

    return 0;

err_out:
    pr_err("pdev 0x%p, err %d.\n", pdev, rv);
    anlogic_pci_card_free(card);
    return rv;
}

static void anlogic_pci_remove(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card;

    if (!pdev)
        return;

    card = dev_get_drvdata(&pdev->dev);
    if (!card)
        return;

    pr_info("pdev 0x%p, card 0x%p, hw 0x%p.\n", pdev, card, card->hw);

    anlogic_pci_card_free(card);

    dev_set_drvdata(&pdev->dev, NULL);
}

static pci_ers_result_t anlogic_pci_error_detected(struct pci_dev *pdev,
                                                   pci_channel_state_t state)
{
    struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

    switch (state) {
    case pci_channel_io_normal:
        return PCI_ERS_RESULT_CAN_RECOVER;
    case pci_channel_io_frozen:
        pr_warn("dev 0x%p,0x%p, frozen state error, reset controller\n",
                pdev, card);
        pci_disable_device(pdev);
        return PCI_ERS_RESULT_NEED_RESET;
    case pci_channel_io_perm_failure:
        pr_warn("dev 0x%p,0x%p, failure state error, req. disconnect\n",
                pdev, card);
        return PCI_ERS_RESULT_DISCONNECT;
    }

    return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t anlogic_pci_slot_reset(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

    pr_info("0x%p restart after slot reset\n", card);
    if (pci_enable_device_mem(pdev)) {
        pr_info("0x%p failed to re-enable after slot reset\n", card);
        return PCI_ERS_RESULT_DISCONNECT;
    }

    pci_set_master(pdev);
    pci_restore_state(pdev);
    pci_save_state(pdev);

    return PCI_ERS_RESULT_RECOVERED;
}

static void anlogic_pci_error_resume(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, card);
}

static void anlogic_pci_reset_prepare(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, card);
}

static void anlogic_pci_reset_done(struct pci_dev *pdev)
{
    struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

    pr_info("dev 0x%p,0x%p.\n", pdev, card);
}

static const struct pci_error_handlers anlogic_pci_err_handler = {
    .error_detected = anlogic_pci_error_detected,
    .slot_reset     = anlogic_pci_slot_reset,
    .resume         = anlogic_pci_error_resume,
    .reset_prepare  = anlogic_pci_reset_prepare,
    .reset_done     = anlogic_pci_reset_done,
};

static struct pci_driver anlogic_pci_driver = {
    .name        = ANLOGIC_PCI_DRV_NAME,
    .id_table    = anlogic_pci_tbl,
    .probe       = anlogic_pci_probe,
    .remove      = anlogic_pci_remove,
    .err_handler = &anlogic_pci_err_handler,
};

static int __init anlogic_pci_init_module(void)
{
    int rv;

    pr_info("Hello,Anlogic-pcie\n");

    rv = anlogic_pci_chrdev_init();
    if (rv < 0)
        return rv;

    if (pci_register_driver(&anlogic_pci_driver)) {
        pr_err("no found pcie\n");
        pci_unregister_driver(&anlogic_pci_driver);
        anlogic_pci_chrdev_cleanup();
        return -ENODEV;
    }

    return 0;
}

static void __exit anlogic_pci_cleanup_module(void)
{
    pci_unregister_driver(&anlogic_pci_driver);
    anlogic_pci_chrdev_cleanup();
    pr_info("Goodbye,Anlogic-pcie\n");
}

module_init(anlogic_pci_init_module);
module_exit(anlogic_pci_cleanup_module);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Anlogic PH1P100 PCIe endpoint driver (host/RC side, PIO register access)");
MODULE_AUTHOR("anlogic");
