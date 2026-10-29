// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：模块与 PCI 驱动骨架。
 *
 * 只做 PCI 驱动的注册/探测/移除与错误处理，资源与寄存器访问在 anlogic_pci_hw.c，
 * 字符设备在 anlogic_pci_chrdev.c（注册）与 anlogic_pci_ctrl.c（文件操作）。
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": [main] " fmt

#include <linux/aer.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "anlogic_pci.h"

#define ANLOGIC_PCI_VENDOR_ID 0x1edb
#define ANLOGIC_PCI_DEVICE_ID 0xabcd

static const struct pci_device_id anlogic_pci_tbl[] = {
	{
	PCI_DEVICE(ANLOGIC_PCI_VENDOR_ID, ANLOGIC_PCI_DEVICE_ID),
	},
	{}};

MODULE_DEVICE_TABLE(pci, anlogic_pci_tbl);

static struct anlogic_pci_card *anlogic_pci_card_alloc(struct pci_dev *pdev)
{
	struct anlogic_pci_card *card;

	card = kzalloc_obj(*card, GFP_KERNEL);
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

	pr_debug("card 0x%p, destroy_interfaces, hw 0x%p.\n", card, hw);
	anlogic_pci_card_destroy_interfaces(card);
	card->hw = NULL;

	if (hw)
		anlogic_pci_hw_close(card->pdev, hw);

	kfree(card);
}

static int anlogic_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct anlogic_pci_card *card;
	void *hw;
	int rv = 0;

	card = anlogic_pci_card_alloc(pdev);
	if (!card)
		return -ENOMEM;

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

	/*
	 * 先置 removed 并等所有 fd/mmap 引用归零，再拆节点与释放缓冲。
	 * PCI 层的 remove（/sys/.../remove、热拔插、AER 致命恢复）不会被
	 * cdev->owner 挡住，不等引用归零的话，进程手上还开着的 fd 或 frame 节点的
	 * VMA 就会在 hw 与 dma 缓冲被释放之后继续被访问（UAF / 读到复用内存）。
	 */
	anlogic_pci_hw_quiesce(card->hw);

	anlogic_pci_card_free(card);

	dev_set_drvdata(&pdev->dev, NULL);
}

/*
 * AER/FLR 之后 FPGA 里的 CTRL、RKA1 session、ARM 都可能已被复位；驱动侧的
 * link_armed 若继续为 1，link_ctl 会误报“已初始化”，ARM/PUSHCFG 也会基于
 * 旧状态。这里只清软件状态，不做 MMIO（frozen/reset 上下文里未必可访问）。
 */
static void anlogic_pci_mark_link_down(struct anlogic_pci_card *card, const char *why)
{
	if (!card || !card->hw || !card->hw->link_armed)
		return;

	card->hw->link_armed = 0;
	card->hw->link_cam_b = 0;
	dev_warn(&card->pdev->dev,
		 "%s, link marked down; re-init required (link_ctl push/retrig or reload)\n", why);
}

static pci_ers_result_t anlogic_pci_error_detected(struct pci_dev *pdev, pci_channel_state_t state)
{
	struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

	switch (state) {
	case pci_channel_io_normal:
		return PCI_ERS_RESULT_CAN_RECOVER;
	case pci_channel_io_frozen:
		pr_warn("dev 0x%p,0x%p, frozen state error, reset controller\n", pdev, card);
		anlogic_pci_mark_link_down(card, "frozen");
		pci_disable_device(pdev);
		/* 与 anlogic_pci_hw_close() 的 enabled 记账保持一致，避免重复 disable */
		if (card && card->hw)
			card->hw->enabled = 0;
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		pr_warn("dev 0x%p,0x%p, failure state error, req. disconnect\n", pdev, card);
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
	if (card && card->hw)
		card->hw->enabled = 1;
	/* slot reset 会清掉 FPGA 的 RKA1/CTRL；软件状态同步清掉，避免假就绪 */
	anlogic_pci_mark_link_down(card, "slot reset");

	return PCI_ERS_RESULT_RECOVERED;
}

static void anlogic_pci_error_resume(struct pci_dev *pdev)
{
	struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, card);
	anlogic_pci_mark_link_down(card, "error resume");
}

static void anlogic_pci_reset_prepare(struct pci_dev *pdev)
{
	struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, card);
	anlogic_pci_mark_link_down(card, "reset prepare");
}

static void anlogic_pci_reset_done(struct pci_dev *pdev)
{
	struct anlogic_pci_card *card = dev_get_drvdata(&pdev->dev);

	pr_info("dev 0x%p,0x%p.\n", pdev, card);
	anlogic_pci_mark_link_down(card, "reset done");
}

static const struct pci_error_handlers anlogic_pci_err_handler = {
	.error_detected = anlogic_pci_error_detected,
	.slot_reset = anlogic_pci_slot_reset,
	.resume = anlogic_pci_error_resume,
	.reset_prepare = anlogic_pci_reset_prepare,
	.reset_done = anlogic_pci_reset_done,
};

static struct pci_driver anlogic_pci_driver = {
	.name = ANLOGIC_PCI_DRV_NAME,
	.id_table = anlogic_pci_tbl,
	.probe = anlogic_pci_probe,
	.remove = anlogic_pci_remove,
	.err_handler = &anlogic_pci_err_handler,
};

static int __init anlogic_pci_init_module(void)
{
	int rv;

	rv = anlogic_pci_chrdev_init();
	if (rv < 0)
		return rv;

	/* 没有匹配的 EP 不算错误：模块可以先进来，等设备（或重新枚举）再 probe */
	rv = pci_register_driver(&anlogic_pci_driver);
	if (rv < 0) {
		pr_err("pci_register_driver() failed, %d.\n", rv);
		anlogic_pci_chrdev_cleanup();
	}

	return rv;
}

static void __exit anlogic_pci_cleanup_module(void)
{
	pci_unregister_driver(&anlogic_pci_driver);
	anlogic_pci_chrdev_cleanup();
}

module_init(anlogic_pci_init_module);
module_exit(anlogic_pci_cleanup_module);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Anlogic PH1P100 PCIe EP driver: BAR PIO, frame, camera, RKA1");
MODULE_AUTHOR("anlogic");
