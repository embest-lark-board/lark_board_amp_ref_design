/*
 * Copyright Altera Corporation (C) 2013. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/msi.h>
#include "pci-altera.h"

#define MSI_STATUS		0x0
#define MSI_ERROR		0x4
#define MSI_INTMASK		0x8

static DEFINE_SPINLOCK(list_lock);	/* protect msi_list */
static LIST_HEAD(msi_list);

static inline void msi_writel(struct altera_pci_msi *msi, u32 value, u32 reg)
{
	writel(value, msi->csr_base + reg);
}

static inline u32 msi_readl(struct altera_pci_msi *msi, u32 reg)
{
	return readl(msi->csr_base + reg);
}

static int altera_pci_msi_alloc(struct altera_pci_msi *msi)
{
	int bit;

	mutex_lock(&msi->lock);

	bit = find_first_zero_bit(msi->used, msi->num_of_vectors);
	if (bit < msi->num_of_vectors)
		set_bit(bit, msi->used);
	else
		bit = -ENOSPC;

	mutex_unlock(&msi->lock);

	return bit;
}

static void altera_pci_msi_irq_free(struct altera_pci_msi *msi,
	unsigned long irq)
{
	struct device *dev = &msi->pdev->dev;
	u32 mask;

	mutex_lock(&msi->lock);

	if (!test_bit(irq, msi->used))
		dev_err(dev, "trying to free unused MSI#%lu\n", irq);
	else {
		clear_bit(irq, msi->used);
		mask = msi_readl(msi, MSI_INTMASK);
		mask &= ~(1 << irq);
		msi_writel(msi, mask, MSI_INTMASK);
	}

	mutex_unlock(&msi->lock);
}

static irqreturn_t altera_pci_msi_isr(int irq, void *data)
{
	struct altera_pci_msi *msi = data;
	unsigned long status;
	u32 num_of_vectors = msi->num_of_vectors;
	u32 processed = 0;
	u32 offset;

	do {
		status = msi_readl(msi, MSI_STATUS);
		if (!status)
			break;

		do {
			offset = find_first_bit(&status, num_of_vectors);
			/* Dummy read from vector to clear the interrupt */
			readl(msi->vector_base + (offset * sizeof(u32)));

			irq = irq_find_mapping(msi->domain, offset);
			if (irq) {
				if (test_bit(offset, msi->used))
					generic_handle_irq(irq);
			}

			/* Clear the bit from status and repeat without reading
			 * again status register. */
			clear_bit(offset, &status);
			processed++;
		} while (status);
	} while (1);

	return processed > 0 ? IRQ_HANDLED : IRQ_NONE;
}

static struct irq_chip altera_pci_msi_irq_chip = {
	.name = "altera-pci-msi",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int altera_pci_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &altera_pci_msi_irq_chip,
		handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = altera_pci_msi_map,
};

int altera_pci_msi_probe(struct platform_device *pdev)
{
	struct altera_pci_msi *msi;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	msi = devm_kzalloc(&pdev->dev, sizeof(struct altera_pci_msi),
		GFP_KERNEL);
	if (!msi)
		return -ENOMEM;

	mutex_init(&msi->lock);
	msi->pdev = pdev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csr");
	msi->csr_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!msi->csr_base) {
		dev_err(&pdev->dev, "get csr resource failed\n");
		return -EADDRNOTAVAIL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"vector_slave");
	msi->vector_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!msi->vector_base) {
		dev_err(&pdev->dev, "get vector slave resource failed\n");
		return -EADDRNOTAVAIL;
	}

	msi->vector_phy = res->start;

	if (of_property_read_u32(np, "num-vectors", &msi->num_of_vectors)) {
		dev_err(&pdev->dev, "failed to parse the number of vectors\n");
		return -EINVAL;
	}

	msi->domain = irq_domain_add_linear(np, msi->num_of_vectors,
					    &msi_domain_ops, msi);
	if (!msi->domain) {
		dev_err(&pdev->dev, "failed to create IRQ domain\n");
		return -ENODEV;
	}

	msi->irq = platform_get_irq(pdev, 0);
	if (msi->irq <= 0) {
		dev_err(&pdev->dev, "failed to map IRQ: %d\n", msi->irq);
		ret = -ENODEV;
		goto err;
	}

	ret = devm_request_irq(&pdev->dev, msi->irq, altera_pci_msi_isr, 0,
				altera_pci_msi_irq_chip.name, msi);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ: %d\n", ret);
		goto err;
	}

	spin_lock(&list_lock);
	list_add_tail(&msi->list, &msi_list);
	spin_unlock(&list_lock);

	platform_set_drvdata(pdev, msi);
	return 0;

err:
	irq_domain_remove(msi->domain);
	return ret;
}

static int altera_pci_msi_remove(struct platform_device *pdev)
{
	struct altera_pci_msi *msi = platform_get_drvdata(pdev);

	msi_writel(msi, 0, MSI_INTMASK);

	if (msi->domain)
		irq_domain_remove(msi->domain);

	spin_lock(&list_lock);
	list_del(&msi->list);
	spin_unlock(&list_lock);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

struct altera_pci_msi *altera_pci_msi_request(struct device_node *msi_np)
{
	struct altera_pci_msi *msi;

	spin_lock(&list_lock);
	list_for_each_entry(msi, &msi_list, list) {
		if (msi_np == msi->pdev->dev.of_node) {
			if (!msi->requested) {
				msi->requested = true;
				spin_unlock(&list_lock);
				return msi;
			} else {
				pr_info("MSI device is in use.\n");
				spin_unlock(&list_lock);
				return NULL;
			}
		}
	}
	spin_unlock(&list_lock);
	pr_info("MSI device not found!\n");
	return NULL;
}

int altera_pci_msi_free(struct altera_pci_msi *msi)
{
	if (!msi || !msi->requested)
		return -EINVAL;

	spin_lock(&list_lock);
	msi->requested = false;
	spin_unlock(&list_lock);

	return 0;
}

/* Arch hooks */
int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	struct altera_pcie *pcie = sys_to_pcie(pdev->bus->sysdata);
	struct altera_pci_msi *msi = pcie->msi;
	struct msi_msg msg;
	u32 irq;
	u32 mask;
	int hwirq;

	hwirq = altera_pci_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_lo = msi->vector_phy + (hwirq * sizeof(u32));
	msg.address_hi = 0;		/* 32 bit address only */
	msg.data = hwirq;

	write_msi_msg(irq, &msg);

	mask = msi_readl(msi, MSI_INTMASK);
	mask |= 1 << hwirq;
	msi_writel(msi, mask, MSI_INTMASK);
	dev_dbg(&pdev->dev, "msi#%d virq %d address_lo 0x%x\n", hwirq, irq,
		msg.address_lo);
	return 0;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	struct msi_desc *entry = irq_get_msi_desc(irq);
	struct pci_dev *pdev = entry->dev;
	struct altera_pcie *pcie = sys_to_pcie(pdev->bus->sysdata);
	struct altera_pci_msi *msi = pcie->msi;
	struct irq_data *d = irq_get_irq_data(irq);

	altera_pci_msi_irq_free(msi, d->hwirq);
	irq_set_msi_desc(entry->irq, NULL);
	irq_dispose_mapping(entry->irq);
}

int arch_msi_check_device(struct pci_dev *pdev, int nvec, int type)
{
	/* Doesn't support MSIX and multiple MSI */
	if ((type == PCI_CAP_ID_MSIX) || (type == PCI_CAP_ID_MSI && nvec > 1))
		return 1;
	return 0;
}


static const struct of_device_id altera_pci_msi_of_match[] = {
	{ .compatible = "altr,msi-1.0", NULL },
	{ },
};
MODULE_DEVICE_TABLE(of, altera_pci_msi_of_match);

static struct platform_driver altera_pci_msi_driver = {
	.driver = {
		.name = "altera-pci-msi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(altera_pci_msi_of_match),
	},
	.probe = altera_pci_msi_probe,
	.remove = altera_pci_msi_remove,
};

static int __init altera_pci_msi_init(void)
{
	return platform_driver_register(&altera_pci_msi_driver);
}
arch_initcall_sync(altera_pci_msi_init);

MODULE_AUTHOR("Ley Foon Tan <lftan@altera.com>");
MODULE_DESCRIPTION("Altera PCIe MSI support");
MODULE_LICENSE("GPL v2");

