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
#ifndef PCI_ALTERA_H
#define PCI_ALTERA_H

#ifdef CONFIG_PCI_MSI

#define MAX_MSI_VECTORS		32
struct altera_pci_msi {
	DECLARE_BITMAP(used, MAX_MSI_VECTORS);
	struct mutex		lock;	/* proctect used variable */
	struct list_head	list;
	struct platform_device	*pdev;
	struct irq_domain	*domain;
	void __iomem		*csr_base;
	void __iomem		*vector_base;
	u32			vector_phy;
	u32			num_of_vectors;
	int			irq;
	bool			requested;
};

struct altera_pci_msi *altera_pci_msi_request(struct device_node *msi_np);
int altera_pci_msi_free(struct altera_pci_msi *msi);

#endif /* CONFIG_PCI_MSI */

struct altera_pcie {
	struct device		*dev;
	struct resource		*txs;
	struct resource		*table_res;
	struct altera_pci_msi	*msi;
	void __iomem		*cra_base;
	int			hwirq;
};

static inline struct altera_pcie *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

#endif /* PCI_ALTERA_H */

