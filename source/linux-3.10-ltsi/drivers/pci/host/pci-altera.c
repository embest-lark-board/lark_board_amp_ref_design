/*
 * Copyright Altera Corporation (C) 2013. All rights reserved
 *
 * Bits taken from arch/arm/mach-tegra/pcie.c
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

#include "pci-altera.h"

/* Registers offset */
#define A2P_ADDR_MAP_LO0  0x1000
#define A2P_ADDR_MAP_HI0  0x1004
#define RP_TX_REG0        0x2000
#define RP_TX_REG1        0x2004
#define RP_TX_CNTRL       0x2008
#define RP_TX_EOP         0x2
#define RP_TX_SOP         0x1
#define RP_RXCPL_STATUS   0x2010
#define RP_RXCPL_EOP      0x2
#define RP_RXCPL_SOP      0x1
#define RP_RXCPL_REG0     0x2014
#define RP_RXCPL_REG1     0x2018
#define P2A_INT_STATUS    0x3060
#define P2A_INT_STS_ALL   0x0F
#define RPRX_CPL_REV      0x10
#define P2A_INT_ENABLE    0x3070
#define P2A_INT_ENA_ALL   0x0F

/* TLP CFG RD WR */
#define TLP_FMTTYPE_CFGRD0 0x04  /* Configuration Read  Type 0 */
#define TLP_FMTTYPE_CFGWR0 0x44  /* Configuration Write Type 0 */
#define TLP_FMTTYPE_CFGRD1 0x05  /* Configuration Read  Type 1 */
#define TLP_FMTTYPE_CFGWR1 0x45  /* Configuration Write Type 1 */
#define TLP_PAYLOAD_SIZE   0x01  /* TLP Data Payload Size      */
#define TLP_CFG_DW0(fmttype)            ((fmttype << 24) | TLP_PAYLOAD_SIZE)
#define TLP_CFG_DW1(reqid, tag)         ((reqid << 16) | (tag << 8) | 0xF)
#define TLP_READ_TAG       0x1D  /* statically define */
#define TLP_WRITE_TAG      0x10  /* statically define */
#define TLP_CFG_DW2(bus, devfn, offset) ((bus << 24) | (devfn << 16) | offset)

/* Address translation table entry size */
#define ATT_ENTRY_SIZE		8

#define SOP_LOOP	1
#define HEADER_LOOP	2
#define DETECTION_LOOP	3

#define DWORD_MASK	3

struct tlp_rp_regpair_t {
	u32 rp_ctrl;
	u32 rp_reg0;
	u32 rp_reg1;
};

/*
 * The Root Port is boot/initialized with bus 0 device 0
 * pribus use TYPE0 CFG, secbus use TYPE1 CFG, subbus use TYPE1 CFG
 */
#define ALTRPCIERP_DEVFN 0
static u8 rp_pribus;
static u8 rp_secbus;

static inline void cra_writel(struct altera_pcie *pcie, u32 value, u32 reg)
{
	writel(value, pcie->cra_base + reg);
}

static inline u32 cra_readl(struct altera_pcie *pcie, u32 reg)
{
	return readl(pcie->cra_base + reg);
}

/* TLP packet RX pair */
static void tlp_read_rp_rx(struct altera_pcie *pcie,
			   struct tlp_rp_regpair_t *tlp_rp_regdata)
{
	tlp_rp_regdata->rp_ctrl = cra_readl(pcie, RP_RXCPL_STATUS);
	tlp_rp_regdata->rp_reg0 = cra_readl(pcie, RP_RXCPL_REG0);
	tlp_rp_regdata->rp_reg1 = cra_readl(pcie, RP_RXCPL_REG1);
}

/* TLP packet TX pair */
static void tlp_write_rp_tx(struct altera_pcie *pcie,
			    struct tlp_rp_regpair_t *tlp_rp_regdata)
{
	cra_writel(pcie, tlp_rp_regdata->rp_reg0, RP_TX_REG0);
	cra_writel(pcie, tlp_rp_regdata->rp_reg1, RP_TX_REG1);
	cra_writel(pcie, tlp_rp_regdata->rp_ctrl, RP_TX_CNTRL);
}

/* TLP packet detect SOP and EOP */
static int tlp_read_packet(struct altera_pcie *pcie, u32 *value)
{
	int err = PCIBIOS_SUCCESSFUL;
	u8 loop;
	struct tlp_rp_regpair_t *tlp_rp_regdata;

	/* allocate tlp_rp_regdata */
	tlp_rp_regdata = kzalloc(sizeof(struct tlp_rp_regpair_t), GFP_KERNEL);
	if (!tlp_rp_regdata) {
		dev_err(pcie->dev, "out of memory !!!\n");
		err = -ENOMEM;
		goto ret_error;
	}

	/*
	 * Detect RP_RXCPL_SOP is not require,
	 * read till EOP to avoid completion packet misalignment.
	 * Detect RP_RXCPL_EOP with loop of
	 * TLP_PAYLOAD_SIZE + 1(SOP) + 2(header) + 3(detection loop)
	 */
	for (loop = (TLP_PAYLOAD_SIZE + SOP_LOOP + HEADER_LOOP +
		DETECTION_LOOP); loop > 0; loop--) {
		tlp_read_rp_rx(pcie, tlp_rp_regdata);
		/* detect EOP */
		if (tlp_rp_regdata->rp_ctrl & RP_RXCPL_EOP)
			break;
	}
	/* if no EOP detected */
	if (loop == 0) {
		err = -ENOENT;
		goto ret_freemem;
	}

	/* read data */
	if (value)
		*value = tlp_rp_regdata->rp_reg0;

ret_freemem:
	kfree(tlp_rp_regdata);
ret_error:
	return err;
}

/* pci_ops read using TLP packet receive */
static int tlp_cfg_dword_read(struct altera_pcie *pcie, u8 bus, u32 devfn,
			      int where, u32 *value)
{
	struct tlp_rp_regpair_t *tlp_rp_regdata;
	int ret;

	/* if not DWORD align */
	if (!IS_ALIGNED(where, sizeof(u32))) {
		dev_err(pcie->dev, "data not DWORD align\n");
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/*
	 * Implement the device number filtering at the lowest level layer.
	 * Per PCIe spec section 7.3.1 - Configuration Requests
	 * targeting the Bus Number associated with a Link specifying
	 * Device Number 0 are delivered to the device attached to the Link;
	 * Configuration Requests specifying all other Device Numbers (1-31)
	 * must be terminated by the Switch Downstream Port or the Root Port
	 * with an Unsupported Request Completion Status.
	 */
	if (((bus == rp_pribus) || (bus == rp_secbus)) &&
		(PCI_SLOT(devfn) > 0)) {
		*value = ~0UL;
		return PCIBIOS_SUCCESSFUL;
	}

	/* allocate tlp_rp_regdata */
	tlp_rp_regdata = kzalloc(sizeof(struct tlp_rp_regpair_t), GFP_KERNEL);
	if (!tlp_rp_regdata) {
		dev_err(pcie->dev, "out of memory !!!\n");
		return -ENOMEM;
	}

	/* sending TLP packet */
	if (bus == rp_pribus)
		tlp_rp_regdata->rp_reg0 = TLP_CFG_DW0(TLP_FMTTYPE_CFGRD0);
	else
		tlp_rp_regdata->rp_reg0 = TLP_CFG_DW0(TLP_FMTTYPE_CFGRD1);
	tlp_rp_regdata->rp_reg1 = TLP_CFG_DW1(
				  ((rp_pribus << 8) | ALTRPCIERP_DEVFN),
				  TLP_READ_TAG);
	tlp_rp_regdata->rp_ctrl = RP_TX_SOP;
	tlp_write_rp_tx(pcie, tlp_rp_regdata);
	tlp_rp_regdata->rp_reg0 = TLP_CFG_DW2(bus, devfn, where);
	tlp_rp_regdata->rp_reg1 = 0x0;
	tlp_rp_regdata->rp_ctrl = RP_TX_EOP;
	tlp_write_rp_tx(pcie, tlp_rp_regdata);

	/* read data from completion package */
	ret = tlp_read_packet(pcie, value);
	if (ret)
		*value = ~0UL;	/* return 0xFFFFFFFF if error */

	kfree(tlp_rp_regdata);
	return ret;
}

/* pci_ops write using TLP packet send */
static int tlp_cfg_dword_write(struct altera_pcie *pcie, u8 bus, u32 devfn,
			       int where, u32 value)
{
	struct tlp_rp_regpair_t *tlp_rp_regdata;

	/* if not DWORD align */
	if (!IS_ALIGNED(where, sizeof(u32))) {
		dev_err(pcie->dev, "data not DWORD align\n");
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/*
	 * Implement the device number filtering at the lowest level layer.
	 * Per PCIe spec section 7.3.1 - Configuration Requests
	 * targeting the Bus Number associated with a Link specifying
	 * Device Number 0 are delivered to the device attached to the Link;
	 * Configuration Requests specifying all other Device Numbers (1-31)
	 * must be terminated by the Switch Downstream Port or the Root Port
	 * with an Unsupported Request Completion Status.
	 */
	if (((bus == rp_pribus) || (bus == rp_secbus)) &&
		(PCI_SLOT(devfn) > 0))
		return PCIBIOS_SET_FAILED;

	/* allocate tlp_rp_regdata */
	tlp_rp_regdata = kzalloc(sizeof(struct tlp_rp_regpair_t), GFP_KERNEL);
	if (!tlp_rp_regdata) {
		dev_err(pcie->dev, "out of memory !!!\n");
		return -ENOMEM;
	}

	/* sending TLP packet */
	if (bus == rp_pribus)
		tlp_rp_regdata->rp_reg0 = TLP_CFG_DW0(TLP_FMTTYPE_CFGWR0);
	else
		tlp_rp_regdata->rp_reg0 = TLP_CFG_DW0(TLP_FMTTYPE_CFGWR1);
	tlp_rp_regdata->rp_reg1 = TLP_CFG_DW1(
				  ((rp_pribus << 8) | ALTRPCIERP_DEVFN),
				  TLP_WRITE_TAG);
	tlp_rp_regdata->rp_ctrl = RP_TX_SOP;
	tlp_write_rp_tx(pcie, tlp_rp_regdata);
	tlp_rp_regdata->rp_reg0 = TLP_CFG_DW2(bus, devfn, where);
	tlp_rp_regdata->rp_reg1 = value;
	tlp_rp_regdata->rp_ctrl = RP_TX_EOP;
	tlp_write_rp_tx(pcie, tlp_rp_regdata);

	/* TLP read response */
	tlp_read_packet(pcie, NULL);

	/* if change altrpcierp bus number, update variable... */
	if ((bus == rp_pribus) && (devfn == ALTRPCIERP_DEVFN) &&
	    (where == PCI_PRIMARY_BUS)) {
		rp_pribus = (u8)(value);
		rp_secbus = (u8)(value >> 8);
	}

	kfree(tlp_rp_regdata);
	return PCIBIOS_SUCCESSFUL;
}

/* pci_ops read */
static int altera_pcie_cfg_read(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 *value)
{
	struct altera_pcie *pcie = sys_to_pcie(bus->sysdata);
	int ret;

	/* if cross DWORD boundary */
	if (((where & DWORD_MASK) + size) > 4) {
		dev_err(pcie->dev, "Data cross DWORD boundary\n");
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/* TLP read */
	ret = tlp_cfg_dword_read(pcie, bus->number, devfn,
		(where & ~DWORD_MASK), value);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	*value = *value >> ((where & DWORD_MASK) << 3);  /* position */
	*value = *value & (~0UL >> ((4 - size) << 3));  /* mask */
	return PCIBIOS_SUCCESSFUL;
}

/* pci_ops write */
static int altera_pcie_cfg_write(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 value)
{
	struct altera_pcie *pcie = sys_to_pcie(bus->sysdata);
	u32 data32 = value;
	int ret;

	/* if cross DWORD boundary */
	if (((where & DWORD_MASK) + size) > 4) {
		dev_err(pcie->dev, "Data cross DWORD boundary\n");
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/* TLP read and merge DWORD */
	if (IS_ALIGNED(where, sizeof(u32)) || (size != sizeof(u32))) {
		ret = tlp_cfg_dword_read(pcie, bus->number, devfn,
			where & ~DWORD_MASK, &data32);
		if (ret)
			return ret;
		data32 = data32 & ~((~0UL >> ((4 - size) << 3))
				    << ((where & DWORD_MASK) << 3));
		data32 = data32 | (value << ((where & DWORD_MASK) << 3));
	}

	/* TLP write */
	return tlp_cfg_dword_write(pcie, bus->number, devfn,
		(where & ~DWORD_MASK), data32);
}

/* pci_ops */
static struct pci_ops altera_pcie_ops = {
	.read = altera_pcie_cfg_read,
	.write = altera_pcie_cfg_write,
};

/* hw_pci setup */
static int altera_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct altera_pcie *pcie = sys_to_pcie(sys);
	int err;
	u32  pages, i;
	u32 translation_size;
	u32 txs_size;
	struct resource *res;

	/* get translation table size */
	cra_writel(pcie, ~0UL, A2P_ADDR_MAP_LO0);
	translation_size = (~(cra_readl(pcie, A2P_ADDR_MAP_LO0)
		& 0xFFFFFFFC)) + 1;

	txs_size = resource_size(pcie->txs);
	/* get number of translation table pages */
	pages = txs_size / translation_size;

	/* allocate resource memory */
	res = kzalloc(sizeof(struct resource) * pages, GFP_KERNEL);
	if (!res) {
		dev_err(pcie->dev, "out of memory !!!\n");
		err = -ENOMEM;
		goto ret_error;
	}
	pcie->table_res = res;
	/* IORESOURCE_MEM use A2P_ADDR_MAP_LO0 */
	if (pages >= 1) {
		res[0].name  = "ALTERA PCIE RP MEM";
		res[0].start = pcie->txs->start;
		res[0].end   = res[0].start + translation_size - 1;
		res[0].flags = IORESOURCE_MEM;
		if (request_resource(&iomem_resource, &res[0])) {
			dev_err(pcie->dev, "out of memory !!!\n");
			err = -ENOMEM;
			goto ret_freeres;
		}
		pci_add_resource_offset(&sys->resources, &res[0],
					sys->mem_offset);
	}

	/* IORESOURCE_MEM | IORESOURCE_PREFETCH */
	if (pages >= 2) {
		res[1].name  = "ALTERA PCIE RP PREF MEM";
		res[1].start = res[0].end + 1;
		res[1].end   = res[1].start + translation_size - 1;
		res[1].flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;
		if (request_resource(&iomem_resource, &res[1])) {
			dev_err(pcie->dev, "out of memory !!!\n");
			err = -ENOMEM;
			goto ret_relres;
		}
		pci_add_resource_offset(&sys->resources, &res[1],
					sys->mem_offset);
	}

	/* Configure Avalon-MM-to-PCI Express Address Translation Table */
	for (i = 0; i < pages; i++) {
		cra_writel(pcie, res[i].start,
			A2P_ADDR_MAP_LO0 + (ATT_ENTRY_SIZE * i));
		cra_writel(pcie, 0, A2P_ADDR_MAP_HI0 + (ATT_ENTRY_SIZE * i));
	}

	return 1;

ret_relres:
	release_resource(res);
ret_freeres:
	kfree(res);
ret_error:
	return err;
}

/* hw_pci map_irq */
static int altera_pcie_map_irq(const struct pci_dev *pdev, u8 slot, u8 pin)
{
	struct altera_pcie *pcie = sys_to_pcie(pdev->bus->sysdata);
	return pcie->hwirq;
}

/* hw_pci */
static struct hw_pci altera_pcie_hw __initdata = {
#ifdef CONFIG_PCI_DOMAINS
	.domain			= 0,
#endif
	.nr_controllers		= 1,
	.ops			= &altera_pcie_ops,
	.setup			= altera_pcie_setup,
	.map_irq		= altera_pcie_map_irq,
};

/* RP ISR - clear status */
static irqreturn_t altera_pcie_isr(int irq, void *arg)
{
	struct altera_pcie *pcie = arg;
	cra_writel(pcie, P2A_INT_STS_ALL, P2A_INT_STATUS);
	return IRQ_HANDLED;
}

/* RP device init and resources, based on Device Tree */
static int altera_pcie_probe(struct platform_device *pdev)
{
	struct altera_pcie *pcie;
	struct resource *cra;
	int ret;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		dev_err(&pdev->dev, "no memory for altera pcie\n");
		return -ENOMEM;
	}
	pcie->dev = &pdev->dev;

	cra = platform_get_resource_byname(pdev, IORESOURCE_MEM, "Cra");
	pcie->cra_base = devm_request_and_ioremap(&pdev->dev, cra);
	if (!pcie->cra_base) {
		dev_err(&pdev->dev, "get Cra resource failed\n");
		return -EADDRNOTAVAIL;
	}

	pcie->txs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "Txs");
	if (!pcie->txs) {
		dev_err(&pdev->dev, "get Txs resource failed\n");
		return PTR_ERR(pcie->txs);
	}

	/* setup IRQ */
	pcie->hwirq = platform_get_irq(pdev, 0);
	if (pcie->hwirq <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ: %d\n", pcie->hwirq);
		return -EINVAL;
	}
	ret = devm_request_irq(&pdev->dev, pcie->hwirq, altera_pcie_isr,
			IRQF_SHARED, pdev->name, pcie);

	if (ret) {
		dev_err(&pdev->dev, "failed to register legacy IRQ\n");
		return ret;
	}

#ifdef CONFIG_PCI_ALTERA_MSI
	{
		struct device_node *np_msi;
		np_msi = of_parse_phandle(pdev->dev.of_node, "altr,msi", 0);
		if (!np_msi)
			return -EINVAL;

		pcie->msi = altera_pci_msi_request(np_msi);
		if (!pcie->msi) {
			dev_err(&pdev->dev, "failed to request MSI support\n");
			return -EINVAL;
		}
	}
#endif
	/* clear PCIe to AvMM IRQ status */
	cra_writel(pcie, P2A_INT_STS_ALL, P2A_INT_STATUS);
	/* enable PCIe to AvMM IRQ */
	cra_writel(pcie, P2A_INT_ENA_ALL, P2A_INT_ENABLE);

	altera_pcie_hw.private_data = (void **)&pcie;
	pci_common_init(&altera_pcie_hw);

	platform_set_drvdata(pdev, pcie);
	return 0;
}

static int __exit altera_pcie_remove(struct platform_device *pdev)
{
	struct altera_pcie *pcie = platform_get_drvdata(pdev);
#ifdef CONFIG_PCI_MSI
	altera_pci_msi_free(pcie->msi);
#endif
	free_irq(pcie->hwirq, pcie);
	release_resource(pcie->table_res);
	kfree(pcie->table_res);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id altera_pcie_of_match[] = {
	{ .compatible = "ALTR,root-port-13.0", },
	{ .compatible = "ALTR,pcie-root-port-13.0", },
	{ .compatible = "altr,pcie-root-port-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, altera_pcie_of_match);

static struct platform_driver altera_pcie_driver = {
	.probe		= altera_pcie_probe,
	.remove		= altera_pcie_remove,
	.driver = {
		.name	= "altera-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(altera_pcie_of_match),
	},
};

static int __init altera_pcie_init(void)
{
	return platform_driver_register(&altera_pcie_driver);
}
subsys_initcall(altera_pcie_init);

MODULE_AUTHOR("Ley Foon Tan <lftan@altera.com>");
MODULE_DESCRIPTION("Altera PCIe host controller driver");
MODULE_LICENSE("GPL v2");
