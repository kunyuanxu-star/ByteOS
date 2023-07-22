/*
 * Copyright (C) 2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <linux/libfdt.h>
#include <reset.h>
#include <mmc.h>
#include <sdhci.h>

#define MMC_TYPE_MMC  0       /* MMC card */
#define MMC_TYPE_SD   1       /* SD card */
#define MMC_TYPE_SDIO 2       /* SDIO card */

#ifdef DEBUG
#define pr_debug(fmt, ...) \
	printf(fmt, ##__VA_ARGS__)
#endif

struct cvi_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct cvi_sdhci_host {
	struct sdhci_host host;
	int pll_index;
	int pll_reg;
	int no_1_8_v;
	int is_64_addressing;
	int reset_tx_rx_phy;
	uint32_t mmc_fmax_freq;
	uint32_t mmc_fmin_freq;
	struct reset_ctl reset_ctl;
};

struct cvi_sdhci_driver_data {
	const struct sdhci_ops *ops;
	int index;
};

static int cvi_sdhci_bind(struct udevice *dev)
{
	struct cvi_sdhci_plat *plat = dev_get_plat(dev);

	pr_debug("[hq] %s\n", __func__);
	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

const struct sdhci_ops cvi_sdhci_sd_ops = {
	.get_cd = cvi_get_cd,
	.platform_execute_tuning = cvi_general_execute_tuning,
	.voltage_switch = cvi_sd_voltage_switch,
	.reset = cvi_general_reset,
};

static const struct cvi_sdhci_driver_data sdhci_cvi_sd_drvdata = {
	.ops = &cvi_sdhci_sd_ops,
	.index = MMC_TYPE_SD,
};

static const struct udevice_id cvi_sdhci_match[] = {
	{
		.compatible = "cvitek,cv181x-sd",
		.data = (ulong)&sdhci_cvi_sd_drvdata,
	},
};

U_BOOT_DRIVER(cvi_sdhci) = {
	.name = "cvi_sdhci",
	.id = UCLASS_MMC,
	.of_match = cvi_sdhci_match,
	.of_to_plat = cvi_ofdata_to_platdata,
	.bind = cvi_sdhci_bind,
	.probe = cvi_sdhci_probe,
	.priv_auto = sizeof(struct cvi_sdhci_host),
	.plat_auto = sizeof(struct cvi_sdhci_plat),
	.ops = &sdhci_ops,
};