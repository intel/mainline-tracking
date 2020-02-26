// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Jasper Lake PCH pinctrl/GPIO driver
 *
 * Copyright (C) 2019, Intel Corporation
 * Author: Andy Shevchenko <andriy.shevchenko@linux.intel.com>
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-intel.h"

#define JSP_PAD_OWN	0x020
#define JSP_PADCFGLOCK	0x080
#define JSP_HOSTSW_OWN	0x0b0
#define JSP_GPI_IS	0x100
#define JSP_GPI_IE	0x120

#define JSP_GPP(r, s, e)				\
	{						\
		.reg_num = (r),				\
		.base = (s),				\
		.size = ((e) - (s) + 1),		\
	}

#define JSP_COMMUNITY(s, e, g)				\
	{						\
		.padown_offset = JSP_PAD_OWN,		\
		.padcfglock_offset = JSP_PADCFGLOCK,	\
		.hostown_offset = JSP_HOSTSW_OWN,	\
		.is_offset = JSP_GPI_IS,		\
		.ie_offset = JSP_GPI_IE,		\
		.pin_base = (s),			\
		.npins = ((e) - (s) + 1),		\
		.gpps = (g),				\
		.ngpps = ARRAY_SIZE(g),			\
	}

/* Jasper Lake */
static const struct pinctrl_pin_desc jsp_community0_pins[] = {
	/* GPP_F */
	PINCTRL_PIN(0, "CNV_BRI_DT_UART0_RTSB"),
	PINCTRL_PIN(1, "CNV_BRI_RSP_UART0_RXD"),
	PINCTRL_PIN(2, "EMMC_HIP_MON"),
	PINCTRL_PIN(3, "CNV_RGI_RSP_UART0_CTSB"),
	PINCTRL_PIN(4, "CNV_RF_RESET_B"),
	PINCTRL_PIN(5, "MODEM_CLKREQ"),
	PINCTRL_PIN(6, "CNV_PA_BLANKING"),
	PINCTRL_PIN(7, "EMMC_CMD"),
	PINCTRL_PIN(8, "EMMC_DATA0"),
	PINCTRL_PIN(9, "EMMC_DATA1"),
	PINCTRL_PIN(10, "EMMC_DATA2"),
	PINCTRL_PIN(11, "EMMC_DATA3"),
	PINCTRL_PIN(12, "EMMC_DATA4"),
	PINCTRL_PIN(13, "EMMC_DATA5"),
	PINCTRL_PIN(14, "EMMC_DATA6"),
	PINCTRL_PIN(15, "EMMC_DATA7"),
	PINCTRL_PIN(16, "EMMC_RCLK"),
	PINCTRL_PIN(17, "EMMC_CLK"),
	PINCTRL_PIN(18, "EMMC_RESETB"),
	PINCTRL_PIN(19, "A4WP_PRESENT"),
	/* GPP_B */
	PINCTRL_PIN(20, "CORE_VID_0"),
	PINCTRL_PIN(21, "CORE_VID_1"),
	PINCTRL_PIN(22, "VRALERTB"),
	PINCTRL_PIN(23, "CPU_GP_2"),
	PINCTRL_PIN(24, "CPU_GP_3"),
	PINCTRL_PIN(25, "SRCCLKREQB_0"),
	PINCTRL_PIN(26, "SRCCLKREQB_1"),
	PINCTRL_PIN(27, "SRCCLKREQB_2"),
	PINCTRL_PIN(28, "SRCCLKREQB_3"),
	PINCTRL_PIN(29, "SRCCLKREQB_4"),
	PINCTRL_PIN(30, "SRCCLKREQB_5"),
	PINCTRL_PIN(31, "PMCALERTB"),
	PINCTRL_PIN(32, "SLP_S0B"),
	PINCTRL_PIN(33, "PLTRSTB"),
	PINCTRL_PIN(34, "SPKR"),
	PINCTRL_PIN(35, "GSPI0_CS0B"),
	PINCTRL_PIN(36, "GSPI0_CLK"),
	PINCTRL_PIN(37, "GSPI0_MISO"),
	PINCTRL_PIN(38, "GSPI0_MOSI"),
	PINCTRL_PIN(39, "GSPI1_CS0B"),
	PINCTRL_PIN(40, "GSPI1_CLK"),
	PINCTRL_PIN(41, "GSPI1_MISO"),
	PINCTRL_PIN(42, "GSPI1_MOSI"),
	PINCTRL_PIN(43, "DDSP_HPD_A"),
	PINCTRL_PIN(44, "GSPI0_CLK_LOOPBK"),
	PINCTRL_PIN(45, "GSPI1_CLK_LOOPBK"),
	/* GPP_A */
	PINCTRL_PIN(46, "ESPI_IO_0"),
	PINCTRL_PIN(47, "ESPI_IO_1"),
	PINCTRL_PIN(48, "ESPI_IO_2"),
	PINCTRL_PIN(49, "ESPI_IO_3"),
	PINCTRL_PIN(50, "ESPI_CSB"),
	PINCTRL_PIN(51, "ESPI_CLK"),
	PINCTRL_PIN(52, "ESPI_RESETB"),
	PINCTRL_PIN(53, "SMBCLK"),
	PINCTRL_PIN(54, "SMBDATA"),
	PINCTRL_PIN(55, "SMBALERTB"),
	PINCTRL_PIN(56, "CPU_GP_0"),
	PINCTRL_PIN(57, "CPU_GP_1"),
	PINCTRL_PIN(58, "USB2_OCB_1"),
	PINCTRL_PIN(59, "USB2_OCB_2"),
	PINCTRL_PIN(60, "USB2_OCB_3"),
	PINCTRL_PIN(61, "DDSP_HPD_A_TIME_SYNC_0"),
	PINCTRL_PIN(62, "DDSP_HPD_B"),
	PINCTRL_PIN(63, "DDSP_HPD_C"),
	PINCTRL_PIN(64, "USB2_OCB_0"),
	PINCTRL_PIN(65, "PCHHOTB"),
	PINCTRL_PIN(66, "ESPI_CLK_LOOPBK"),
	/* GPP_S */
	PINCTRL_PIN(67, "SNDW1_CLK"),
	PINCTRL_PIN(68, "SNDW1_DATA"),
	PINCTRL_PIN(69, "SNDW2_CLK"),
	PINCTRL_PIN(70, "SNDW2_DATA"),
	PINCTRL_PIN(71, "SNDW1_CLK"),
	PINCTRL_PIN(72, "SNDW1_DATA"),
	PINCTRL_PIN(73, "SNDW4_CLK_DMIC_CLK_0"),
	PINCTRL_PIN(74, "SNDW4_DATA_DMIC_DATA_0"),
	/* GPP_R */
	PINCTRL_PIN(75, "HDA_BCLK"),
	PINCTRL_PIN(76, "HDA_SYNC"),
	PINCTRL_PIN(77, "HDA_SDO"),
	PINCTRL_PIN(78, "HDA_SDI_0"),
	PINCTRL_PIN(79, "HDA_RSTB"),
	PINCTRL_PIN(80, "HDA_SDI_1"),
	PINCTRL_PIN(81, "I2S1_SFRM"),
	PINCTRL_PIN(82, "I2S1_TXD"),
};

static const struct intel_padgroup jsp_community0_gpps[] = {
	JSP_GPP(0, 0, 19),	/* GPP_F */
	JSP_GPP(1, 20, 45),	/* GPP_B */
	JSP_GPP(2, 46, 66),	/* GPP_A */
	JSP_GPP(3, 67, 74),	/* GPP_S */
	JSP_GPP(4, 75, 82),	/* GPP_R */
};

static const struct intel_community jsp_community0[] = {
	JSP_COMMUNITY(0, 82, jsp_community0_gpps),
};

static const struct intel_pinctrl_soc_data jsp_community0_soc_data = {
	.uid = "0",
	.pins = jsp_community0_pins,
	.npins = ARRAY_SIZE(jsp_community0_pins),
	.communities = jsp_community0,
	.ncommunities = ARRAY_SIZE(jsp_community0),
};

static const struct pinctrl_pin_desc jsp_community1_pins[] = {
	/* GPP_H */
	PINCTRL_PIN(0, "GPPC_H_0"),
	PINCTRL_PIN(1, "SD_PWR_EN_B"),
	PINCTRL_PIN(2, "MODEM_CLKREQ"),
	PINCTRL_PIN(3, "SX_EXIT_HOLDOFFB"),
	PINCTRL_PIN(4, "I2C2_SDA"),
	PINCTRL_PIN(5, "I2C2_SCL"),
	PINCTRL_PIN(6, "I2C3_SDA"),
	PINCTRL_PIN(7, "I2C3_SCL"),
	PINCTRL_PIN(8, "I2C4_SDA"),
	PINCTRL_PIN(9, "I2C4_SCL"),
	PINCTRL_PIN(10, "CPU_VCCIO_PWR_GATEB"),
	PINCTRL_PIN(11, "I2S2_SCLK"),
	PINCTRL_PIN(12, "I2S2_SFRM"),
	PINCTRL_PIN(13, "I2S2_TXD"),
	PINCTRL_PIN(14, "I2S2_RXD"),
	PINCTRL_PIN(15, "I2S1_SCLK"),
	PINCTRL_PIN(16, "GPPC_H_16"),
	PINCTRL_PIN(17, "GPPC_H_17"),
	PINCTRL_PIN(18, "GPPC_H_18"),
	PINCTRL_PIN(19, "GPPC_H_19"),
	PINCTRL_PIN(20, "GPPC_H_20"),
	PINCTRL_PIN(21, "GPPC_H_21"),
	PINCTRL_PIN(22, "GPPC_H_22"),
	PINCTRL_PIN(23, "GPPC_H_23"),
	/* GPP_D */
	PINCTRL_PIN(24, "SPI1_CSB"),
	PINCTRL_PIN(25, "SPI1_CLK"),
	PINCTRL_PIN(26, "SPI1_MISO_IO_1"),
	PINCTRL_PIN(27, "SPI1_MOSI_IO_0"),
	PINCTRL_PIN(28, "ISH_I2C0_SDA"),
	PINCTRL_PIN(29, "ISH_I2C0_SCL"),
	PINCTRL_PIN(30, "ISH_I2C1_SDA"),
	PINCTRL_PIN(31, "ISH_I2C1_SCL"),
	PINCTRL_PIN(32, "ISH_SPI_CSB"),
	PINCTRL_PIN(33, "ISH_SPI_CLK"),
	PINCTRL_PIN(34, "ISH_SPI_MISO"),
	PINCTRL_PIN(35, "ISH_SPI_MOSI"),
	PINCTRL_PIN(36, "ISH_UART0_RXD"),
	PINCTRL_PIN(37, "ISH_UART0_TXD"),
	PINCTRL_PIN(38, "ISH_UART0_RTSB"),
	PINCTRL_PIN(39, "ISH_UART0_CTSB"),
	PINCTRL_PIN(40, "SPI1_IO_2"),
	PINCTRL_PIN(41, "SPI1_IO_3"),
	PINCTRL_PIN(42, "I2S_MCLK"),
	PINCTRL_PIN(43, "CNV_MFUART2_RXD"),
	PINCTRL_PIN(44, "CNV_MFUART2_TXD"),
	PINCTRL_PIN(45, "CNV_PA_BLANKING"),
	PINCTRL_PIN(46, "I2C5_SDA"),
	PINCTRL_PIN(47, "I2C5_SCL"),
	PINCTRL_PIN(48, "GSPI2_CLK_LOOPBK"),
	PINCTRL_PIN(49, "SPI1_CLK_LOOPBK"),
	/* vGPIO */
	PINCTRL_PIN(50, "CNV_BTEN"),
	PINCTRL_PIN(51, "CNV_WCEN"),
	PINCTRL_PIN(52, "CNV_BT_HOST_WAKEB"),
	PINCTRL_PIN(53, "CNV_BT_IF_SELECT"),
	PINCTRL_PIN(54, "vCNV_BT_UART_TXD"),
	PINCTRL_PIN(55, "vCNV_BT_UART_RXD"),
	PINCTRL_PIN(56, "vCNV_BT_UART_CTS_B"),
	PINCTRL_PIN(57, "vCNV_BT_UART_RTS_B"),
	PINCTRL_PIN(58, "vCNV_MFUART1_TXD"),
	PINCTRL_PIN(59, "vCNV_MFUART1_RXD"),
	PINCTRL_PIN(60, "vCNV_MFUART1_CTS_B"),
	PINCTRL_PIN(61, "vCNV_MFUART1_RTS_B"),
	PINCTRL_PIN(62, "vUART0_TXD"),
	PINCTRL_PIN(63, "vUART0_RXD"),
	PINCTRL_PIN(64, "vUART0_CTS_B"),
	PINCTRL_PIN(65, "vUART0_RTS_B"),
	PINCTRL_PIN(66, "vISH_UART0_TXD"),
	PINCTRL_PIN(67, "vISH_UART0_RXD"),
	PINCTRL_PIN(68, "vISH_UART0_CTS_B"),
	PINCTRL_PIN(69, "vISH_UART0_RTS_B"),
	PINCTRL_PIN(70, "vCNV_BT_I2S_BCLK"),
	PINCTRL_PIN(71, "vCNV_BT_I2S_WS_SYNC"),
	PINCTRL_PIN(72, "vCNV_BT_I2S_SDO"),
	PINCTRL_PIN(73, "vCNV_BT_I2S_SDI"),
	PINCTRL_PIN(74, "vI2S2_SCLK"),
	PINCTRL_PIN(75, "vI2S2_SFRM"),
	PINCTRL_PIN(76, "vI2S2_TXD"),
	PINCTRL_PIN(77, "vI2S2_RXD"),
	PINCTRL_PIN(78, "vSD3_CD_B"),
	/* GPP_C */
	PINCTRL_PIN(79, "GPPC_C_0"),
	PINCTRL_PIN(80, "GPPC_C_1"),
	PINCTRL_PIN(81, "GPPC_C_2"),
	PINCTRL_PIN(82, "GPPC_C_3"),
	PINCTRL_PIN(83, "GPPC_C_4"),
	PINCTRL_PIN(84, "GPPC_C_5"),
	PINCTRL_PIN(85, "SUSWARNB_SUSPWRDNACK"),
	PINCTRL_PIN(86, "SUSACKB"),
	PINCTRL_PIN(87, "UART0_RXD"),
	PINCTRL_PIN(88, "UART0_TXD"),
	PINCTRL_PIN(89, "UART0_RTSB"),
	PINCTRL_PIN(90, "UART0_CTSB"),
	PINCTRL_PIN(91, "UART1_RXD"),
	PINCTRL_PIN(92, "UART1_TXD"),
	PINCTRL_PIN(93, "UART1_RTSB"),
	PINCTRL_PIN(94, "UART1_CTSB"),
	PINCTRL_PIN(95, "I2C0_SDA"),
	PINCTRL_PIN(96, "I2C0_SCL"),
	PINCTRL_PIN(97, "I2C1_SDA"),
	PINCTRL_PIN(98, "I2C1_SCL"),
	PINCTRL_PIN(99, "UART2_RXD"),
	PINCTRL_PIN(100, "UART2_TXD"),
	PINCTRL_PIN(101, "UART2_RTSB"),
	PINCTRL_PIN(102, "UART2_CTSB"),
};

static const struct intel_padgroup jsp_community1_gpps[] = {
	JSP_GPP(0, 0, 23),	/* GPP_H */
	JSP_GPP(1, 24, 49),	/* GPP_D */
	JSP_GPP(2, 50, 78),	/* vGPIO */
	JSP_GPP(3, 79, 102),	/* GPP_C */
};

static const struct intel_community jsp_community1[] = {
	JSP_COMMUNITY(0, 102, jsp_community1_gpps),
};

static const struct intel_pinctrl_soc_data jsp_community1_soc_data = {
	.uid = "1",
	.pins = jsp_community1_pins,
	.npins = ARRAY_SIZE(jsp_community1_pins),
	.communities = jsp_community1,
	.ncommunities = ARRAY_SIZE(jsp_community1),
};

static const struct pinctrl_pin_desc jsp_community4_pins[] = {
	/* GPP_E */
	PINCTRL_PIN(0, "ISH_GP_0"),
	PINCTRL_PIN(1, "ISH_GP_1"),
	PINCTRL_PIN(2, "IMGCLKOUT_1"),
	PINCTRL_PIN(3, "ISH_GP_2"),
	PINCTRL_PIN(4, "IMGCLKOUT_2"),
	PINCTRL_PIN(5, "SATA_LEDB"),
	PINCTRL_PIN(6, "IMGCLKOUT_3"),
	PINCTRL_PIN(7, "ISH_GP_3"),
	PINCTRL_PIN(8, "ISH_GP_4"),
	PINCTRL_PIN(9, "ISH_GP_5"),
	PINCTRL_PIN(10, "ISH_GP_6"),
	PINCTRL_PIN(11, "ISH_GP_7"),
	PINCTRL_PIN(12, "IMGCLKOUT_4"),
	PINCTRL_PIN(13, "DDPA_CTRLCLK"),
	PINCTRL_PIN(14, "DDPA_CTRLDATA"),
	PINCTRL_PIN(15, "DDPB_CTRLCLK"),
	PINCTRL_PIN(16, "DDPB_CTRLDATA"),
	PINCTRL_PIN(17, "DDPC_CTRLCLK"),
	PINCTRL_PIN(18, "DDPC_CTRLDATA"),
	PINCTRL_PIN(19, "IMGCLKOUT_5"),
	PINCTRL_PIN(20, "CNV_BRI_DT"),
	PINCTRL_PIN(21, "CNV_BRI_RSP"),
	PINCTRL_PIN(22, "CNV_RGI_DT"),
	PINCTRL_PIN(23, "CNV_RGI_RSP"),
	/* vGPIO_4 */
	PINCTRL_PIN(24, "CPU_PCIE_LNK_DN_0"),
	PINCTRL_PIN(25, "CPU_PCIE_LNK_DN_1"),
	PINCTRL_PIN(26, "CPU_PCIE_LNK_DN_2"),
	PINCTRL_PIN(27, "CPU_PCIE_LNK_DN_3"),
};

static const struct intel_padgroup jsp_community4_gpps[] = {
	JSP_GPP(0, 0, 23),	/* GPP_E */
	JSP_GPP(1, 24, 27),	/* vGPIO_4 */
};

static const struct intel_community jsp_community4[] = {
	JSP_COMMUNITY(0, 27, jsp_community4_gpps),
};

static const struct intel_pinctrl_soc_data jsp_community4_soc_data = {
	.uid = "4",
	.pins = jsp_community4_pins,
	.npins = ARRAY_SIZE(jsp_community4_pins),
	.communities = jsp_community4,
	.ncommunities = ARRAY_SIZE(jsp_community4),
};

static const struct pinctrl_pin_desc jsp_community5_pins[] = {
	/* GPP_G */
	PINCTRL_PIN(0, "SD3_CMD"),
	PINCTRL_PIN(1, "SD3_D0"),
	PINCTRL_PIN(2, "SD3_D1"),
	PINCTRL_PIN(3, "SD3_D2"),
	PINCTRL_PIN(4, "SD3_D3"),
	PINCTRL_PIN(5, "SD3_CDB"),
	PINCTRL_PIN(6, "SD3_CLK"),
	PINCTRL_PIN(7, "SD3_WP"),
};

static const struct intel_padgroup jsp_community5_gpps[] = {
	JSP_GPP(0, 0, 7),	/* GPP_G */
};

static const struct intel_community jsp_community5[] = {
	JSP_COMMUNITY(0, 7, jsp_community5_gpps),
};

static const struct intel_pinctrl_soc_data jsp_community5_soc_data = {
	.uid = "5",
	.pins = jsp_community5_pins,
	.npins = ARRAY_SIZE(jsp_community5_pins),
	.communities = jsp_community5,
	.ncommunities = ARRAY_SIZE(jsp_community5),
};

static const struct intel_pinctrl_soc_data *jsp_soc_data_array[] = {
	&jsp_community0_soc_data,
	&jsp_community1_soc_data,
	&jsp_community4_soc_data,
	&jsp_community5_soc_data,
	NULL
};

static const struct acpi_device_id jsp_pinctrl_acpi_match[] = {
	{ "INT34C8", (kernel_ulong_t)jsp_soc_data_array },
	{ }
};
MODULE_DEVICE_TABLE(acpi, jsp_pinctrl_acpi_match);

static INTEL_PINCTRL_PM_OPS(jsp_pinctrl_pm_ops);

static struct platform_driver jsp_pinctrl_driver = {
	.probe = intel_pinctrl_probe_by_uid,
	.driver = {
		.name = "jasperlake-pinctrl",
		.acpi_match_table = jsp_pinctrl_acpi_match,
		.pm = &jsp_pinctrl_pm_ops,
	},
};

module_platform_driver(jsp_pinctrl_driver);

MODULE_AUTHOR("Andy Shevchenko <andriy.shevchenko@linux.intel.com>");
MODULE_DESCRIPTION("Intel Jasper Lake PCH pinctrl/GPIO driver");
MODULE_LICENSE("GPL v2");
