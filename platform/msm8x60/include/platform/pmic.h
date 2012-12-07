/*
 * * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __PLATFORM_MSM8X60_PMIC_H
#define __PLATFORM_MSM8X60_PMIC_H

/* PMIC 8901 LDO Module defines */
#define PM8901_LDO_BASE (0x2F)

#define PM8901_LDO_L0           (PM8901_LDO_BASE + 0x00)
#define PM8901_LDO_L0_TEST_BANK     (PM8901_LDO_BASE + 0x01)
#define PM8901_LDO_L1           (PM8901_LDO_BASE + 0x02)
#define PM8901_LDO_L1_TEST_BANK     (PM8901_LDO_BASE + 0x03)
#define PM8901_LDO_L2           (PM8901_LDO_BASE + 0x04)
#define PM8901_LDO_L2_TEST_BANK     (PM8901_LDO_BASE + 0x05)
#define PM8901_LDO_L3           (PM8901_LDO_BASE + 0x06)
#define PM8901_LDO_L3_TEST_BANK     (PM8901_LDO_BASE + 0x07)
#define PM8901_LDO_L4           (PM8901_LDO_BASE + 0x08)
#define PM8901_LDO_L4_TEST_BANK     (PM8901_LDO_BASE + 0x09)
#define PM8901_LDO_L5           (PM8901_LDO_BASE + 0x0A)
#define PM8901_LDO_L5_TEST_BANK     (PM8901_LDO_BASE + 0x0B)
#define PM8901_LDO_L6           (PM8901_LDO_BASE + 0x0C)
#define PM8901_LDO_L6_TEST_BANK     (PM8901_LDO_BASE + 0x0D)
#define PM8901_LDO_L7           (PM8901_LDO_BASE + 0x0E)
#define PM8901_LDO_L7_TEST_BANK     (PM8901_LDO_BASE + 0x0F)
#define PM8901_PMR_7            (0xAD)

#define PM8901_LDO_TEST_BANK(n) ((n)<<4)

#define PM8901_LDO_CTL_ENABLE__S    (7)
#define PM8901_LDO_CTL_PULL_DOWN__S (6)
#define PM8901_LDO_CTL_MODE__S      (5)
/* LDO CTL */
#define LDO_CTL_ENABLE_MASK     (0x80)
#define LDO_CTL_PULL_DOWN_MASK      (0x40)
#define LDO_CTL_NORMAL_POWER_MODE_MASK  (0x20)
#define LDO_CTL_VOLTAGE_SET_MASK    (0x1F)

/* LDO TEST BANK 2 */
#define LDO_TEST_RANGE_SELECT_MASK (0x01)

/* LDO TEST BANK 4 */
#define LDO_TEST_OUTPUT_RANGE_MASK (0x01)

/* LDO TEST BANK 5 */
#define LDO_TEST_XO_EN_ALL_MASK (0x1F)

/* PMIC 8058 defines */
#define PM8058_LPG_CTL_BASE      (0x13C)
#define PM8058_LPG_CTL(n)        (PM8058_LPG_CTL_BASE + (n))
#define PM8058_LPG_BANK_SEL      (0x143)
#define PM8058_LPG_BANK_ENABLE   (0x144)

#define PM8058_GPIOS		40

#define GPIO24_GPIO_CNTRL (0x167)
#define GPIO25_GPIO_CNTRL (0x168)

#define IRQ_BLOCK_SEL_USR_ADDR 0x1C0
#define IRQ_STATUS_RT_USR_ADDR 0x1C3

/* PMIC 8058 LDO module defines */
#define PM8058_LDO_CTRL_L0   (0x009)
#define PM8058_LDO_TEST_L0   (0x065)
#define PM8058_LDO_CTRL_L1   (0x00A)
#define PM8058_LDO_TEST_L1   (0x066)
#define PM8058_LDO_CTRL_L2   (0x00B)
#define PM8058_LDO_TEST_L2   (0x067)
#define PM8058_LDO_CTRL_L3   (0x00C)
#define PM8058_LDO_TEST_L3   (0x068)
#define PM8058_LDO_CTRL_L4   (0x00D)
#define PM8058_LDO_TEST_L4   (0x069)
#define PM8058_LDO_CTRL_L5   (0x00E)
#define PM8058_LDO_TEST_L5   (0x06A)
#define PM8058_LDO_CTRL_L6   (0x00F)
#define PM8058_LDO_TEST_L6   (0x06B)
#define PM8058_LDO_CTRL_L7   (0x010)
#define PM8058_LDO_TEST_L7   (0x06C)
#define PM8058_LDO_CTRL_L8   (0x011)
#define PM8058_LDO_TEST_L8   (0x06D)
#define PM8058_LDO_CTRL_L9   (0x012)
#define PM8058_LDO_TEST_L9   (0x06E)
#define PM8058_LDO_CTRL_L10  (0x013)
#define PM8058_LDO_TEST_L10  (0x06F)
#define PM8058_LDO_CTRL_L11  (0x014)
#define PM8058_LDO_TEST_L11  (0x070)
#define PM8058_LDO_CTRL_L12  (0x015)
#define PM8058_LDO_TEST_L12  (0x071)
#define PM8058_LDO_CTRL_L13  (0x016)
#define PM8058_LDO_TEST_L13  (0x072)
#define PM8058_LDO_CTRL_L14  (0x017)
#define PM8058_LDO_TEST_L14  (0x073)

#define PM8058_LDO_CTRL_L15  (0x089)
#define PM8058_LDO_TEST_L15  (0x0E5)
#define PM8058_LDO_CTRL_L16  (0x08A)
#define PM8058_LDO_TEST_L16  (0x0E6)
#define PM8058_LDO_CTRL_L17  (0x08B)
#define PM8058_LDO_TEST_L17  (0x0E7)

#define PM8058_LDO_CTRL_L18  (0x11D)
#define PM8058_LDO_TEST_L18  (0x125)
#define PM8058_LDO_CTRL_L19  (0x11E)
#define PM8058_LDO_TEST_L19  (0x126)
#define PM8058_LDO_CTRL_L20  (0x11F)
#define PM8058_LDO_TEST_L20  (0x127)
#define PM8058_LDO_CTRL_L21  (0x120)
#define PM8058_LDO_TEST_L21  (0x128)
#define PM8058_LDO_CTRL_L22  (0x121)
#define PM8058_LDO_TEST_L22  (0x129)
#define PM8058_LDO_CTRL_L23  (0x122)
#define PM8058_LDO_TEST_L23  (0x12A)
#define PM8058_LDO_CTRL_L24  (0x123)
#define PM8058_LDO_TEST_L24  (0x12B)
#define PM8058_LDO_CTRL_L25  (0x124)
#define PM8058_LDO_TEST_L25  (0x12C)






#define TRUE  1
#define FALSE 0

/* FTS regulator PMR registers */
#define SSBI_REG_ADDR_S1_PMR		(0xA7)
#define SSBI_REG_ADDR_S2_PMR		(0xA8)
#define SSBI_REG_ADDR_S3_PMR		(0xA9)
#define SSBI_REG_ADDR_S4_PMR		(0xAA)

#define REGULATOR_PMR_STATE_MASK	0x60
#define REGULATOR_PMR_STATE_OFF		0x20

/* Regulator control registers for shutdown/reset */
#define SSBI_REG_ADDR_L22_CTRL		0x121

/* SLEEP CNTL register */
#define SSBI_REG_ADDR_SLEEP_CNTL	0x02B

#define PM8058_SLEEP_SMPL_EN_MASK	0x04
#define PM8058_SLEEP_SMPL_EN_RESET	0x04
#define PM8058_SLEEP_SMPL_EN_PWR_OFF	0x00

/* PON CNTL 1 register */
#define SSBI_REG_ADDR_PON_CNTL_1	0x01C

#define PM8058_PON_PUP_MASK		0xF0

#define PM8058_PON_WD_EN_MASK		0x08
#define PM8058_PON_WD_EN_RESET		0x08
#define PM8058_PON_WD_EN_PWR_OFF	0x00

#define PM8058_RTC_CTRL		0x1E8
#define PM8058_RTC_ALARM_ENABLE	BIT(1)

#define PM_IRQ_ID_TO_BLOCK_INDEX(id) (uint8_t)(id / 8)
#define PM_IRQ_ID_TO_BIT_MASK(id)    (uint8_t)(1 << (id % 8))

/* HDMI MPP Registers */
#define SSBI_MPP_CNTRL_BASE		0x27
#define SSBI_MPP_CNTRL(n)		(SSBI_MPP_CNTRL_BASE + (n))

#define PM8901_MPP_TYPE_MASK		0xE0
#define PM8901_MPP_CONFIG_LVL_MASK	0x1C
#define PM8901_MPP_CONFIG_CTL_MASK	0x03
#define PM8901_MPP0_CTRL_VAL		0x30
#define VREG_PMR_STATE_MASK		0x60
#define VREG_PMR_STATE_HPM		0x7F
#define VS_CTRL_USE_PMR			0xD0
#define VS_CTRL_ENABLE_MASK		0xD0
#define LDO_CTRL_VPROG_MASK		0x1F
#define REGULATOR_EN_MASK		0x80
#define PM8901_HDMI_MVS_CTRL		0x058
#define PM8901_HDMI_MVS_PMR		0x0B8
#define PM8058_HDMI_L16_CTRL		0x08A

#define PM8058_MIPI_L0_CTRL		0x009



/* Low dropout regulator ids */
#define PM8058_VREG_ID_L0	0
#define PM8058_VREG_ID_L1	1
#define PM8058_VREG_ID_L2	2
#define PM8058_VREG_ID_L3	3
#define PM8058_VREG_ID_L4	4
#define PM8058_VREG_ID_L5	5
#define PM8058_VREG_ID_L6	6
#define PM8058_VREG_ID_L7	7
#define PM8058_VREG_ID_L8	8
#define PM8058_VREG_ID_L9	9
#define PM8058_VREG_ID_L10	10
#define PM8058_VREG_ID_L11	11
#define PM8058_VREG_ID_L12	12
#define PM8058_VREG_ID_L13	13
#define PM8058_VREG_ID_L14	14
#define PM8058_VREG_ID_L15	15
#define PM8058_VREG_ID_L16	16
#define PM8058_VREG_ID_L17	17
#define PM8058_VREG_ID_L18	18
#define PM8058_VREG_ID_L19	19
#define PM8058_VREG_ID_L20	20
#define PM8058_VREG_ID_L21	21
#define PM8058_VREG_ID_L22	22
#define PM8058_VREG_ID_L23	23
#define PM8058_VREG_ID_L24	24
#define PM8058_VREG_ID_L25	25

/* Switched-mode power supply regulator ids */
#define PM8058_VREG_ID_S0	26
#define PM8058_VREG_ID_S1	27
#define PM8058_VREG_ID_S2	28
#define PM8058_VREG_ID_S3	29
#define PM8058_VREG_ID_S4	30

/* Low voltage switch regulator ids */
#define PM8058_VREG_ID_LVS0	31
#define PM8058_VREG_ID_LVS1	32

/* Negative charge pump regulator id */
#define PM8058_VREG_ID_NCP	33

#define PM8058_VREG_MAX		(PM8058_VREG_ID_NCP + 1)

#define PM8058_VREG_PIN_CTRL_NONE	0x00
#define PM8058_VREG_PIN_CTRL_A0		0x01
#define PM8058_VREG_PIN_CTRL_A1		0x02
#define PM8058_VREG_PIN_CTRL_D0		0x04
#define PM8058_VREG_PIN_CTRL_D1		0x08

/* Minimum high power mode loads in uA. */
#define PM8058_VREG_LDO_50_HPM_MIN_LOAD		5000
#define PM8058_VREG_LDO_150_HPM_MIN_LOAD	10000
#define PM8058_VREG_LDO_300_HPM_MIN_LOAD	10000
#define PM8058_VREG_SMPS_HPM_MIN_LOAD		50000


typedef enum {
	PM_KYPD_PWRON_IRQ_ID = 51,

	/* Block 24 Interrupts */
	PM_GPIO01_CHGED_ST_IRQ_ID = 192,
	PM_GPIO02_CHGED_ST_IRQ_ID = 193,
	PM_GPIO03_CHGED_ST_IRQ_ID = 194,
	PM_GPIO04_CHGED_ST_IRQ_ID = 195,
	PM_GPIO05_CHGED_ST_IRQ_ID = 196,
	PM_GPIO06_CHGED_ST_IRQ_ID = 197,
	PM_GPIO07_CHGED_ST_IRQ_ID = 198,
	PM_GPIO08_CHGED_ST_IRQ_ID = 199,

	/* Block 25 Interrupts */
	PM_GPIO09_CHGED_ST_IRQ_ID = 200,
	PM_GPIO10_CHGED_ST_IRQ_ID = 201,
	PM_GPIO11_CHGED_ST_IRQ_ID = 202,
	PM_GPIO12_CHGED_ST_IRQ_ID = 203,
	PM_GPIO13_CHGED_ST_IRQ_ID = 204,
	PM_GPIO14_CHGED_ST_IRQ_ID = 205,
	PM_GPIO15_CHGED_ST_IRQ_ID = 206,
	PM_GPIO16_CHGED_ST_IRQ_ID = 207,

	/* Block 26 Interrupts */
	PM_GPIO17_CHGED_ST_IRQ_ID = 208,
	PM_GPIO18_CHGED_ST_IRQ_ID = 209,
	PM_GPIO19_CHGED_ST_IRQ_ID = 210,
	PM_GPIO20_CHGED_ST_IRQ_ID = 211,
	PM_GPIO21_CHGED_ST_IRQ_ID = 212,
	PM_GPIO22_CHGED_ST_IRQ_ID = 213,
	PM_GPIO23_CHGED_ST_IRQ_ID = 214,
	PM_GPIO24_CHGED_ST_IRQ_ID = 215,

	/* Block 27 Interrupts */
	PM_GPIO25_CHGED_ST_IRQ_ID = 216,
	PM_GPIO26_CHGED_ST_IRQ_ID = 217,
	PM_GPIO27_CHGED_ST_IRQ_ID = 218,
	PM_GPIO28_CHGED_ST_IRQ_ID = 219,
	PM_GPIO29_CHGED_ST_IRQ_ID = 220,
	PM_GPIO30_CHGED_ST_IRQ_ID = 221,
	PM_GPIO31_CHGED_ST_IRQ_ID = 222,
	PM_GPIO32_CHGED_ST_IRQ_ID = 223,

	/* Block 28 Interrupts */
	PM_GPIO33_CHGED_ST_IRQ_ID = 224,
	PM_GPIO34_CHGED_ST_IRQ_ID = 225,
	PM_GPIO35_CHGED_ST_IRQ_ID = 226,
	PM_GPIO36_CHGED_ST_IRQ_ID = 227,
	PM_GPIO37_CHGED_ST_IRQ_ID = 228,
	PM_GPIO38_CHGED_ST_IRQ_ID = 229,
	PM_GPIO39_CHGED_ST_IRQ_ID = 230,
	PM_GPIO40_CHGED_ST_IRQ_ID = 231,
} pm_irq_id_type;




/* Regulator types */
#define REGULATOR_TYPE_LDO		0
#define REGULATOR_TYPE_SMPS		1
#define REGULATOR_TYPE_LVS		2
#define REGULATOR_TYPE_NCP		3

/* Common masks */
#define REGULATOR_EN_MASK		0x80

#define REGULATOR_BANK_MASK		0xF0
#define REGULATOR_BANK_SEL(n)		((n) << 4)
#define REGULATOR_BANK_WRITE		0x80

#define LDO_TEST_BANKS			7
#define SMPS_TEST_BANKS			8
#define REGULATOR_TEST_BANKS_MAX	SMPS_TEST_BANKS

/* LDO programming */

/* CTRL register */
#define LDO_ENABLE_MASK				0x80
#define LDO_ENABLE					0x80
#define LDO_PULL_DOWN_ENABLE_MASK	0x40
#define LDO_PULL_DOWN_ENABLE		0x40

#define LDO_CTRL_PM_MASK			0x20
#define LDO_CTRL_PM_HPM			0x00
#define LDO_CTRL_PM_LPM			0x20

#define LDO_CTRL_VPROG_MASK		0x1F

/* TEST register bank 0 */
#define LDO_TEST_LPM_MASK			0x40
#define LDO_TEST_LPM_SEL_CTRL		0x00
#define LDO_TEST_LPM_SEL_TCXO		0x40

/* TEST register bank 2 */
#define LDO_TEST_VPROG_UPDATE_MASK	0x08
#define LDO_TEST_RANGE_SEL_MASK		0x04
#define LDO_TEST_FINE_STEP_MASK		0x02
#define LDO_TEST_FINE_STEP_SHIFT		1

/* TEST register bank 4 */
#define LDO_TEST_RANGE_EXT_MASK		0x01

/* TEST register bank 5 */
#define LDO_TEST_PIN_CTRL_MASK		0x0F
#define LDO_TEST_PIN_CTRL_EN3		0x08
#define LDO_TEST_PIN_CTRL_EN2		0x04
#define LDO_TEST_PIN_CTRL_EN1		0x02
#define LDO_TEST_PIN_CTRL_EN0		0x01

/* TEST register bank 6 */
#define LDO_TEST_PIN_CTRL_LPM_MASK	0x0F

/* Allowable voltage ranges */
#define PLDO_LOW_UV_MIN				750000
#define PLDO_LOW_UV_MAX				1537500
#define PLDO_LOW_FINE_STEP_UV		12500

#define PLDO_NORM_UV_MIN			1500000
#define PLDO_NORM_UV_MAX			3075000
#define PLDO_NORM_FINE_STEP_UV		25000

#define PLDO_HIGH_UV_MIN				1750000
#define PLDO_HIGH_UV_MAX				4900000
#define PLDO_HIGH_FINE_STEP_UV		50000

#define NLDO_UV_MIN					750000
#define NLDO_UV_MAX					1537500
#define NLDO_FINE_STEP_UV				12500

/* SMPS masks and values */

/* CTRL register */

/* Legacy mode */
#define SMPS_LEGACY_ENABLE				0x80
#define SMPS_LEGACY_PULL_DOWN_ENABLE	0x40
#define SMPS_LEGACY_VREF_SEL_MASK		0x20
#define SMPS_LEGACY_VPROG_MASK			0x1F

/* Advanced mode */
#define SMPS_ADVANCED_BAND_MASK		0xC0
#define SMPS_ADVANCED_BAND_OFF			0x00
#define SMPS_ADVANCED_BAND_1			0x40
#define SMPS_ADVANCED_BAND_2			0x80
#define SMPS_ADVANCED_BAND_3			0xC0
#define SMPS_ADVANCED_VPROG_MASK		0x3F

/* Legacy mode voltage ranges */
#define SMPS_MODE1_UV_MIN		1500000
#define SMPS_MODE1_UV_MAX		3050000
#define SMPS_MODE1_UV_STEP		50000

#define SMPS_MODE2_UV_MIN		750000
#define SMPS_MODE2_UV_MAX		1525000
#define SMPS_MODE2_UV_STEP		25000

#define SMPS_MODE3_UV_MIN		375000
#define SMPS_MODE3_UV_MAX		1150000
#define SMPS_MODE3_UV_STEP		25000

/* Advanced mode voltage ranges */
#define SMPS_BAND3_UV_MIN		1500000
#define SMPS_BAND3_UV_MAX		3075000
#define SMPS_BAND3_UV_STEP		25000

#define SMPS_BAND2_UV_MIN		750000
#define SMPS_BAND2_UV_MAX		1537500
#define SMPS_BAND2_UV_STEP		12500

#define SMPS_BAND1_UV_MIN		375000
#define SMPS_BAND1_UV_MAX		1162500
#define SMPS_BAND1_UV_STEP		12500

#define SMPS_UV_MIN			SMPS_MODE3_UV_MIN
#define SMPS_UV_MAX			SMPS_MODE1_UV_MAX

/* Test2 register bank 1 */
#define SMPS_LEGACY_VLOW_SEL_MASK	0x01

/* Test2 register bank 6 */
#define SMPS_ADVANCED_PULL_DOWN_ENABLE	0x08

/* Test2 register bank 7 */
#define SMPS_ADVANCED_MODE_MASK	0x02
#define SMPS_ADVANCED_MODE			0x02
#define SMPS_LEGACY_MODE			0x00

#define SMPS_IN_ADVANCED_MODE(vreg) \
	((vreg->test_reg[7] & SMPS_ADVANCED_MODE_MASK) == SMPS_ADVANCED_MODE)

/* BUCK_SLEEP_CNTRL register */
#define SMPS_PIN_CTRL_MASK	0xF0
#define SMPS_PIN_CTRL_A1		0x80
#define SMPS_PIN_CTRL_A0		0x40
#define SMPS_PIN_CTRL_D1		0x20
#define SMPS_PIN_CTRL_D0		0x10

#define SMPS_PIN_CTRL_LPM_MASK	0x0F
#define SMPS_PIN_CTRL_LPM_A1		0x08
#define SMPS_PIN_CTRL_LPM_A0		0x04
#define SMPS_PIN_CTRL_LPM_D1		0x02
#define SMPS_PIN_CTRL_LPM_D0		0x01

/* BUCK_CLOCK_CNTRL register */
#define SMPS_CLK_DIVIDE2		0x40

#define SMPS_CLK_CTRL_MASK			0x30
#define SMPS_CLK_CTRL_FOLLOW_TCXO	0x00
#define SMPS_CLK_CTRL_PWM			0x10
#define SMPS_CLK_CTRL_PFM			0x20

/* LVS masks and values */

/* CTRL register */
#define LVS_ENABLE_MASK				0x80
#define LVS_ENABLE						0x80
#define LVS_PULL_DOWN_ENABLE_MASK	0x40
#define LVS_PULL_DOWN_ENABLE		0x40

#define LVS_PIN_CTRL_MASK		0x0F
#define LVS_PIN_CTRL_EN0		0x08
#define LVS_PIN_CTRL_EN1		0x04
#define LVS_PIN_CTRL_EN2		0x02
#define LVS_PIN_CTRL_EN3		0x01

/* NCP masks and values */

/* CTRL register */
#define NCP_VPROG_MASK		0x1F

#define NCP_UV_MIN			1500000
#define NCP_UV_MAX			3050000
#define NCP_UV_STEP			50000



/* Pin ctrl enables/disables or toggles high/low power modes */
enum pm8058_vreg_pin_fn {
	PM8058_VREG_PIN_FN_ENABLE = 0,
	PM8058_VREG_PIN_FN_MODE,
};

struct pm8058_vreg_pdata {
//	struct regulator_init_data	init_data;
	unsigned			pull_down_enable;
	unsigned			pin_ctrl;
	enum pm8058_vreg_pin_fn		pin_fn;
};

#define GLOBAL_ENABLE_MAX		(2)
struct pm8058_enable {
	uint16_t		addr;
	uint8_t			reg;
};

struct pm8058_vreg {
	struct pm8058_vreg_pdata	*pdata;
//	struct regulator_dev		*rdev;
	struct pm8058_enable		*global_enable[GLOBAL_ENABLE_MAX];
	int				hpm_min_load;
	int				save_uV;
	unsigned		pc_vote;
	unsigned		optimum;
	unsigned		mode_initialized;
	uint16_t		ctrl_addr;
	uint16_t		test_addr;
	uint16_t		clk_ctrl_addr;
	uint16_t		sleep_ctrl_addr;
	uint8_t			type;
	uint8_t			ctrl_reg;
	uint8_t			test_reg[REGULATOR_TEST_BANKS_MAX];
	uint8_t			clk_ctrl_reg;
	uint8_t			sleep_ctrl_reg;
	uint8_t			is_nmos;
	uint8_t			global_enable_mask[GLOBAL_ENABLE_MAX];
};



#define LDO_M2(_id, _ctrl_addr, _test_addr, _is_nmos, _hpm_min_load, _en0, _en0_mask, _en1, _en1_mask) \
	[PM8058_VREG_ID_##_id] = { \
		.ctrl_addr = _ctrl_addr, \
		.test_addr = _test_addr, \
		.type = REGULATOR_TYPE_LDO, \
		.hpm_min_load = PM8058_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
		.is_nmos = _is_nmos, \
		.global_enable = { \
			[0] = _en0, \
			[1] = _en1, \
		}, \
		.global_enable_mask = { \
			[0] = _en0_mask, \
			[1] = _en1_mask, \
		}, \
	}

#define LDO(_id, _ctrl_addr, _test_addr, _is_nmos, _hpm_min_load, _en0, _en0_mask) \
		LDO_M2(_id, _ctrl_addr, _test_addr, _is_nmos, _hpm_min_load, _en0, _en0_mask, NULL, 0)

#define SMPS(_id, _ctrl_addr, _test_addr, _clk_ctrl_addr, _sleep_ctrl_addr, _hpm_min_load, _en0, _en0_mask) \
	[PM8058_VREG_ID_##_id] = { \
		.ctrl_addr = _ctrl_addr, \
		.test_addr = _test_addr, \
		.clk_ctrl_addr = _clk_ctrl_addr, \
		.sleep_ctrl_addr = _sleep_ctrl_addr, \
		.type = REGULATOR_TYPE_SMPS, \
		.hpm_min_load = PM8058_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
		.global_enable = { \
			[0] = _en0, \
			[1] = NULL, \
		}, \
		.global_enable_mask = { \
			[0] = _en0_mask, \
			[1] = 0, \
		}, \
	}

#define LVS(_id, _ctrl_addr, _en0, _en0_mask) \
	[PM8058_VREG_ID_##_id] = { \
		.ctrl_addr = _ctrl_addr, \
		.type = REGULATOR_TYPE_LVS, \
		.global_enable = { \
			[0] = _en0, \
			[1] = NULL, \
		}, \
		.global_enable_mask = { \
			[0] = _en0_mask, \
			[1] = 0, \
		}, \
	}

#define NCP(_id, _ctrl_addr, _test1) \
	[PM8058_VREG_ID_##_id] = { \
		.ctrl_addr = _ctrl_addr, \
		.type = REGULATOR_TYPE_NCP, \
		.test_addr = _test1, \
		.global_enable = { \
			[0] = NULL, \
			[1] = NULL, \
		}, \
		.global_enable_mask = { \
			[0] = 0, \
			[1] = 0, \
		}, \
	}

#define MASTER_ENABLE_COUNT	6

#define EN_MSM			0
#define EN_PH			1
#define EN_RF			2
#define EN_GRP_5_4		3
#define EN_GRP_3_2		4
#define EN_GRP_1_0		5



struct pm8058_gpio1 {
	int		direction;
	int		output_buffer;
	int		output_value;
	int		pull;
	int		vin_sel;	/* 0..7 */
	int		out_strength;
	int		function;
	int		inv_int_pol;	/* invert interrupt polarity */
	int		disable_pin;	/* disable pin and tri-state its pad */
};


#endif
