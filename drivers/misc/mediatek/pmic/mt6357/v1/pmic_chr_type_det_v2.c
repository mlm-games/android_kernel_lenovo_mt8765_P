/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include <mt-plat/mtk_battery.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/charger_type.h>

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
#include <mt-plat/mtk_usb2jtag.h>
#endif

/* #define __FORCE_USB_TYPE__ */
#define __SW_CHRDET_IN_PROBE_PHASE__

static enum charger_type g_chr_type;
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static struct work_struct chr_work;
#endif

static DEFINE_MUTEX(chrdet_lock);

#if !defined(CONFIG_FPGA_EARLY_PORTING)
static bool first_connect = true;
#endif

#define THUB_SUPPORT	1

#if THUB_SUPPORT
typedef enum {
   ACA_UNKNOWN = 0,
   ACA_DOCK, //has external power source
   ACA_A,     //B device on accessory port
   ACA_B,     //only for charging
   ACA_C,    // A device on accessory port
   ACA_UNCONFIG = 0x8, // Unconfig State
} ACA_TYPE;

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);

#define voltage_threshold 850//mv
#define voltage_otg 200

static int g_aca_detection = 0xF;

void hw_otg_reset_aca_type(void)
{
	g_aca_detection = 0xF;
	pr_err("%s!\n", __func__);
}

int hw_otg_get_aca_type_detection(void)
{
	int ADC_voltage, data[4], i, ret_value = 0;
	int times=1, IDDIG_CHANNEL_NUM = 12; //(AUXIN2)
	if( IMM_IsAdcInitReady() == 0 )
	{
		pr_err("[%s]: AUXADC is not ready\n", __func__);
		return ACA_UNCONFIG;
	}

	i = times;
	while (i--)
	{
		ret_value = IMM_GetOneChannelValue(IDDIG_CHANNEL_NUM, data, &ADC_voltage);
		ADC_voltage = ADC_voltage * 1500 / 4096;
		if (ret_value == -1) // AUXADC is busy
		{
			pr_err("IMM_GetOneChannelValue wrong, AUXADC is busy");
			ADC_voltage = 0;
		}
		pr_err("[%s]: AUXADC Channel %d, ADC_voltage %d\n, voltage_threshold %d",
				__func__, IDDIG_CHANNEL_NUM, ADC_voltage, voltage_threshold);
	}

	if (ADC_voltage > voltage_threshold)
		return ACA_UNCONFIG;
	else if (ADC_voltage < voltage_threshold && ADC_voltage > voltage_otg)
		return ACA_C;
	else
		return ACA_UNKNOWN;
}

int hw_otg_get_aca_type(void)
{
  /* need otg type detection */
  if(0xF == g_aca_detection)
	  g_aca_detection = hw_otg_get_aca_type_detection();

  return g_aca_detection;
}
#endif //THUB_SUPPORT

static struct power_supply *chrdet_psy;
static int chrdet_inform_psy_changed(enum charger_type chg_type, bool chg_online)
{
	int ret = 0;
	union power_supply_propval propval;

	pr_info("charger type: %s: online = %d, type = %d\n", __func__, chg_online, chg_type);

	/* Inform chg det power supply */
	if (chg_online) {
		propval.intval = chg_online;
		ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			pr_err("%s: psy online failed, ret = %d\n", __func__, ret);

		propval.intval = chg_type;
		ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0)
			pr_err("%s: psy type failed, ret = %d\n", __func__, ret);

		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_err("%s: psy type failed, ret(%d)\n", __func__, ret);

	propval.intval = chg_online;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE,
		&propval);
	if (ret < 0)
		pr_err("%s: psy online failed, ret(%d)\n", __func__, ret);
	return ret;
}

#if defined(CONFIG_FPGA_EARLY_PORTING)
/*****************************************************************************
 * FPGA
 ******************************************************************************/
int hw_charging_get_charger_type(void)
{
	g_chr_type = STANDARD_HOST;
	chrdet_inform_psy_changed(g_chr_type, 1);
	return g_chr_type;
}

#else
/*****************************************************************************
 * EVB / Phone
 ******************************************************************************/
static void hw_bc11_init(void)
{
	int timeout = 200;

	msleep(200);

	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_info("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_info("CDP, timeout\n");
			else
				pr_info("CDP, free\n");
		} else
			pr_info("CDP, PASS\n");
		first_connect = false;
	}

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	msleep(50);

	Charger_Detect_Init();
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		pr_info("charger type: DCP, keep DM voltage source in stepB2\n");
	}
	return wChargerAvail;

}

static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);
	Charger_Detect_Release();
}

static void dump_charger_name(enum charger_type type)
{
	switch (type) {
	case CHARGER_UNKNOWN:
		pr_info("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case STANDARD_HOST:
		pr_info("charger type: %d, Standard USB Host\n", type);
		break;
	case CHARGING_HOST:
		pr_info("charger type: %d, Charging USB Host\n", type);
		break;
	case NONSTANDARD_CHARGER:
		pr_info("charger type: %d, Non-standard Charger\n", type);
		break;
	case STANDARD_CHARGER:
		pr_info("charger type: %d, Standard Charger\n", type);
		break;
	case APPLE_2_1A_CHARGER:
		pr_info("charger type: %d, APPLE_2_1A_CHARGER\n", type);
		break;
	case APPLE_1_0A_CHARGER:
		pr_info("charger type: %d, APPLE_1_0A_CHARGER\n", type);
		break;
	case APPLE_0_5A_CHARGER:
		pr_info("charger type: %d, APPLE_0_5A_CHARGER\n", type);
		break;
	default:
		pr_info("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}

int hw_charging_get_charger_type(void)
{
	enum charger_type CHR_Type_num = CHARGER_UNKNOWN;
	int i_ACA_type = 0;

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
	if (usb2jtag_mode()) {
		pr_info("[USB2JTAG] in usb2jtag mode, skip charger detection\n");
		return STANDARD_HOST;
	}
#endif

	hw_bc11_init();

	if (hw_bc11_DCD()) {
		if (hw_bc11_stepA1())
			CHR_Type_num = APPLE_2_1A_CHARGER;
		else
			CHR_Type_num = NONSTANDARD_CHARGER;
	} else {
		if (hw_bc11_stepA2()) {
			if (hw_bc11_stepB2())
				CHR_Type_num = STANDARD_CHARGER;
			else
				CHR_Type_num = CHARGING_HOST;
		} else
			CHR_Type_num = STANDARD_HOST;
	}

	if (CHR_Type_num != STANDARD_CHARGER)
		hw_bc11_done();
	else
		pr_info("charger type: skip bc11 release for BC12 DCP SPEC\n");

	dump_charger_name(CHR_Type_num);

	i_ACA_type = hw_otg_get_aca_type();
	if (0x4 == i_ACA_type)
		CHR_Type_num = STANDARD_CHARGER;

#ifdef __FORCE_USB_TYPE__
	CHR_Type_num = STANDARD_HOST;
	pr_info("charger type: Froce to STANDARD_HOST\n");
#endif

	return CHR_Type_num;
}

/*****************************************************************************
 * Charger Detection
 ******************************************************************************/
void do_charger_detect(void)
{

#if THUB_SUPPORT
	int i_ACA_type = 0;
#endif //THUB_SUPPORT
	if (!mt_usb_is_device()) {
#if THUB_SUPPORT
		i_ACA_type = hw_otg_get_aca_type();
		if (0x4 == i_ACA_type) {
			g_chr_type = STANDARD_CHARGER;
			chrdet_inform_psy_changed(g_chr_type, 1);
		} else {
			g_chr_type = CHARGER_UNKNOWN;
			chrdet_inform_psy_changed(g_chr_type, 0);
		}
		pr_err("charger type: g_chr_type, Now is usb host mode. Skip detection\n");
#else
		g_chr_type = CHARGER_UNKNOWN;
		pr_err("charger type: UNKNOWN, Now is usb host mode. Skip detection!!!\n");
#endif
		return;
	}

	if (is_meta_mode()) {
		/* Skip charger type detection to speed up meta boot */
		pr_notice("charger type: force Standard USB Host in meta\n");
		if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
			g_chr_type = STANDARD_HOST;
			chrdet_inform_psy_changed(g_chr_type, 1);
		} else {
			g_chr_type = CHARGER_UNKNOWN;
			chrdet_inform_psy_changed(g_chr_type, 0);
		}
		return;
	}

	mutex_lock(&chrdet_lock);

	if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
		pr_info("charger type: charger IN\n");
		g_chr_type = hw_charging_get_charger_type();
		chrdet_inform_psy_changed(g_chr_type, 1);
	} else {
		pr_info("charger type: charger OUT\n");
		g_chr_type = CHARGER_UNKNOWN;
		chrdet_inform_psy_changed(g_chr_type, 0);
	}

	mutex_unlock(&chrdet_lock);
}



/*****************************************************************************
 * PMIC Int Handler
 ******************************************************************************/
void chrdet_int_handler(void)
{
	/*
	 * pr_info("[chrdet_int_handler]CHRDET status = %d....\n",
	 *	pmic_get_register_value(PMIC_RGS_CHRDET));
	 */
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!pmic_get_register_value(PMIC_RGS_CHRDET)) {
		int boot_mode = 0;

		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
		    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_info("[chrdet_int_handler] Unplug Charger/USB\n");
#ifndef CONFIG_TCPC_CLASS
			//orderly_poweroff(true);
#else
			return;
#endif
		}
	}
#endif
	pmic_set_register_value(PMIC_RG_USBDL_RST, 1);

	do_charger_detect();
}


/************************************************/
/* Charger Probe Related
*************************************************/
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static void do_charger_detection_work(struct work_struct *data)
{
	if (pmic_get_register_value(PMIC_RGS_CHRDET))
		do_charger_detect();
}
#endif


static int __init pmic_chrdet_init(void)
{
	mutex_init(&chrdet_lock);
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_err("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

#ifdef __SW_CHRDET_IN_PROBE_PHASE__
	/* do charger detect here to prevent HW miss interrupt*/
	INIT_WORK(&chr_work, do_charger_detection_work);
	schedule_work(&chr_work);
#endif

	pmic_register_interrupt_callback(INT_CHRDET_EDGE, chrdet_int_handler);
	pmic_enable_interrupt(INT_CHRDET_EDGE, 1, "PMIC");

	return 0;
}

late_initcall(pmic_chrdet_init);

#endif
