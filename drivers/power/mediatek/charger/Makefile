#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-$(CONFIG_USB_MU3D_DRV) += -Werror -I$(srctree)/drivers/misc/mediatek/mu3d/drv
subdir-ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb_c/mt6336
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb_c/tcpc/inc
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6335)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6335
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6355)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6356)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6357)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6357
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6358)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6358
subdir-ccflags-y += -I$(srctree)/drivers/power/mediatek/misc/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6336
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include

obj-$(CONFIG_MTK_SMART_BATTERY) += charger_class.o mtk_charger.o mtk_pdc_intf.o mtk_pe40_intf.o\
				mtk_switch_charging.o mtk_linear_charging.o
ifneq ($(CONFIG_MTK_PMIC_CHIP_MT6336),y)
obj-$(CONFIG_MTK_SMART_BATTERY) += mtk_chg_type_det.o
endif

obj-$(CONFIG_MTK_DUAL_CHARGER_SUPPORT) += mtk_dual_switch_charging.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) += mtk_pe_intf.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) += mtk_pe20_intf.o
obj-$(CONFIG_MTK_PUMP_EXPRESS_PLUS_30_SUPPORT) += mtk_pe30_intf.o
obj-$(CONFIG_MTK_PMIC_CHIP_MT6336) += mt6336_charger.o
obj-$(CONFIG_LOAD_SWITCH_RT9750) += rt9750.o
obj-$(CONFIG_CHARGER_RT9465) += rt9465.o
obj-$(CONFIG_CHARGER_RT9466) += rt9466.o
obj-$(CONFIG_CHARGER_RT9467) += rt9467.o
obj-$(CONFIG_CHARGER_RT9458) += rt9458.o
obj-$(CONFIG_MTK_BQ25890_SUPPORT) += bq25890.o
obj-$(CONFIG_MTK_BQ2560X_SUPPORT) += bq2560x_charger.o
obj-$(CONFIG_CHARGER_HL7005) += hl7005.o
