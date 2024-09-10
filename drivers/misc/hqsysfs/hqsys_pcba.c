#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mt-plat/mtk_devinfo.h>
#include <linux/of_gpio.h>
#include "hqsys_pcba.h"

#define BOARD_ID0 3
#define BOARD_ID1 4
#define BOARD_ID2 5
#define BOARD_ID3 27

PCBA_CONFIG huaqin_pcba_config=PCBA_UNKNOW;

typedef struct {
	int boradid;
	PCBA_CONFIG version;
}board_id_map_t;
static const board_id_map_t board_id_map[] =
{
	{0x0,PCBA_M7_LTE_V1},
	{0x1,PCBA_M7_LTE_V2},
	{0x2,PCBA_M7_LTE_V3},
	{0x3,PCBA_M7_LTE_V4},
	{0x4,PCBA_M7_LTE_V5}
};
static unsigned int board_id0_pin;
static unsigned int board_id1_pin;
static unsigned int board_id2_pin;
static unsigned int board_id3_pin;

static void get_board_id_pin(void)
{
	struct device_node *devnode;

	devnode = of_find_compatible_node(NULL, NULL, "mediatek, board_id_pin");
	if (devnode){
		board_id0_pin = of_get_named_gpio(devnode, "board_id0_pin", 0);
		board_id1_pin = of_get_named_gpio(devnode, "board_id1_pin", 0);
		board_id2_pin = of_get_named_gpio(devnode, "board_id2_pin", 0);
		board_id3_pin = of_get_named_gpio(devnode, "board_id3_pin", 0);
	}
	else {
		printk(KERN_WARNING "get_board_id_pin failed\n");
	}
}

static bool read_pcba_config(void)
{
	int board_id = 0;
	int ret = 0;
	int i=0,map_size=0;

	get_board_id_pin();
	ret = gpio_get_value(board_id3_pin);
	/*if (ret < 0)
	{
		huaqin_pcba_config=PCBA_UNKNOW;
		printk(KERN_WARNING "gpio_get_value board_id3 error: %d\n", ret);
		return false;
	}*/
	printk(KERN_INFO "gpio_get_value board_id3 %d value: %d\n", board_id3_pin, ret);
	//board_id = ret*8;
	ret = gpio_get_value(board_id0_pin);
	if (ret < 0)
	{
		huaqin_pcba_config=PCBA_UNKNOW;
		printk(KERN_WARNING "gpio_get_value board_id0 error: %d\n", ret);
		return false;
	}
	printk(KERN_INFO "gpio_get_value board_id0 %d value: %d\n", board_id0_pin, ret);
	board_id += ret*4;
	ret = gpio_get_value(board_id1_pin);
	if (ret < 0)
	{
		huaqin_pcba_config=PCBA_UNKNOW;
		printk(KERN_WARNING "gpio_get_value board_id1 error: %d\n", ret);
		return false;
	}
	printk(KERN_INFO "gpio_get_value board_id1 %d value: %d\n", board_id1_pin, ret);
	board_id += ret*2;
	ret = gpio_get_value(board_id2_pin);
	if (ret < 0)
	{
		huaqin_pcba_config=PCBA_UNKNOW;
		printk(KERN_WARNING "gpio_get_value board_id2 error: %d\n", ret);
		return false;
	}
	printk(KERN_INFO "gpio_get_value board_id2 %d value: %d\n", board_id2_pin, ret);
	board_id += ret;
	/*Cause we only have just one version,so its just one version */
	map_size = sizeof(board_id_map)/sizeof(board_id_map_t);
	while(i < map_size)
	{
		if(board_id == board_id_map[i].boradid)
		{
			huaqin_pcba_config = board_id_map[i].version;
			break;
		}
		i++;
	}
	if(i >= map_size)
	{
		huaqin_pcba_config = PCBA_UNKNOW;
	}
	printk(KERN_INFO "%s pcba_config:%d i:%d \n",__func__,huaqin_pcba_config,i);
	return true;
}
PCBA_CONFIG get_huaqin_pcba_config(void)
{
	return huaqin_pcba_config;
}
EXPORT_SYMBOL_GPL(get_huaqin_pcba_config);


static int __init huaqin_pcba_early_init(void)
{
	read_pcba_config();
	return 0;
}

module_init(huaqin_pcba_early_init);//before device_initcall

//late_initcall(huaqin_pcba_module_init);   //late initcall

MODULE_AUTHOR("wangqi<wangqi6@huaqin.com>");
MODULE_DESCRIPTION("huaqin sys pcba");
MODULE_LICENSE("GPL");
