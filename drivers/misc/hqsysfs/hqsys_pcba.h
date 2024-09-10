#ifndef HQSYS_PCBA
#define HQSYS_PCBA


typedef enum
{
	PCBA_UNKNOW     =0,
	PCBA_M7_3G_V1   =0x20,
	PCBA_M7_WIFI_V1 =0x21,
	PCBA_M7_3G_V2   =0x22,
	PCBA_M7_WIFI_V2 =0x23,
	PCBA_M7_3G_V3   =0x24,
	PCBA_M7_WIFI_V3 =0x25,
	PCBA_M7_3G_V4   =0x26,
	PCBA_M7_WIFI_V4 =0x27,
	PCBA_M7_3G_V5   =0x28,
	PCBA_M7_WIFI_V5 =0x29,
	PCBA_M7_LTE_V1  =0x30,
	PCBA_M7_LTE_V2  =0x31,
	PCBA_M7_LTE_V3  =0x32,
	PCBA_M7_LTE_V4  =0x33,
	PCBA_M7_LTE_V5  =0x34,
	PCBA_END        =0x40,

}PCBA_CONFIG;

extern PCBA_CONFIG huaqin_pcba_config;

struct pcba_info
{
	PCBA_CONFIG pcba_config;
	char pcba_name[32];
};

PCBA_CONFIG get_huaqin_pcba_config(void);

#endif
