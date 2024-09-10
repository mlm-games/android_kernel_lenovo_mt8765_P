/*
 * stk832x_driver.c - Linux driver for sensortek stk832x accelerometer
 * Copyright (C) 2017 Sensortek
 */
#if (defined CONFIG_OF && defined CONFIG_MTK_SENSOR_SUPPORT)
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>

#include <accel.h>
#include <linux/hqsysfs.h>
#include "cust_acc.h"
#include "sensors_io.h"

/********* Global *********/
//#define STK_INTERRUPT_MODE
#define STK_POLLING_MODE
//#define STK_AMD /* Turn ON any motion */
//#define STK_HAND_DETECT /* Turn ON  hand detect feature */
//#define STK_STEP_COUNTER /* Turn on step counter */
//#define STK_CHECK_CODE /* enable check code feature */
//#define STK_CALI /* Turn on sensortek calibration feature */
#define STK_FIR /* low-pass mode */
//#define STK_ZG
    /* enable Zero-G simulation.
     * This feature only works when both of STK_FIR and STK_ZG are turn On. */

/* Any motion only works under either STK_INTERRUPT_MODE or STK_POLLING_MODE */
#ifdef STK_AMD
    #if !defined STK_INTERRUPT_MODE && !defined STK_POLLING_MODE
        #undef STK_AMD
    #endif /* !defined STK_INTERRUPT_MODE && !defined STK_POLLING_MODE*/
#endif /* STK_AMD */

/* Hand detect only works under STK_AMD */
#ifdef STK_HAND_DETECT
    #if !defined STK_AMD
        #undef STK_HAND_DETECT
    #endif /* !defined STK_AMD */
#endif /* STK_HAND_DETECT */
#ifdef STK_HAND_DETECT
    #include "../stkAlgorithm/hand_detect/stk_hand_detect.h"
    #define STK_HAND_DETECT_CNT_THRESHOLD   12
#endif /* STK_HAND_DETECT */

#ifdef STK_STEP_COUNTER
    #include "../../step_counter/step_counter.h"
    #define STK_STEP_C_DEV_NAME "stk832x_step_c"
    static DEFINE_MUTEX(stk_step_init_mutex);
    static struct step_c_init_info stk_step_c_init_info;
#endif /* STK_STEP_COUNTER */

#ifdef STK_CHECK_CODE
    /* Ignore the first STK_CHECKCODE_IGNORE+1 data for STK_CHECK_CODE feature */
    #define STK_CHECKCODE_IGNORE    3
    /* for stk832x_data.cc_status */
    #define STK_CCSTATUS_NORMAL     0x0
    #define STK_CCSTATUS_ZSIM       0x1
    #define STK_CCSTATUS_XYSIM      0x2
#endif /* STK_CHECK_CODE */

#ifdef STK_FIR
    #define STK_FIR_LEN         2
    #define STK_FIR_LEN_MAX     32
    struct data_fir
    {
        s16 xyz[STK_FIR_LEN_MAX][3];
        int sum[3];
        int idx;
        int count;
    };
#endif /* STK_FIR */

#if (defined STK_FIR && defined STK_ZG)
    #define ZG_FACTOR   0
#endif /* defined STK_FIR && defined STK_ZG */

#define STK_ACC_DRIVER_VERSION "404.3.1"
#define STK_ACC_DEV_NAME "stk832x"
#define STK_ACC_DTS_NAME "mediatek,gsensor_stk832x"
#ifdef STK_INTERRUPT_MODE
    #define STK832X_IRQ_INT1_NAME "stk832x_int1"
#endif /* STK_INTERRUPT_MODE */

#define STK_ACC_TAG                 "[accel]"
#define STK_ACC_FUN(f)              printk(KERN_INFO STK_ACC_TAG" %s\n", __FUNCTION__)
#define STK_ACC_ERR(fmt, args...)   printk(KERN_ERR STK_ACC_TAG" %s %d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define STK_ACC_LOG(fmt, args...)   printk(KERN_INFO STK_ACC_TAG" %s %d: "fmt"\n", __FUNCTION__, __LINE__, ##args)

/*****************************************************************************
 * stk832x register, start
 *****************************************************************************/
/* stk832x slave address */
//#define STK832X_SLAVE_ADDR          0x1F
#define STK832X_SLAVE_ADDR          0x0F

/* stk832x register */
#define STK832X_REG_CHIPID          0x00
#define STK832X_REG_XOUT1           0x02
#define STK832X_REG_XOUT2           0x03
#define STK832X_REG_YOUT1           0x04
#define STK832X_REG_YOUT2           0x05
#define STK832X_REG_ZOUT1           0x06
#define STK832X_REG_ZOUT2           0x07
#define STK832X_REG_INTSTS1         0x09
#define STK832X_REG_INTSTS2         0x0A
#define STK832X_REG_STEPOUT1        0x0D
#define STK832X_REG_STEPOUT2        0x0E
#define STK832X_REG_RANGESEL        0x0F
#define STK832X_REG_BWSEL           0x10
#define STK832X_REG_POWMODE         0x11
#define STK832X_REG_SWRST           0x14
#define STK832X_REG_INTEN1          0x16
#define STK832X_REG_INTEN2          0x17
#define STK832X_REG_INTMAP1         0x19
#define STK832X_REG_INTMAP2         0x1A
#define STK832X_REG_INTCFG1         0x20
#define STK832X_REG_INTCFG2         0x21
#define STK832X_REG_SLOPEDLY        0x27
#define STK832X_REG_SLOPETHD        0x28
#define STK832X_REG_SIGMOT1         0x29
#define STK832X_REG_SIGMOT2         0x2A
#define STK832X_REG_SIGMOT3         0x2B
#define STK832X_REG_STEPCNT1        0x2C
#define STK832X_REG_STEPCNT2        0x2D
#define STK832X_REG_STEPTHD         0x2E
#define STK832X_REG_STEPDEB         0x2F
#define STK832X_REG_STEPMAXTW       0x31
#define STK832X_REG_INTFCFG         0x34
#define STK832X_REG_OFSTCOMP1       0x36
#define STK832X_REG_OFSTX           0x38
#define STK832X_REG_OFSTY           0x39
#define STK832X_REG_OFSTZ           0x3A
#define STK832X_REG_CFG1            0x3D
#define STK832X_REG_CFG2            0x3E
#define STK832X_REG_FIFOOUT         0x3F

/* STK832X_REG_CHIPID */
#define STK8323_ID                      0x23 /* include for STK8321 */
#define STK8325_ID                      0x25

/* STK832X_REG_INTSTS1 */
#define STK832X_INTSTS1_SIG_MOT_STS     0x1
#define STK832X_INTSTS1_ANY_MOT_STS     0x4

/* STK832X_REG_INTSTS2 */
#define STK832X_INTSTS2_FWM_STS_MASK    0x40

/* STK832X_REG_RANGESEL */
#define STK832X_RANGESEL_2G             0x3
#define STK832X_RANGESEL_4G             0x5
#define STK832X_RANGESEL_8G             0x8
#define STK832X_RANGESEL_BW_MASK        0x1F
#define STK832X_RANGESEL_DEF            STK832X_RANGESEL_2G
typedef enum
{
    STK_2G = STK832X_RANGESEL_2G,
    STK_4G = STK832X_RANGESEL_4G,
    STK_8G = STK832X_RANGESEL_8G
} stk_rangesel;

/* STK832X_REG_BWSEL */
#define STK832X_BWSEL_INIT_ODR          0x0A    /* ODR = BW x 2 = 62.5Hz */
/* ODR: 31.25, 62.5, 125 */
const static int STK832X_SAMPLE_TIME[] = {32000, 16000, 8000}; /* usec */
#define STK832X_SPTIME_BASE             0x9     /* for 32000, ODR:31.25 */
#define STK832X_SPTIME_BOUND            0xB     /* for 8000, ODR:125 */

/* STK832X_REG_POWMODE */
#define STK832X_PWMD_SUSPEND            0x80
#define STK832X_PWMD_LOWPOWER           0x40
#define STK832X_PWMD_NORMAL             0x00
#define STK832X_PWMD_SLP_MASK           0x3E

/* STK832X_REG_SWRST */
#define STK832X_SWRST_VAL               0xB6

/* STK832X_REG_INTEN1 */
#define STK832X_INTEN1_SLP_EN_XYZ       0x07

/* STK832X_REG_INTEN2 */
#define STK832X_INTEN2_DATA_EN          0x10
#define STK832X_INTEN2_FWM_EN           0x40

/* STK832X_REG_INTMAP1 */
#define STK832X_INTMAP1_SIGMOT2INT1         0x01
#define STK832X_INTMAP1_ANYMOT2INT1         0x04

/* STK832X_REG_INTMAP2 */
#define STK832X_INTMAP2_DATA2INT1           0x01
#define STK832X_INTMAP2_FWM2INT1            0x02
#define STK832X_INTMAP2_FWM2INT2            0x40

/* STK832X_REG_INTCFG1 */
#define STK832X_INTCFG1_INT1_ACTIVE_H       0x01
#define STK832X_INTCFG1_INT1_OD_PUSHPULL    0x00
#define STK832X_INTCFG1_INT2_ACTIVE_H       0x04
#define STK832X_INTCFG1_INT2_OD_PUSHPULL    0x00

/* STK832X_REG_INTCFG2 */
#define STK832X_INTCFG2_NOLATCHED           0x00
#define STK832X_INTCFG2_LATCHED             0x0F
#define STK832X_INTCFG2_INT_RST             0x80

/* STK832X_REG_SLOPETHD */
#define STK832X_SLOPETHD_DEF                0x14

/* STK832X_REG_SIGMOT1 */
#define STK832X_SIGMOT1_SKIP_TIME_3SEC      0x96    /* default value */

/* STK832X_REG_SIGMOT2 */
#define STK832X_SIGMOT2_SIG_MOT_EN          0x02
#define STK832X_SIGMOT2_ANY_MOT_EN          0x04

/* STK832X_REG_SIGMOT3 */
#define STK832X_SIGMOT3_PROOF_TIME_1SEC     0x32    /* default value */

/* STK832X_REG_INTFCFG */
#define STK832X_INTFCFG_I2C_WDT_EN          0x04

/* STK832X_REG_STEPCNT2 */
#define STK832X_STEPCNT2_RST_CNT            0x04
#define STK832X_STEPCNT2_STEP_CNT_EN        0x08

/* STK832X_REG_OFSTCOMP1 */
#define STK832X_OFSTCOMP1_OFST_RST          0x80

/* STK832X_REG_CFG1 */
/* the maximum space for FIFO is 32*3 bytes */
#define STK832X_CFG1_XYZ_FRAME_MAX      32

/* STK832X_REG_CFG2 */
#define STK832X_CFG2_FIFO_MODE_BYPASS       0x0
#define STK832X_CFG2_FIFO_MODE_FIFO         0x1
#define STK832X_CFG2_FIFO_MODE_SHIFT        5
#define STK832X_CFG2_FIFO_DATA_SEL_XYZ      0x0
#define STK832X_CFG2_FIFO_DATA_SEL_X        0x1
#define STK832X_CFG2_FIFO_DATA_SEL_Y        0x2
#define STK832X_CFG2_FIFO_DATA_SEL_Z        0x3
#define STK832X_CFG2_FIFO_DATA_SEL_MASK     0x3

/* STK832X_REG_OFSTx */
#define STK832X_OFST_LSB                    128     /* 8 bits for +-1G */
/*****************************************************************************
 * stk832x register, end
 *****************************************************************************/

/* axis */
#define STK_AXIS_X                  0
#define STK_AXIS_Y                  1
#define STK_AXIS_Z                  2
#define STK_AXES_NUM                3

/* file_operateions parameters */
#define STK_BUFSIZE                 60

#ifdef STK_CALI
/* calibration parameters */
#define STK_CALI_SAMPLE_NO          10
#define STK_CALI_VER0               0x18
#define STK_CALI_VER1               0x03
#define STK_CALI_END                '\0'
#define STK_CALI_FILE               "/data/misc/sensors/stkacccali.conf"
#define STK_CALI_FILE_SIZE          25
/* parameter for cali_status/atomic_t and cali file */
#define STK_K_SUCCESS_FILE          0x01
/* parameter for cali_status/atomic_t */
#define STK_K_FAIL_WRITE_OFST       0xF2
#define STK_K_FAIL_I2C              0xF8
#define STK_K_FAIL_W_FILE           0xFB
#define STK_K_FAIL_VERIFY_CALI      0xFD
#define STK_K_RUNNING               0xFE
#define STK_K_NO_CALI               0xFF
#endif /* STK_CALI */

/* selftest usage */
#define STK_SELFTEST_SAMPLE_NUM             100
#define STK_SELFTEST_RESULT_NA              0
#define STK_SELFTEST_RESULT_RUNNING         (1 << 0)
#define STK_SELFTEST_RESULT_NO_ERROR        (1 << 1)
#define STK_SELFTEST_RESULT_DRIVER_ERROR    (1 << 2)
#define STK_SELFTEST_RESULT_FAIL_X          (1 << 3)
#define STK_SELFTEST_RESULT_FAIL_Y          (1 << 4)
#define STK_SELFTEST_RESULT_FAIL_Z          (1 << 5)
#define STK_SELFTEST_RESULT_NO_OUTPUT       (1 << 6)

static inline int stk_selftest_offset_factor(int sen)
{
    return sen * 3 / 10;
}
static inline int stk_selftest_noise_factor(int sen)
{
    return sen / 10;
}

struct stk832x_data
{
    /* platform related */
    struct i2c_client           *client;
    struct acc_hw               hw;
    struct hwmsen_convert       cvt;
    /* chip informateion */
    int                         pid;
    /* system operation */
    atomic_t                    enabled;            /* chip is enabled or not */
    atomic_t                    enabled_for_acc;    /* enable status for acc_control_path.enable_nodata */
#ifdef STK_CALI
    atomic_t                    cali_status;        /* cali status */
#endif /* STK_CALI */
    int                         cali_sw[STK_AXES_NUM];  /* cali data */
    atomic_t                    recv;               /* recv data. DEVICE_ATTR(recv, ...) */
    struct mutex                reg_lock;           /* mutex lock for register R/W */
    u8                          power_mode;
    int                         sensitivity;        /* sensitivity, bit number per G */
    u8                          odr_no;
    s16                         xyz[3];             /* The latest data of xyz */
    atomic_t                    selftest;           /* selftest result */
#ifdef STK_STEP_COUNTER
    int                         steps;              /* The latest step counter value */
#endif /* STK_STEP_COUNTER */

#ifdef STK_INTERRUPT_MODE
    int                         interrupt_int1_pin; /* get from device tree */
    int                         irq1;               /* for all data usage(DATA, FIFO, ANYMOTION) */
    struct workqueue_struct     *alldata_workqueue; /* all data workqueue for int1. (DATA, FIFO, ANYMOTION) */
    struct work_struct          alldata_work;       /* all data work for int1. (DATA, FIFO, ANYMOTION) */
#elif defined STK_POLLING_MODE
    struct delayed_work         accel_delaywork;
    struct hrtimer              accel_timer;
    ktime_t                     poll_delay;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

#ifdef STK_HAND_DETECT
    struct delayed_work         hd_delaywork;      /* hand detect delay work. */
    atomic_t                    hd_cnt;
#endif /* STK_HAND_DETECT */

#ifdef STK_CHECK_CODE
    int                         cc_count;
    u8                          cc_status;          /* refer STK_CCSTATUS_x */
#endif /* STK_CHECK_CODE */

#ifdef STK_FIR
    struct data_fir             fir;
    /*
     * fir_len
     * 0: turn OFF FIR operation
     * 1 ~ STK_FIR_LEN_MAX: turn ON FIR operation
     */
    atomic_t                    fir_len;
#endif /* STK_FIR */
};

static int stk_reg_init(struct stk832x_data *stk, stk_rangesel range, u8 odr_no);

static struct stk832x_data *stk_data = NULL;
static int stk832x_init_flag = 0;

static int stk_acc_init(void);
static int stk_acc_uninit(void);

static struct acc_init_info stk_acc_init_info =
{
    .name = STK_ACC_DEV_NAME,
    .init = stk_acc_init,
    .uninit = stk_acc_uninit,
};

/********* Functions *********/
/**
 * stk832x register write
 * @brief: Register writing via I2C
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] reg: Register address
 * @param[in] val: Data, what you want to write.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_reg_write(struct stk832x_data *stk, u8 reg, u8 val)
{
    int error = 0;

    mutex_lock(&stk->reg_lock);
    error = i2c_smbus_write_byte_data(stk->client, reg, val);
    mutex_unlock(&stk->reg_lock);

    if (error)
    {
        STK_ACC_ERR("transfer failed to write reg:0x%x with val:0x%x, error=%d\n", reg, val, error);
    }

    return error;
}

/**
 * stk832x register read
 * @brief: Register reading via I2C
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] reg: Register address
 * @param[in] len: 0/1, for normal usage. Others, read length (FIFO used).
 * @param[out] val: Data, the register what you want to read.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_reg_read(struct stk832x_data *stk, u8 reg, int len, u8 *val)
{
    int error = 0;
    struct i2c_msg msgs[2] = {
        {
            .addr = stk->client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg
        },
        {
            .addr = stk->client->addr,
            .flags = I2C_M_RD,
            .len = (0 >= len) ? 1 : len,
            .buf = val
        }
    };

    mutex_lock(&stk->reg_lock);
    error = i2c_transfer(stk->client->adapter, msgs, 2);
    mutex_unlock(&stk->reg_lock);

    if (2 == error)
        error = 0;
    else if (0 > error)
    {
        STK_ACC_ERR("transfer failed to read reg:0x%x with len:%d, error=%d\n", reg, len, error);
    }
    else
    {
        STK_ACC_ERR("size error in reading reg:0x%x with len:%d, error=%d\n", reg, len, error);
        error = -1;
    }

    return error;
}

/**
 * @brief: Read PID and write to stk832x_data.pid.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_get_pid(struct stk832x_data *stk)
{
    int error = 0;
    u8 val = 0;
    error = stk832x_reg_read(stk, STK832X_REG_CHIPID, 0, &val);

    if (error)
        STK_ACC_ERR("failed to read PID");
    else
        stk->pid = (int)val;

    return error;
}

/**
 * @brief: Initialize some data in stk832x_data.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_data_initialize(struct stk832x_data *stk)
{
    atomic_set(&stk->enabled, 0);
    atomic_set(&stk->enabled_for_acc, 0);
#ifdef STK_CALI
    atomic_set(&stk->cali_status, STK_K_NO_CALI);
#endif /* STK_CALI */
    memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));
    atomic_set(&stk->recv, 0);
    atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NA);
    stk->power_mode = STK832X_PWMD_NORMAL;
    stk->odr_no = STK832X_BWSEL_INIT_ODR;
#ifdef STK_FIR
    memset(&stk->fir, 0, sizeof(struct data_fir));
    atomic_set(&stk->fir_len, STK_FIR_LEN);
#endif /* STK_FIR */
#ifdef STK_HAND_DETECT
    atomic_set(&stk->hd_cnt, 0);
#endif /* STK_HAND_DETECT */
    //STK_ACC_LOG("done");
}

/**
 * @brief: SW reset for stk832x
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_sw_reset(struct stk832x_data *stk)
{
    int error = 0;
    error = stk832x_reg_write(stk, STK832X_REG_SWRST, STK832X_SWRST_VAL);

    if (error)
        return error;

    usleep_range(1000, 2000);
    stk->power_mode = STK832X_PWMD_NORMAL;
    atomic_set(&stk->enabled, 1);

    return 0;
}

/**
 * @brief: Change power mode
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] pwd_md: power mode for STK832X_REG_POWMODE
 *              STK832X_PWMD_SUSPEND
 *              STK832X_PWMD_LOWPOWER
 *              STK832X_PWMD_NORMAL
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_change_power_mode(struct stk832x_data *stk, u8 pwd_md)
{
    if (pwd_md != stk->power_mode)
    {
        int error = 0;
        u8 val = 0;
        error = stk832x_reg_read(stk, STK832X_REG_POWMODE, 0, &val);

        if (error)
            return error;

        val &= STK832X_PWMD_SLP_MASK;
        error = stk832x_reg_write(stk, STK832X_REG_POWMODE, (val | pwd_md));

        if (error)
            return error;

        stk->power_mode = pwd_md;
    }
    else
        STK_ACC_LOG("Same as original power mode: 0x%X\n", stk->power_mode);

    return 0;
}

/**
 * @brief: Get sensitivity. Set result to stk832x_data.sensitivity.
 *          sensitivity = number bit per G (LSB/g)
 *          Example: RANGESEL=8g, 12 bits for STK832x full resolution
 *          Ans: number bit per G = 2^12 / (8x2) = 256 (LSB/g)
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_get_sensitivity(struct stk832x_data *stk)
{
    u8 val = 0;
    stk->sensitivity = 0;

    if ( 0 == stk832x_reg_read(stk, STK832X_REG_RANGESEL, 0, &val))
    {
        val &= STK832X_RANGESEL_BW_MASK;

        switch (val)
        {
            case STK832X_RANGESEL_2G:
                stk->sensitivity = 1024;
                break;

            case STK832X_RANGESEL_4G:
                stk->sensitivity = 512;
                break;

            case STK832X_RANGESEL_8G:
                stk->sensitivity = 256;
                break;

            default:
                break;
        }
    }
}

/**
 * @brief: Set range
 *          1. Setting STK832X_REG_RANGESEL
 *          2. Calculate sensitivity and store to stk832x_data.sensitivity
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] range: stk_rangesel for STK832X_REG_RANGESEL
 *              STK_2G
 *              STK_4G
 *              STK_8G
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_range_selection(struct stk832x_data *stk, stk_rangesel range)
{
    int result = 0;
    result = stk832x_reg_write(stk, STK832X_REG_RANGESEL, range);

    if (result)
        return result;

    stk_get_sensitivity(stk);
    return 0;
}

/**
 * stk_set_enable
 * @brief: Turn ON/OFF the power state of stk832x.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] en: turn ON/OFF
 *              0 for suspend mode;
 *              1 for normal mode.
 */
static void stk_set_enable(struct stk832x_data *stk, char en)
{
    int error = 0;
    u8 val = 0;

    if (en == atomic_read(&stk->enabled))
        return;

    error = stk832x_reg_read(stk, STK832X_REG_POWMODE, 0, &val);

    if (error)
    {
        STK_ACC_ERR("failed read STK832X_REG_POWMODE");
        return;
    }

    /* ID46 */
    if (val & STK832X_PWMD_LOWPOWER)
    {
        stk_rangesel range = STK_2G;

        switch (stk->sensitivity)
        {
            case 512:
                range = STK_4G;
                break;
            case 256:
                range = STK_8G;
                break;
            default:
                range = STK_2G;
                break;
        }

        stk_reg_init(stk, range, stk->odr_no);
    }

    if (en)
    {
        if (stk_change_power_mode(stk, STK832X_PWMD_NORMAL))
            return;

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
        hrtimer_start(&stk->accel_timer, stk->poll_delay, HRTIMER_MODE_REL);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
#ifdef STK_CHECK_CODE
        stk->cc_count = 0;
        stk->cc_status = STK_CCSTATUS_NORMAL;
#endif /* STK_CHECK_CODE */
    }
    else
    {
        if (stk_change_power_mode(stk, STK832X_PWMD_SUSPEND))
            return;

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
        hrtimer_cancel(&stk->accel_timer);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    }

    atomic_set(&stk->enabled, en);
}

/**
 * @brief: Get delay
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: delay in usec
 *          Please refer STK832X_SAMPLE_TIME[]
 */
static int stk_get_delay(struct stk832x_data *stk)
{
    u8 data = 0;
    int delay_us = 0;

    if (stk832x_reg_read(stk, STK832X_REG_BWSEL, 0, &data))
    {
        STK_ACC_ERR("failed to read delay");
    }
    else if ((STK832X_SPTIME_BASE > data) || (STK832X_SPTIME_BOUND < data))
    {
        STK_ACC_ERR("BW out of range, 0x%X", data);
    }
    else
    {
        delay_us = STK832X_SAMPLE_TIME[data - STK832X_SPTIME_BASE];
    }

    return delay_us;
}

/**
 * @brief: Set delay
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_set_delay(struct stk832x_data *stk, int delay_us)
{
    int error = 0;
    bool enable = false;
    unsigned char sr_no;

    for (sr_no = 0; sr_no <= STK832X_SPTIME_BOUND - STK832X_SPTIME_BASE;
         sr_no++)
        if (delay_us >= STK832X_SAMPLE_TIME[sr_no])
            break;

    if (sr_no == STK832X_SPTIME_BOUND - STK832X_SPTIME_BASE + 1)
    {
        sr_no--;
        //delay_us = STK832X_SAMPLE_TIME[sr_no];
    }

    stk->odr_no = (u8)sr_no;
    sr_no += STK832X_SPTIME_BASE;

    if (atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 0);
        enable = true;
    }

    error = stk832x_reg_write(stk, STK832X_REG_BWSEL, sr_no);

    if (error)
        STK_ACC_ERR("failed to change ODR");

    if (enable)
    {
        stk_set_enable(stk, 1);
    }

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
    stk->poll_delay = ns_to_ktime(
                          STK832X_SAMPLE_TIME[sr_no - STK832X_SPTIME_BASE] * NSEC_PER_USEC);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    return error;
}

#ifdef STK_CHECK_CODE
/**
 * @brief: check stiction or not
 *          If 3 times and continue stiction will change stk832x_data.cc_status
 *          to STK_CCSTATUS_ZSIM or STK_CCSTATUS_XYZIM.
 *          Others, keep stk832x_data.cc_status to STK_CCSTATUS_NORMAL.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] clean: clean internal flag of check_result or not.
 *                  true: clean check_result
 *                  false: don't clean check_result
 */
static void stk_check_data(struct stk832x_data *stk, bool clean)
{
    static s8 event_no = 0;
    static s8 check_result = 0;
    /* 12 bits per axis */
    const int max_value = 2047;
    const int min_value = -2048;

    if (18 <= event_no)
        return;

    if (max_value == stk->xyz[0] || min_value == stk->xyz[0]
        || max_value == stk->xyz[1] || min_value == stk->xyz[1]
        || max_value == stk->xyz[2] || min_value == stk->xyz[2])
    {
        STK_ACC_LOG("acc:0x%X, 0x%X, 0x%X\n", stk->xyz[0], stk->xyz[1], stk->xyz[2]);
        check_result++;
    }
    else
    {
        check_result = 0;
        goto exit;
    }

    if (clean)
    {
        if (3 <= check_result)
        {
            if (max_value != stk->xyz[0] && min_value != stk->xyz[0]
                && max_value != stk->xyz[1] && min_value != stk->xyz[1])
                stk->cc_status = STK_CCSTATUS_ZSIM;
            else
                stk->cc_status = STK_CCSTATUS_XYSIM;

            STK_ACC_LOG("incorrect reading");
        }

        check_result = 0;
    }

exit:
    event_no++;
    return;
}

/**
 * @brief: sqrt operation
 *
 * @return sqrt(in)
 */
static int stk_sqrt(int in)
{
    int root, bit;
    root = 0;
    for (bit = 0x4000; bit > 0; bit >>= 2)
    {
        int trial = root + bit;
        root >>= 1;
        if (trial <= in)
        {
            root += bit;
            in -= trial;
        }
    }
    return root;
}

/**
 * @brief: check_code operation
 *          z = sqrt(x^2 + y^2)
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_check_code(struct stk832x_data *stk)
{
    u16 x, y;
    int sen;
    sen = stk->sensitivity;

    if (0 <= stk->xyz[0])
        x = stk->xyz[0];
    else
        x = -stk->xyz[0];

    if (0 <= stk->xyz[1])
        y = stk->xyz[1];
    else
        y = -stk->xyz[1];

    if ((x >= sen) || (y >= sen))
    {
        stk->xyz[2] = 0;
        return;
    }

    stk->xyz[2] = stk_sqrt(sen * sen - x * x - y * y);
}
#endif /* STK_CHECK_CODE */

#ifdef STK_FIR
/**
 * @brief: low-pass filter operation
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_low_pass_fir(struct stk832x_data *stk)
{
    int firlength = atomic_read(&stk->fir_len);
#ifdef STK_ZG
    s16 avg;
    int jitter_boundary = stk->sensitivity / 128;
#if 0

    if (0 == jitter_boundary)
        jitter_boundary = 1;

#endif
#endif /* STK_ZG */

    if (0 == firlength)
    {
        /* stk832x_data.fir_len == 0: turn OFF FIR operation */
        return;
    }

    if (firlength > stk->fir.count)
    {
        stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
        stk->fir.sum[0] += stk->xyz[0];
        stk->fir.sum[1] += stk->xyz[1];
        stk->fir.sum[2] += stk->xyz[2];
        stk->fir.count++;
        stk->fir.idx++;
    }
    else
    {
        if (firlength <= stk->fir.idx)
            stk->fir.idx = 0;

        stk->fir.sum[0] -= stk->fir.xyz[stk->fir.idx][0];
        stk->fir.sum[1] -= stk->fir.xyz[stk->fir.idx][1];
        stk->fir.sum[2] -= stk->fir.xyz[stk->fir.idx][2];
        stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
        stk->fir.sum[0] += stk->xyz[0];
        stk->fir.sum[1] += stk->xyz[1];
        stk->fir.sum[2] += stk->xyz[2];

        stk->fir.idx++;
#ifdef STK_ZG
        avg = stk->fir.sum[0] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[0] = avg * ZG_FACTOR;
        else
            stk->xyz[0] = avg;

        avg = stk->fir.sum[1] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[1] = avg * ZG_FACTOR;
        else
            stk->xyz[1] = avg;

        avg = stk->fir.sum[2] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[2] = avg * ZG_FACTOR;
        else
            stk->xyz[2] = avg;

#else /* STK_ZG */
        stk->xyz[0] = stk->fir.sum[0] / firlength;
        stk->xyz[1] = stk->fir.sum[1] / firlength;
        stk->xyz[2] = stk->fir.sum[2] / firlength;
#endif /* STK_ZG */
    }
}
#endif /* STK_FIR */

/**
 * @brief: read accel raw data from register.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_accel_rawdata(struct stk832x_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk832x_reg_read(stk, STK832X_REG_XOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_XOUT2, 0, &dataH))
        return;

    stk->xyz[0] = dataH << 8 | dataL;
    stk->xyz[0] >>= 4;

    if (stk832x_reg_read(stk, STK832X_REG_YOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_YOUT2, 0, &dataH))
        return;

    stk->xyz[1] = dataH << 8 | dataL;
    stk->xyz[1] >>= 4;

    if (stk832x_reg_read(stk, STK832X_REG_ZOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_ZOUT2, 0, &dataH))
        return;

    stk->xyz[2] = dataH << 8 | dataL;
    stk->xyz[2] >>= 4;
}

/**
 * @brief: read accel data from register.
 *          Store result to stk832x_data.xyz[].
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_accel_data(struct stk832x_data *stk)
{
    stk_read_accel_rawdata(stk);
#ifdef STK_CHECK_CODE

    if ((STK_CHECKCODE_IGNORE + 1) == stk->cc_count || (STK_CHECKCODE_IGNORE + 2) == stk->cc_count)
        stk_check_data(stk, false);
    else if ((STK_CHECKCODE_IGNORE + 3) == stk->cc_count)
        stk_check_data(stk, true);
    else if (STK_CCSTATUS_ZSIM == stk->cc_status)
        stk_check_code(stk);

    if ((STK_CHECKCODE_IGNORE + 6) > stk->cc_count)
        stk->cc_count++;

#endif /* STK_CHECK_CODE */
#ifdef STK_FIR
    stk_low_pass_fir(stk);
#endif /* STK_FIR */
}

/**
 * @brief: Selftest for XYZ offset and noise.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static char stk_testOffsetNoise(struct stk832x_data *stk)
{
    int read_delay_ms = 8; /* 125Hz = 8ms */
    int acc_ave[3] = {0, 0, 0};
    int acc_min[3] = {INT_MAX, INT_MAX, INT_MAX};
    int acc_max[3] = {INT_MIN, INT_MIN, INT_MIN};
    int noise[3] = {0, 0, 0};
    int sn = 0, axis = 0;
    int thresholdOffset, thresholdNoise;
    u8 localResult = 0;

    if (stk_sw_reset(stk))
        return -1;

    if (stk832x_reg_write(stk, STK832X_REG_BWSEL, 0x0B)) /* ODR = 125Hz */
        return -1;

    if (stk_range_selection(stk, STK832X_RANGESEL_2G))
        return -1;

    thresholdOffset = stk_selftest_offset_factor(stk->sensitivity);
    thresholdNoise = stk_selftest_noise_factor(stk->sensitivity);

    for (sn = 0; sn < STK_SELFTEST_SAMPLE_NUM; sn++)
    {
        msleep(read_delay_ms);
        stk_read_accel_rawdata(stk);
        STK_ACC_LOG("acc = %d, %d, %d", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

        for (axis = 0; axis < 3; axis++)
        {
            acc_ave[axis] += stk->xyz[axis];

            if (stk->xyz[axis] > acc_max[axis])
                acc_max[axis] = stk->xyz[axis];

            if (stk->xyz[axis] < acc_min[axis])
                acc_min[axis] = stk->xyz[axis];
        }
    }

    for (axis = 0; axis < 3; axis++)
    {
        acc_ave[axis] /= STK_SELFTEST_SAMPLE_NUM;
        noise[axis] = acc_max[axis] - acc_min[axis];
    }

    STK_ACC_LOG("acc_ave=%d, %d, %d, noise=%d, %d, %d",
            acc_ave[0], acc_ave[1], acc_ave[2], noise[0], noise[1], noise[2]);
    STK_ACC_LOG("offset threshold=%d, noise threshold=%d", thresholdOffset, thresholdNoise);

    if (0 < acc_ave[2])
        acc_ave[2] -= stk->sensitivity;
    else
        acc_ave[2] += stk->sensitivity;

    if (0 == acc_ave[0] && 0 == acc_ave[1] && 0 == acc_ave[2])
        localResult |= STK_SELFTEST_RESULT_NO_OUTPUT;

    if (thresholdOffset <= abs(acc_ave[0])
            || 0 == noise[0] || thresholdNoise <= noise[0])
        localResult |= STK_SELFTEST_RESULT_FAIL_X;

    if (thresholdOffset <= abs(acc_ave[1])
            || 0 == noise[1] || thresholdNoise <= noise[1])
        localResult |= STK_SELFTEST_RESULT_FAIL_Y;

    if (thresholdOffset <= abs(acc_ave[2])
            || 0 == noise[2] || thresholdNoise <= noise[2])
        localResult |= STK_SELFTEST_RESULT_FAIL_Z;

    if (0 == localResult)
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NO_ERROR);
    else
        atomic_set(&stk->selftest, localResult);
    return 0;
}

/**
 * @brief: SW selftest function.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_selftest(struct stk832x_data *stk)
{
    int i = 0;
    u8 data = 0;

    STK_ACC_FUN();

    atomic_set(&stk->selftest, STK_SELFTEST_RESULT_RUNNING);

    /* Check PID */
    if (stk_get_pid(stk))
    {
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
        return;
    }

    STK_ACC_LOG("PID 0x%x", stk->pid);

    if (STK8323_ID != stk->pid
            && STK8325_ID != stk->pid)
    {
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
        return;
    }

    /* Touch all register */
    for (i = 0; i <= 0x3F; i++)
    {
        if (stk832x_reg_read(stk, i, 0, &data))
        {
            atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
            return;
        }

        STK_ACC_LOG("[0x%2X]=0x%2X", i, data);
    }

    if (stk_testOffsetNoise(stk))
    {
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
    }

    stk_reg_init(stk, STK_2G, STK832X_BWSEL_INIT_ODR);
}

/**
 * @brief: Read all register (0x0 ~ 0x3F)
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] show_buffer: record all register value
 *
 * @return: buffer length or fail
 *          positive value: return buffer length
 *          -1: Fail
 */
static int stk_show_all_reg(struct stk832x_data *stk, char *show_buffer)
{
    int reg;
    int len = 0;
    u8 data = 0;

    if (NULL == show_buffer)
        return -1;

    for (reg = 0; reg <= 0x3F; reg++)
    {
        if (stk832x_reg_read(stk, reg, 0, &data))
        {
            len = -1;
            goto exit;
        }

        if (0 >= (PAGE_SIZE - len))
        {
            STK_ACC_ERR("print string out of PAGE_SIZE");
            goto exit;
        }

        len += scnprintf(show_buffer + len, PAGE_SIZE - len,
                         "[0x%2X]=0x%2X, ", reg, data);
    }

    len += scnprintf(show_buffer + len, PAGE_SIZE - len, "\n");
exit:

    return len;
}

/**
 * @brief: Get offset
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] offset: offset value read from register
 *                  STK832X_REG_OFSTX,  STK832X_REG_OFSTY, STK832X_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_get_offset(struct stk832x_data *stk, u8 offset[3])
{
    int error = 0;
    bool enable = false;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_read(stk, STK832X_REG_OFSTX, 0, &offset[0]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_read(stk, STK832X_REG_OFSTY, 0, &offset[1]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_read(stk, STK832X_REG_OFSTZ, 0, &offset[2]))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return error;
}

/**
 * @brief: Set offset
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value write to register
 *                  STK832X_REG_OFSTX,  STK832X_REG_OFSTY, STK832X_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_set_offset(struct stk832x_data *stk, u8 offset[3])
{
    int error = 0;
    bool enable = false;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_write(stk, STK832X_REG_OFSTX, offset[0]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_write(stk, STK832X_REG_OFSTY, offset[1]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_write(stk, STK832X_REG_OFSTZ, offset[2]))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return error;
}

#ifdef STK_STEP_COUNTER
/**
 * @brief: read step counter value from register.
 *          Store result to stk832x_data.steps.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_step_data(struct stk832x_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk832x_reg_read(stk, STK832X_REG_STEPOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_STEPOUT2, 0, &dataH))
        return;

    stk->steps = dataH << 8 | dataL;
}

/**
 * @brief: Turn ON/OFF step count.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] turn: true to turn ON step count; false to turn OFF.
 */
static void stk_turn_step_counter(struct stk832x_data *stk, bool turn)
{
    if (turn)
    {
        if (stk832x_reg_write(stk, STK832X_REG_STEPCNT2,
                              STK832X_STEPCNT2_RST_CNT | STK832X_STEPCNT2_STEP_CNT_EN))
            return;
    }
    else
    {
        if (stk832x_reg_write(stk, STK832X_REG_STEPCNT2, 0))
            return;
    }

    stk->steps = 0;
}
#endif /* STK_STEP_COUNTER */

#ifdef STK_CALI
/**
 * @brief: Write calibration config file to STK_CALI_FILE.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] w_buf: cali data what want to write to STK_CALI_FILE.
 * @param[in] buf_size: size of w_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_write_to_file(struct stk832x_data *stk,
                             char *w_buf, int8_t buf_size)
{
    struct file *cali_file;
    char r_buf[buf_size];
    mm_segment_t fs;
    ssize_t ret;
    int i;
    cali_file = filp_open(STK_CALI_FILE, O_CREAT | O_RDWR, 0666);

    if (IS_ERR(cali_file))
    {
        STK_ACC_ERR("err=%ld, failed to open %s", PTR_ERR(cali_file), STK_CALI_FILE);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->write(cali_file, w_buf, buf_size,
                                     &cali_file->f_pos);

        if (0 > ret)
        {
            STK_ACC_ERR("write error, ret=%d", (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        cali_file->f_pos = 0x0;
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);

        if (0 > ret)
        {
            STK_ACC_ERR("read error, ret=%d", (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        set_fs(fs);

        for (i = 0; i < buf_size; i++)
        {
            if (r_buf[i] != w_buf[i])
            {
                STK_ACC_ERR("read back error! r_buf[%d]=0x%X, w_buf[%d]=0x%X", i, r_buf[i], i, w_buf[i]);
                filp_close(cali_file, NULL);
                return -1;
            }
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/**
 * @brief: Get calibration config file from STK_CALI_FILE.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] r_buf: cali data what want to read from STK_CALI_FILE.
 * @param[in] buf_size: size of r_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_get_from_file(struct stk832x_data *stk,
                             char *r_buf, int8_t buf_size)
{
    struct file *cali_file;
    mm_segment_t fs;
    ssize_t ret;
    cali_file = filp_open(STK_CALI_FILE, O_RDONLY, 0);

    if (IS_ERR(cali_file))
    {
        STK_ACC_ERR("err=%ld, failed to open %s", PTR_ERR(cali_file), STK_CALI_FILE);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);
        set_fs(fs);

        if (0 > ret)
        {
            STK_ACC_ERR("read error, ret=%d\n", (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/**
 * @brief:
 */
static void stk_get_cali(struct stk832x_data *stk, u8 cali[3])
{
    char stk_file[STK_CALI_FILE_SIZE];

    if (stk_get_from_file(stk, stk_file, STK_CALI_FILE_SIZE) == 0)
    {
        if (STK_CALI_VER0 == stk_file[0]
            && STK_CALI_VER1 == stk_file[1]
            && STK_CALI_END == stk_file[STK_CALI_FILE_SIZE - 1])
        {
            atomic_set(&stk->cali_status, (int)stk_file[8]);
            cali[0] = stk_file[3];
            cali[1] = stk_file[5];
            cali[2] = stk_file[7];
            STK_ACC_LOG("offset:%d,%d,%d, mode=0x%X", stk_file[3], stk_file[5], stk_file[7], stk_file[8]);
#if 0
            STK_ACC_LOG("variance=%u,%u,%u",
                        (stk_file[9] << 24 | stk_file[10] << 16 | stk_file[11] << 8 | stk_file[12]),
                        (stk_file[13] << 24 | stk_file[14] << 16 | stk_file[15] << 8 | stk_file[16]),
                        (stk_file[17] << 24 | stk_file[18] << 16 | stk_file[19] << 8 | stk_file[20]));
#endif
        }
        else
        {
            int i;
            STK_ACC_ERR("wrong cali version number");

            for (i = 0; i < STK_CALI_FILE_SIZE; i++)
                STK_ACC_LOG("cali_file[%d]=0x%X\n", i, stk_file[i]);
        }
    }
}

/**
 * @brief: Get sample_no of samples then calculate average
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_ms: delay in msec
 * @param[in] sample_no: amount of sample
 * @param[out] acc_ave: XYZ average
 */
static void stk_calculate_average(struct stk832x_data *stk,
                                  unsigned int delay_ms, int sample_no, int acc_ave[3])
{
    int i;

    for (i = 0; i < sample_no; i++)
    {
        msleep(delay_ms);
        stk_read_accel_data(stk);
        acc_ave[0] += stk->xyz[0];
        acc_ave[1] += stk->xyz[1];
        acc_ave[2] += stk->xyz[2];
    }

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (i = 0; i < 3; i++)
    {
        if ( 0 <= acc_ave[i])
            acc_ave[i] = (acc_ave[i] + sample_no / 2) / sample_no;
        else
            acc_ave[i] = (acc_ave[i] - sample_no / 2) / sample_no;
    }

    /*
     * For Z-axis
     * Pre-condition: Sensor be put on a flat plane, with +z face up.
     */
    if (0 < acc_ave[2])
        acc_ave[2] -= stk->sensitivity;
    else
        acc_ave[2] += stk->sensitivity;
}

/**
 * @brief: Align STK832X_REG_OFSTx sensitivity with STK832X_REG_RANGESEL
 *  Description:
 *  Example:
 *      RANGESEL=0x3 -> +-2G / 12bits for STK832x full resolution
 *              number bit per G = 2^12 / (2x2) = 1024 (LSB/g)
 *              (2x2) / 2^12 = 0.97 mG/bit
 *      OFSTx: There are 8 bits to describe OFSTx for +-1G
 *              number bit per G = 2^8 / (1x2) = 128 (LSB/g)
 *              (1x2) / 2^8 = 7.8125mG/bit
 *      Align: acc_OFST = acc * 128 / 1024
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in/out] acc: accel data
 *
 */
static void stk_align_offset_sensitivity(struct stk832x_data *stk, int acc[3])
{
    int axis;

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (axis = 0; axis < 3; axis++)
    {
        if (acc[axis] > 0)
        {
            acc[axis] = (acc[axis] * STK832X_OFST_LSB + stk->sensitivity / 2)
                        / stk->sensitivity;
        }
        else
        {
            acc[axis] = (acc[axis] * STK832X_OFST_LSB - stk->sensitivity / 2)
                        / stk->sensitivity;
        }
    }
}

/**
 * @brief: Verify offset.
 *          Read register of STK832X_REG_OFSTx, then check data are the same as
 *          what we wrote or not.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value to compare with the value in register
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 */
static int stk_verify_offset(struct stk832x_data *stk, u8 offset[3])
{
    int axis;
    u8 offset_from_reg[3] = {0, 0, 0};

    if (stk_get_offset(stk, offset_from_reg))
        return STK_K_FAIL_I2C;

    for (axis = 0; axis < 3; axis++)
    {
        if (offset_from_reg[axis] != offset[axis])
        {
            STK_ACC_ERR("set OFST failed! offset[%d]=%d, read from reg[%d]=%d",
                        axis, offset[axis], axis, offset_from_reg[axis]);
            atomic_set(&stk->cali_status, STK_K_FAIL_WRITE_OFST);
            return STK_K_FAIL_WRITE_OFST;
        }
    }

    return 0;
}

/**
 * @brief: Write calibration data to config file
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value
 * @param[in] status: status
 *                  STK_K_SUCCESS_FILE
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_write_cali_to_file(struct stk832x_data *stk,
                                  u8 offset[3], u8 status)
{
    char file_buf[STK_CALI_FILE_SIZE];
    memset(file_buf, 0, sizeof(file_buf));
    file_buf[0] = STK_CALI_VER0;
    file_buf[1] = STK_CALI_VER1;
    file_buf[3] = offset[0];
    file_buf[5] = offset[1];
    file_buf[7] = offset[2];
    file_buf[8] = status;
    file_buf[STK_CALI_FILE_SIZE - 2] = '\0';
    file_buf[STK_CALI_FILE_SIZE - 1] = STK_CALI_END;

    if (stk_write_to_file(stk, file_buf, STK_CALI_FILE_SIZE))
        return -1;

    return 0;
}

/**
 * @brief: Calibration action
 *          1. Calculate calibration data
 *          2. Write data to STK832X_REG_OFSTx
 *          3. Check calibration well-done with chip register
 *          4. Write calibration data to file
 *          Pre-condition: Sensor be put on a flat plane, with +z face up.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 *          STK_K_FAIL_W_FILE: fail during writing cali to file
 */
static int stk_cali_do(struct stk832x_data *stk, int delay_us)
{
    int error = 0;
    int acc_ave[3] = {0, 0, 0};
    unsigned int delay_ms = delay_us / 1000;
    u8 offset[3] = {0, 0, 0};
    int acc_verify[3] = {0, 0, 0};
    const unsigned char verify_diff = stk->sensitivity / 10;
    int axis;
#ifdef STK_CHECK_CODE
    msleep(delay_ms * STK_CHECKCODE_IGNORE);
#endif /* STK_CHECK_CODE */
    stk_calculate_average(stk, delay_ms, STK_CALI_SAMPLE_NO, acc_ave);
    stk_align_offset_sensitivity(stk, acc_ave);

    for (axis = 0; axis < 3; axis++)
        offset[axis] = -acc_ave[axis];

    STK_ACC_LOG("New offset for XYZ: %d, %d, %d\n", acc_ave[0], acc_ave[1], acc_ave[2]);
    error = stk_set_offset(stk, offset);

    if (error)
        return STK_K_FAIL_I2C;

    /* Read register, then check OFSTx are the same as we wrote or not */
    error = stk_verify_offset(stk, offset);

    if (error)
        return error;

    /* verify cali */
    stk_calculate_average(stk, delay_ms, 3, acc_verify);

    if (verify_diff < abs(acc_verify[0]) || verify_diff < abs(acc_verify[1])
        || verify_diff < abs(acc_verify[2]))
    {
        STK_ACC_ERR("Check data x:%d, y:%d, z:%d. Check failed!",
                    acc_verify[0], acc_verify[1], acc_verify[2]);
        return STK_K_FAIL_VERIFY_CALI;
    }

    /* write cali to file */
    error = stk_write_cali_to_file(stk, offset, STK_K_SUCCESS_FILE);

    if (error)
    {
        STK_ACC_ERR("failed to stk_write_cali_to_file, error=%d", error);
        return STK_K_FAIL_W_FILE;
    }

    atomic_set(&stk->cali_status, STK_K_SUCCESS_FILE);
    return 0;
}

/**
 * @brief: Set calibration
 *          1. Change delay to 8000msec
 *          2. Reset offset value by trigger OFST_RST
 *          3. Calibration action
 *          4. Change delay value back
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_set_cali(struct stk832x_data *stk)
{
    int error = 0;
    bool enable;
    int org_delay_us, real_delay_us;
    atomic_set(&stk->cali_status, STK_K_RUNNING);
    org_delay_us = stk_get_delay(stk);
    /* Use several samples (with ODR:125) for calibration data base */
    error = stk_set_delay(stk, 8000);

    if (error)
    {
        STK_ACC_ERR("failed to stk_set_delay, error=%d", error);
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        return;
    }

    real_delay_us = stk_get_delay(stk);

    /* SW reset before getting calibration data base */
    if (atomic_read(&stk->enabled))
    {
        enable = true;
        stk_set_enable(stk, 0);
    }
    else
        enable = false;

    stk_set_enable(stk, 1);
    error = stk832x_reg_write(stk, STK832X_REG_OFSTCOMP1,
                              STK832X_OFSTCOMP1_OFST_RST);

    if (error)
    {
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        goto exit_for_OFST_RST;
    }

    /* Action for calibration */
    error = stk_cali_do(stk, real_delay_us);

    if (error)
    {
        STK_ACC_ERR("failed to stk_cali_do, error=%d", error);
        atomic_set(&stk->cali_status, error);
        goto exit_for_OFST_RST;
    }

    STK_ACC_LOG("successful calibration");
exit_for_OFST_RST:

    if (!enable)
        stk_set_enable(stk, 0);

    stk_set_delay(stk, org_delay_us);
}

/**
 * @brief: Reset calibration
 *          1. Reset offset value by trigger OFST_RST
 *          2. Calibration action
 */
static void stk_reset_cali(struct stk832x_data *stk)
{
    stk832x_reg_write(stk, STK832X_REG_OFSTCOMP1,
                       STK832X_OFSTCOMP1_OFST_RST);
    atomic_set(&stk->cali_status, STK_K_NO_CALI);
    memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));
}

/**
 * @brief: Get calibration status
 *          Send calibration status to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    u8 cali[3] = {0, 0, 0};

    if (STK_K_RUNNING != atomic_read(&stk->cali_status))
        stk_get_cali(stk, cali);

    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", atomic_read(&stk->cali_status));
}

/**
 * @brief: Trigger to calculate calibration data
 *          Get 1 from userspace, then start to calculate calibration data.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;

    if (sysfs_streq(buf, "1"))
        stk_set_cali(stk);
    else
    {
        STK_ACC_ERR("invalid value %d", *buf);
        return -EINVAL;
    }

    return count;
}
#endif /* STK_CALI */

/**
 * @brief: Read FIFO data
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] fifo: FIFO data
 * @param[in] len: FIFO size what you want to read
 */
static void stk_fifo_reading(struct stk832x_data *stk, u8 fifo[], int len)
{
    /* Reject all register R/W to protect FIFO data reading */
    STK_ACC_LOG("Start to read FIFO data");

    if (stk832x_reg_read(stk, STK832X_REG_FIFOOUT, len, fifo))
    {
        STK_ACC_ERR("Break to read FIFO data");
    }

    STK_ACC_LOG("Done for reading FIFO data");
}

/**
 * @brief: Change FIFO status
 *          If wm = 0, change FIFO to bypass mode.
 *          STK832X_CFG1_XYZ_FRAME_MAX >= wm, change FIFO to FIFO mode +
 *                                          STK832X_CFG2_FIFO_DATA_SEL_XYZ.
 *          Do nothing if STK832X_CFG1_XYZ_FRAME_MAX < wm.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] wm: water mark
 *
 * @return: Success or fail
 *          0: Success
 *          Others: Fail
 */
static int stk_change_fifo_status(struct stk832x_data *stk, u8 wm)
{
    int error = 0;
    u8 regIntmap2 = 0, regInten2 = 0;

    if (STK832X_CFG1_XYZ_FRAME_MAX < wm)
    {
        STK_ACC_ERR("water mark out of range(%d)", wm);
        return -1;
    }

    error = stk832x_reg_read(stk, STK832X_REG_INTMAP2, 0, &regIntmap2);
    if (error)
        return error;

    error = stk832x_reg_read(stk, STK832X_REG_INTEN2, 0, &regInten2);
    if (error)
        return error;

    if (wm)
    {
        /* FIFO settings: FIFO mode + XYZ per frame */
        error = stk832x_reg_write(stk, STK832X_REG_CFG2,
                                  (STK832X_CFG2_FIFO_MODE_FIFO << STK832X_CFG2_FIFO_MODE_SHIFT)
                                  | STK832X_CFG2_FIFO_DATA_SEL_XYZ);

        if (error)
            return error;

        error = stk832x_reg_write(stk, STK832X_REG_INTMAP2, regIntmap2 | STK832X_INTMAP2_FWM2INT1);
        if (error)
            return error;

        error = stk832x_reg_write(stk, STK832X_REG_INTEN2, regInten2 | STK832X_INTEN2_FWM_EN);
        if (error)
            return error;
    }
    else
    {
        /* FIFO settings: bypass mode */
        error = stk832x_reg_write(stk, STK832X_REG_CFG2,
                                  STK832X_CFG2_FIFO_MODE_BYPASS << STK832X_CFG2_FIFO_MODE_SHIFT);

        if (error)
            return error;

        error = stk832x_reg_write(stk, STK832X_REG_INTMAP2, regIntmap2 & (~STK832X_INTMAP2_FWM2INT1));
        if (error)
            return error;

        error = stk832x_reg_write(stk, STK832X_REG_INTEN2, regInten2 & (~STK832X_INTEN2_FWM_EN));
        if (error)
            return error;
    }

    error = stk832x_reg_write(stk, STK832X_REG_CFG1, wm);

    if (error)
        return error;

    return 0;
}

/**
 * @brief: Get power status
 *          Send 0 or 1 to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    char en;
    en = atomic_read(&stk->enabled);
    return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

/**
 * @brief: Set power status
 *          Get 0 or 1 from userspace, then set stk832x power status.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    unsigned int data;
    int error;
    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        STK_ACC_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    if ((1 == data) || (0 == data))
        stk_set_enable(stk, data);
    else
        STK_ACC_ERR("invalid argument, en=%d", data);

    return count;
}

/**
 * @brief: Get accel data
 *          Send accel data to userspce.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_value_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    bool enable = true;

    if (!atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 1);
        enable = false;
    }

    stk_read_accel_data(stk);

    if (!enable)
        stk_set_enable(stk, 0);

    return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
                     stk->xyz[0], stk->xyz[1], stk->xyz[2]);
}

/**
 * @brief: Get delay value in usec
 *          Send delay in usec to userspce.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)stk_get_delay(stk) * 1000);
}

/**
 * @brief: Set delay value in usec
 *          Get delay value in usec from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    long long data;
    int error;
    int int32_data = 0;
    error = kstrtoll(buf, 10, &data);

    if (error)
    {
        STK_ACC_ERR("kstrtoul failed, error=%d", error);
        return error;
    }

    int32_data = (int)(data >> 10);
    STK_ACC_LOG("delay us = %d, %lld", int32_data, data);
//    stk_set_delay(stk, (int)(data / 1000));
    stk_set_delay(stk, int32_data);
    return count;
}

/**
 * @brief: Get offset value
 *          Send X/Y/Z offset value to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    u8 offset[3] = {0, 0, 0};
    stk_get_offset(stk, offset);
    return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X\n",
                     offset[0], offset[1], offset[2]);
}

/**
 * @brief: Set offset value
 *          Get X/Y/Z offset value from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    char *token[10];
    u8 r_offset[3];
    int error, data, i;

    for (i = 0; i < 3; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &data);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    r_offset[0] = (u8)data;
    error = kstrtoint(token[1], 16, &data);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    r_offset[1] = (u8)data;
    error = kstrtoint(token[2], 16, &data);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    r_offset[2] = (u8)data;
    STK_ACC_LOG("offset=0x%X, 0x%X, 0x%X", r_offset[0], r_offset[1], r_offset[2]);
    stk_set_offset(stk, r_offset);
    return count;
}

/**
 * @brief: Register writting
 *          Get address and content from userspace, then write to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_send_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    char *token[10];
    int addr, cmd, error, i;
    bool enable = false;

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &addr);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    error = kstrtoint(token[1], 16, &cmd);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    STK_ACC_LOG("write reg[0x%X]=0x%X", addr, cmd);

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_write(stk, (u8)addr, (u8)cmd))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/**
 * @brief: Read stk832x_data.recv(from stk_recv_store), then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    return scnprintf(buf, PAGE_SIZE, "0x%X\n", atomic_read(&stk->recv));
}

/**
 * @brief: Get the read address from userspace, then store the result to
 *          stk832x_data.recv.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    int addr, error;
    u8 data = 0;
    bool enable = false;
    error = kstrtoint(buf, 16, &addr);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_read(stk, (u8)addr, 0, &data))
    {
        error = -1;
        goto exit;
    }

    atomic_set(&stk->recv, data);
    STK_ACC_LOG("read reg[0x%X]=0x%X", addr, data);
exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/**
 * @brief: Read all register value, then send result to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_allreg_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    int result;
    result = stk_show_all_reg(stk, buf);

    if (0 >  result)
        return result;

    return (ssize_t)result;
}

/**
 * @brief: Check PID, then send chip number to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_chipinfo_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;

    if (STK8323_ID == stk->pid)
        return scnprintf(buf, PAGE_SIZE, "stk8321/8323\n");
    else if (STK8325_ID == stk->pid)
        return scnprintf(buf, PAGE_SIZE, "stk8325\n");

    return scnprintf(buf, PAGE_SIZE, "unknown\n");
}

/**
 * @brief: Read FIFO data, then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    u8 fifo_wm = 0;
    u8 frame_unit = 0;
    int fifo_len, len = 0;

    if (stk832x_reg_read(stk, STK832X_REG_CFG1, 0, &fifo_wm))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");

    if (0 == fifo_wm)
        return scnprintf(buf, PAGE_SIZE , "FIFO disabled\n");

    if (stk832x_reg_read(stk, STK832X_REG_CFG2, 0, &frame_unit))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");

    frame_unit &= STK832X_CFG2_FIFO_DATA_SEL_MASK;

    if (0 == frame_unit)
        fifo_len = fifo_wm * 6; /* xyz * 2 bytes/axis */
    else
        fifo_len = fifo_wm * 2; /* single axis * 2 bytes/axis */

    {
        u8 *fifo = NULL;
        int i;
        /* vzalloc: allocate memory and set to zero. */
        fifo = vzalloc(sizeof(u8) * fifo_len);

        if (!fifo)
        {
            STK_ACC_ERR("memory allocation error");
            return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");
        }

        stk_fifo_reading(stk, fifo, fifo_len);

        for (i = 0; i < fifo_wm; i++)
        {
            if (0 == frame_unit)
            {
                s16 x, y, z;
                x = fifo[i * 6 + 1] << 8 | fifo[i * 6];
                x >>= 4;
                y = fifo[i * 6 + 3] << 8 | fifo[i * 6 + 2];
                y >>= 4;
                z = fifo[i * 6 + 5] << 8 | fifo[i * 6 + 4];
                z >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth x:%d, y:%d, z:%d\n", i, x, y, z);
            }
            else
            {
                s16 xyz;
                xyz = fifo[i * 2 + 1] << 8 | fifo[i * 2];
                xyz >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth fifo:%d\n", i, xyz);
            }

            if ( 0 >= (PAGE_SIZE - len))
            {
                STK_ACC_ERR("print string out of PAGE_SIZE");
                break;
            }
        }

        vfree(fifo);
    }
    return len;
}

/**
 * @brief: Read water mark from userspace, then send to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    int wm, error;
    error = kstrtoint(buf, 10, &wm);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    if (stk_change_fifo_status(stk, (u8)wm))
    {
        return -1;
    }

    return count;
}

#ifdef STK_STEP_COUNTER
/**
 * @brief: Read step counter data, then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_step_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    /*
        bool enable = true;

        if (!atomic_read(&stk->enabled))
        {
            stk_set_enable(stk, 1);
            enable = false;
        }
    */
    stk_read_step_data(stk);
    /*
        if (!enable)
            stk_set_enable(stk, 0);
    */
    return scnprintf(buf, PAGE_SIZE, "%hd\n",
                     stk->steps);
}

/**
 * @brief: Read step counter setting from userspace, then send to register.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_step_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    int step, error;
    error = kstrtoint(buf, 10, &step);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    if (step)
        stk_turn_step_counter(stk, true);
    else
        stk_turn_step_counter(stk, false);

    return count;
}
#endif /* STK_STEP_COUNTER */

#ifdef STK_FIR
/**
 * @brief: Get FIR parameter, then send to userspace.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_show(struct device_driver *ddri, char *buf)
{
    struct stk832x_data *stk = stk_data;
    int len = atomic_read(&stk->fir_len);

    if (len)
    {
        STK_ACC_LOG("FIR count=%2d, idx=%2d", stk->fir.count, stk->fir.idx);
        STK_ACC_LOG("sum = [\t%d \t%d \t%d]", stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2]);
        STK_ACC_LOG("avg = [\t%d \t%d \t%d]", stk->fir.sum[0] / len, stk->fir.sum[1] / len, stk->fir.sum[2] / len);
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

/**
 * @brief: Get FIR length from userspace, then write to stk832x_data.fir_len.
 *
 * @param[in] ddri: struct device_driver *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_store(struct device_driver *ddri, const char *buf, size_t count)
{
    struct stk832x_data *stk = stk_data;
    int firlen, error;
    error = kstrtoint(buf, 10, &firlen);

    if (error)
    {
        STK_ACC_ERR("kstrtoint failed, error=%d", error);
        return error;
    }

    if (STK_FIR_LEN_MAX < firlen)
        STK_ACC_ERR("maximum FIR length is %d", STK_FIR_LEN_MAX);
    else
    {
        memset(&stk->fir, 0, sizeof(struct data_fir));
        atomic_set(&stk->fir_len, firlen);
    }

    return count;
}
#endif /* STK_FIR */

static DRIVER_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DRIVER_ATTR(value, 0444, stk_value_show, NULL);
static DRIVER_ATTR(delay, 0664, stk_delay_show, stk_delay_store);
static DRIVER_ATTR(offset, 0664, stk_offset_show, stk_offset_store);
static DRIVER_ATTR(send, 0220, NULL, stk_send_store);
static DRIVER_ATTR(recv, 0664, stk_recv_show, stk_recv_store);
static DRIVER_ATTR(allreg, 0444, stk_allreg_show, NULL);
static DRIVER_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DRIVER_ATTR(fifo, 0664, stk_fifo_show, stk_fifo_store);
#ifdef STK_STEP_COUNTER
static DRIVER_ATTR(stepcount, 0644, stk_step_show, stk_step_store);
#endif /* STK_STEP_COUNTER */
#ifdef STK_CALI
static DRIVER_ATTR(cali, 0664, stk_cali_show, stk_cali_store);
#endif /* STK_CALI */
#ifdef STK_FIR
    static DRIVER_ATTR(firlen, 0664, stk_firlen_show, stk_firlen_store);
#endif /* STK_FIR */

static struct driver_attribute *stk_attr_list[] =
{
    &driver_attr_enable,
    &driver_attr_value,
    &driver_attr_delay,
    &driver_attr_offset,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_allreg,
    &driver_attr_chipinfo,
    &driver_attr_fifo,
#ifdef STK_STEP_COUNTER
    &driver_attr_stepcount,
#endif /* STK_STEP_COUNTER */
#ifdef STK_CALI
    &driver_attr_cali,
#endif /* STK_CALI */
#ifdef STK_FIR
    &driver_attr_firlen,
#endif /* STK_FIR */
};

/**
 * @brief:
 */
static int stk_create_attr(struct device_driver *driver)
{
    int i, error = 0;
    int num = (int)(sizeof(stk_attr_list) / sizeof(stk_attr_list[0]));

    if (NULL == driver)
        return -EINVAL;

    for (i = 0; i < num; i++)
    {
        error = driver_create_file(driver, stk_attr_list[i]);

        if (error)
        {
            STK_ACC_ERR("driver_create_file (%s) = %d",
                        stk_attr_list[i]->attr.name, error);
            break;
        }
    }

    return error;
}

/**
 * @brief:
 */
static void stk_create_attr_exit(struct device_driver *driver)
{
    int i;
    int num = (int)(sizeof(stk_attr_list) / sizeof(stk_attr_list[0]));

    if (NULL == driver)
        return;

    for (i = 0; i < num; i++)
        driver_remove_file(driver, stk_attr_list[i]);
}

#ifdef STK_HAND_DETECT
/**
 * TODO
 */
static bool stk_check_hand_detect(struct stk832x_data *stk)
{
    int acc[3] = {0};
    stk_read_accel_data(stk);
    acc[stk->cvt.map[0]] = stk->cvt.sign[0] * stk->xyz[0];
    acc[stk->cvt.map[1]] = stk->cvt.sign[1] * stk->xyz[1];
    acc[stk->cvt.map[2]] = stk->cvt.sign[2] * stk->xyz[2];
    return stk_hand_detect(acc);
}

/**
 * TODO
 * stk->hd_delaywork
 */
static void stk_hand_detect_cnt(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, hd_delaywork.work);
    int hd_cnt = atomic_read(&stk->hd_cnt);

    if (stk_check_hand_detect(stk))
    {
        hd_cnt++;
    }
    else if (1 < hd_cnt)
    {
        hd_cnt--;
    }
    else
    {
        atomic_set(&stk->hd_cnt, 0);
        return;
    }

    atomic_set(&stk->hd_cnt, hd_cnt);

    if (STK_HAND_DETECT_CNT_THRESHOLD <= hd_cnt)
    {
        atomic_set(&stk->hd_cnt, 0);
        STK_ACC_LOG("hand detect!");
        /* hand detect! Do something.*/
    }
    else
    {
        STK_ACC_LOG("hand detect cnt:%d", hd_cnt);
        schedule_delayed_work(&stk->hd_delaywork, msecs_to_jiffies(16));
    }
}

/**
 * TODO
 */
static void stk_hand_detect_start(struct stk832x_data *stk)
{
    STK_ACC_FUN();
    atomic_set(&stk->hd_cnt, 1);
    schedule_delayed_work(&stk->hd_delaywork, msecs_to_jiffies(16));
}
#endif /* STK_HAND_DETECT */

#if defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE
/**
 * @brief:
 */
static void stk_report_accel_data(struct stk832x_data *stk)
{
#ifdef STK_CHECK_CODE

    if ((STK_CCSTATUS_XYSIM == stk->cc_status)
        || ((STK_CHECKCODE_IGNORE + 6) > stk->cc_count))
        return;

#endif /* STK_CHECK_CODE */
    //STK_ACC_LOG("x:%d, y:%d, z:%d", stk->xyz[0], stk->xyz[1], stk->xyz[2]);
}
#endif /* defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE */

#ifdef STK_AMD
/**
 * @brief:
 */
static void stk_reset_latched_int(struct stk832x_data *stk)
{
    u8 data = 0;

    if (stk832x_reg_read(stk, STK832X_REG_INTCFG2, 0, &data))
        return;

    if (stk832x_reg_write(stk, STK832X_REG_INTCFG2, (data | STK832X_INTCFG2_INT_RST)))
        return;
}
#endif /* STK_AMD */

#ifdef STK_INTERRUPT_MODE
/**
 * @brief: Queue work list.
 *              ???????
 *          5. Enable IRQ.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_data_irq_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, alldata_work);
    bool enable = true;
#ifdef STK_AMD
    u8 data = 0;
#endif /* STK_AMD */

    if (!atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 1);
        enable = false;
    }

    stk_read_accel_data(stk);
    stk_report_accel_data(stk);

#ifdef STK_AMD
    if (!stk832x_reg_read(stk, STK832X_REG_INTSTS1, 0, &data))
    {
        if (STK832X_INTSTS1_ANY_MOT_STS & data)
        {
            STK_ACC_LOG("Get trigger for any motion");
#ifdef STK_HAND_DETECT

            if (0 == atomic_read(&stk->hd_cnt))
            {
                if (stk_check_hand_detect(stk))
                    stk_hand_detect_start(stk);
            }

#endif /* STK_HAND_DETECT */
        }
    }

    stk_reset_latched_int(stk);
#endif /* STK_AMD */

    if (!enable)
        stk_set_enable(stk, 0);

    enable_irq(stk->irq1);
}

/**
 * @brief: IRQ handler. This function will be trigger after receiving IRQ.
 *          1. Disable IRQ without waiting.
 *          2. Send work to quque.
 *
 * @param[in] irq: irq number
 * @param[in] data: void *
 *
 * @return: IRQ_HANDLED
 */
static irqreturn_t stk_all_data_handler(int irq, void *data)
{
    struct stk832x_data *stk = data;
    disable_irq_nosync(irq);
    queue_work(stk->alldata_workqueue, &stk->alldata_work);
    return IRQ_HANDLED;
}

/**
 * @brief:
 */
static int stk_interrupt_mode_setup(struct stk832x_data *stk)
{
    int error = 0;
    struct device_node *stk_node;
    u32 ints[2] = {0, 0};
    stk->alldata_workqueue = create_singlethread_workqueue("stk_int1_wq");

    if (stk->alldata_workqueue)
        INIT_WORK(&stk->alldata_work, stk_data_irq_work);
    else
    {
        STK_ACC_ERR("create_singlethread_workqueue error");
        error = -EPERM;
        goto exit_err;
    }

    stk_node = of_find_compatible_node(NULL, NULL, "mediatek, ACCEL-eint");

    if (stk_node)
    {
        of_property_read_u32_array(stk_node, "interrupts", ints, ARRAY_SIZE(ints));
        stk->interrupt_int1_pin = ints[0];
        gpio_direction_input(stk->interrupt_int1_pin);
        error = gpio_to_irq(stk->interrupt_int1_pin);

        if (0 > error)
        {
            STK_ACC_ERR("gpio_to_irq failed");
            error = -EINVAL;
            goto exit_gpio_request_1;
        }

        stk->irq1 = error;
        STK_ACC_LOG("irq #=%d, interrupt pin=%d", stk->irq1, stk->interrupt_int1_pin);
        error = request_irq(stk->irq1, stk_all_data_handler,
                                        IRQF_TRIGGER_RISING, STK832X_IRQ_INT1_NAME, stk);

        if (0 > error)
        {
            STK_ACC_ERR("request_irq(%d) failed for %d", stk->irq1, error);
            goto exit_gpio_to_irq_1;
        }

#ifdef STK_HAND_DETECT
        INIT_DELAYED_WORK(&stk->hd_delaywork, stk_hand_detect_cnt);
#endif /* STK_HAND_DETECT */
    }
    else
    {
        STK_ACC_ERR("Null device node of ACCEL-eint");
        return -EINVAL;
    }

    return 0;
exit_gpio_to_irq_1:
    gpio_free(stk->interrupt_int1_pin);
exit_gpio_request_1:
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
exit_err:
    return error;
}

/**
 * @brief:
 */
static void stk_interrupt_mode_exit(struct stk832x_data *stk)
{
#ifdef STK_HAND_DETECT
    cancel_delayed_work_sync(&stk->hd_delaywork);
#endif /* STK_HAND_DETECT */
    free_irq(stk->irq1, stk);
    gpio_free(stk->interrupt_int1_pin);
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
}

#elif defined STK_POLLING_MODE
/**
 * @brief: Queue delayed_work list.
 *          ??????.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_accel_delay_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, accel_delaywork.work);
#ifdef STK_AMD
    u8 data = 0;
#endif /* STK_AMD */

    stk_read_accel_data(stk);
    stk_report_accel_data(stk);

#ifdef STK_AMD
    if (!stk832x_reg_read(stk, STK832X_REG_INTSTS1, 0, &data))
    {
        if (STK832X_INTSTS1_ANY_MOT_STS & data)
        {
            STK_ACC_LOG("Get trigger for any motion");
#ifdef STK_HAND_DETECT

            if (0 == atomic_read(&stk->hd_cnt))
            {
                if (stk_check_hand_detect(stk))
                    stk_hand_detect_start(stk);
            }

#endif /* STK_HAND_DETECT */
        }
    }

    stk_reset_latched_int(stk);
#endif /* STK_AMD */
}

/**
 * @brief: This function will send delayed_work to queue.
 *          This function will be called regularly with period:
 *          stk832x_data.poll_delay.
 *
 * @param[in] timer: struct hrtimer *
 *
 * @return: HRTIMER_RESTART.
 */
static enum hrtimer_restart stk_accel_timer_func(struct hrtimer *timer)
{
    struct stk832x_data *stk =
        container_of(timer, struct stk832x_data, accel_timer);
    schedule_delayed_work(&stk->accel_delaywork, 0);
    hrtimer_forward_now(&stk->accel_timer, stk->poll_delay);
    return HRTIMER_RESTART;
}

static int stk_polling_mode_setup(struct stk832x_data *stk)
{
    /* polling accel data */
    INIT_DELAYED_WORK(&stk->accel_delaywork, stk_accel_delay_work);
    hrtimer_init(&stk->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    stk->poll_delay = ns_to_ktime(
                          STK832X_SAMPLE_TIME[STK832X_BWSEL_INIT_ODR - STK832X_SPTIME_BASE]
                          * NSEC_PER_USEC);
    stk->accel_timer.function = stk_accel_timer_func;
#ifdef STK_HAND_DETECT
    INIT_DELAYED_WORK(&stk->hd_delaywork, stk_hand_detect_cnt);
#endif /* STK_HAND_DETECT */
    return 0;
}

/**
 * @brief:
 */
static void stk_polling_mode_exit(struct stk832x_data *stk)
{
#ifdef STK_HAND_DETECT
    cancel_delayed_work_sync(&stk->hd_delaywork);
#endif /* STK_HAND_DETECT */
    hrtimer_try_to_cancel(&stk->accel_timer);
    cancel_delayed_work_sync(&stk->accel_delaywork);
}
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

/**
 * @brief: stk832x register initialize
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] range: stk_rangesel for STK832X_REG_RANGESEL
 *              STK_2G
 *              STK_4G
 *              STK_8G
 * @param[in] odr_no: odr index.
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_reg_init(struct stk832x_data *stk, stk_rangesel range, u8 odr_no)
{
    int error = 0;
    /* SW reset */
    error = stk_sw_reset(stk);

    if (error)
        return error;

    /* INT1, push-pull, active high. */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG1,
                              STK832X_INTCFG1_INT1_ACTIVE_H | STK832X_INTCFG1_INT1_OD_PUSHPULL);

    if (error)
        return error;

#ifdef STK_INTERRUPT_MODE
    /* map new accel data interrupt to int1 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP2, STK832X_INTMAP2_DATA2INT1);

    if (error)
        return error;

    /* enable new data interrupt for both new accel data */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN2, STK832X_INTEN2_DATA_EN);

    if (error)
        return error;
#else /* no STK_INTERRUPT_MODE */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP2, 0);

    if (error)
        return error;

    error = stk832x_reg_write(stk, STK832X_REG_INTEN2, 0);

    if (error)
        return error;
#endif /* STK_INTERRUPT_MODE */
#ifdef STK_AMD
    /* map any motion interrupt to int1 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP1,
                              STK832X_INTMAP1_ANYMOT2INT1);

    if (error)
        return error;

    /* enable new data interrupt for any motion */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN1, STK832X_INTEN1_SLP_EN_XYZ);

    if (error)
        return error;

    /*
     * latch int
     * In interrupt mode + significant/any motion mode, both of them share the same INT.
     * Set latched to make sure we can get SIG data(SIG_MOT_STS/ANY_MOT_SYS) before signal fall down.
     * Read SIG/ANY flow:
     * Get INT --> check INTSTS1.SIG_MOT_STS/ANY_MOT_SYS status -> INTCFG2.INT_RST(relese all latched INT)
     * Read FIFO flow:
     * Get INT --> check INTSTS2.FWM_STS status -> INTCFG2.INT_RST(relese all latched INT)
     * In latch mode, echo interrupt(SIT_MOT_STS/FWM_STS) will cause all INT(INT1/INT2)
     * rising up.
     */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG2,
                              STK832X_INTCFG2_INT_RST | STK832X_INTCFG2_LATCHED);

    if (error)
        return error;

    /* SIGMOT2 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT2,
                              STK832X_SIGMOT2_ANY_MOT_EN);

    if (error)
        return error;
#else /* no STK_AMD */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP1, 0);

    if (error)
        return error;

    error = stk832x_reg_write(stk, STK832X_REG_INTEN1, 0);

    if (error)
        return error;

    /* non-latch int */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG2, STK832X_INTCFG2_NOLATCHED);

    if (error)
        return error;

    /* SIGMOT2 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT2, 0);

    if (error)
        return error;
#endif /* STK_AMD */

#ifdef STK_HAND_DETECT
    /* SLOPE THRESHOLD */
    error = stk832x_reg_write(stk, STK832X_REG_SLOPETHD, 0x32);

    if (error)
        return error;

#else /* no STK_HAND_DETECT */
    /* SLOPE THRESHOLD */
    error = stk832x_reg_write(stk, STK832X_REG_SLOPETHD, STK832X_SLOPETHD_DEF);

    if (error)
        return error;

#endif /* STK_HAND_DETECT */
    /* SLOPE DELAY */
    error = stk832x_reg_write(stk, STK832X_REG_SLOPEDLY, 0x00);

    if (error)
        return error;

    /* SIGMOT1 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT1,
                              STK832X_SIGMOT1_SKIP_TIME_3SEC);

    if (error)
        return error;

    /* SIGMOT3 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT3,
                              STK832X_SIGMOT3_PROOF_TIME_1SEC);

    if (error)
        return error;

    /* According to STK_DEF_DYNAMIC_RANGE */
    error = stk_range_selection(stk, range);

    if (error)
        return error;

    /* ODR */
    error = stk832x_reg_write(stk, STK832X_REG_BWSEL, odr_no);

    if (error)
        return error;

    stk_change_fifo_status(stk, 0);
    /* i2c watchdog enable */
    error = stk832x_reg_write(stk, STK832X_REG_INTFCFG,
                              STK832X_INTFCFG_I2C_WDT_EN);

    if (error)
        return error;

    return 0;
}

/**
 * @brief
 */
#if 0
static int stk_fops_open(struct inode *inode, struct file *file)
{
    file->private_data = stk_data->client;

    if (NULL == file->private_data)
    {
        STK_ACC_ERR("Null point for i2c_client");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/**
 * @brief
 */
static int stk_fops_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

/**
 * @brief:
 */
static long stk_fops_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct stk832x_data *stk = i2c_get_clientdata(client);
    char strbuf[STK_BUFSIZE];
    void __user *data;
    int error = 0;
    bool enable = true;
    struct GSENSOR_VECTOR3D sensor_vector;
    struct SENSOR_DATA sensor_data;
    u8 xyz[3] = {0, 0, 0};

    if (_IOC_DIR(cmd) & _IOC_READ)
        error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (error)
    {
        STK_ACC_ERR("access error: %08X, (%2d, %2d)", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
        case GSENSOR_IOCTL_INIT:
            stk_reg_init(stk, STK_2G, STK832X_BWSEL_INIT_ODR);
            break;

        case GSENSOR_IOCTL_READ_CHIPINFO:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            sprintf(strbuf, "STK832x Chip");

            if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_READ_SENSORDATA:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            if (!atomic_read(&stk->enabled))
            {
                stk_set_enable(stk, 1);
                enable = false;
            }

            stk_read_accel_data(stk);

            if (!enable)
                stk_set_enable(stk, 0);

            sprintf(strbuf, "%04x %04x %04x", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

            if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_READ_OFFSET:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            stk_get_offset(stk, xyz);
            sensor_vector.x = (unsigned short)(xyz[0]);
            sensor_vector.y = (unsigned short)(xyz[1]);
            sensor_vector.z = (unsigned short)(xyz[2]);

            if (copy_to_user(data, &sensor_vector, sizeof(sensor_vector)))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_READ_GAIN:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            stk_get_sensitivity(stk);
            sensor_vector.x = (unsigned short)(stk->sensitivity);
            sensor_vector.y = (unsigned short)(stk->sensitivity);
            sensor_vector.z = (unsigned short)(stk->sensitivity);

            if (copy_to_user(data, &sensor_vector, sizeof(sensor_vector)))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_READ_RAW_DATA:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            if (!atomic_read(&stk->enabled))
            {
                stk_set_enable(stk, 1);
                enable = false;
            }

            stk_read_accel_rawdata(stk);

            if (!enable)
                stk_set_enable(stk, 0);

            sprintf(strbuf, "%04x %04x %04x", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

            if (copy_to_user(data, strbuf, strlen(strbuf) + 1))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_SET_CALI:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

            if (copy_from_user(&sensor_data, data, sizeof(sensor_data)))
            {
                error = -EFAULT;
                break;
            }

#ifdef STK_CALI
            xyz[0] = (u8)(sensor_data.x * stk->sensitivity / GRAVITY_EARTH_1000);
            xyz[1] = (u8)(sensor_data.y * stk->sensitivity / GRAVITY_EARTH_1000);
            xyz[2] = (u8)(sensor_data.z * stk->sensitivity / GRAVITY_EARTH_1000);
            /* write cali to file */
            error = stk_write_cali_to_file(stk, xyz, STK_K_SUCCESS_FILE);

            if (error)
            {
                STK_ACC_ERR("failed to stk_write_cali_to_file, error=%d", error);
                error = -EFAULT;
                break;
            }
#endif /* STK_CALI */
            stk->cali_sw[0] = (int)(sensor_data.x * stk->sensitivity / GRAVITY_EARTH_1000);
            stk->cali_sw[1] = (int)(sensor_data.y * stk->sensitivity / GRAVITY_EARTH_1000);
            stk->cali_sw[2] = (int)(sensor_data.z * stk->sensitivity / GRAVITY_EARTH_1000);

            break;

        case GSENSOR_IOCTL_GET_CALI:
            data = (void __user *)arg;

            if (NULL == data)
            {
                error = -EINVAL;
                break;
            }

#ifdef STK_CALI
            stk_get_cali(stk, xyz);
            sensor_data.x = (unsigned short)(xyz[0] * GRAVITY_EARTH_1000 / stk->sensitivity);
            sensor_data.y = (unsigned short)(xyz[1] * GRAVITY_EARTH_1000 / stk->sensitivity);
            sensor_data.z = (unsigned short)(xyz[2] * GRAVITY_EARTH_1000 / stk->sensitivity);
#else /* no STK_CALI */
            sensor_data.x = (unsigned short)stk->cali_sw[0];
            sensor_data.y = (unsigned short)stk->cali_sw[1];
            sensor_data.z = (unsigned short)stk->cali_sw[2];
#endif /* STK_CALI */

            if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
            {
                error = -EFAULT;
                break;
            }

            break;

        case GSENSOR_IOCTL_CLR_CALI:
#ifdef STK_CALI
            /* write cali to file */
            error = stk_write_cali_to_file(stk, xyz, STK_K_SUCCESS_FILE);

            if (error)
            {
                STK_ACC_ERR("failed to stk_write_cali_to_file, error=%d", error);
                error = -EFAULT;
                break;
            }
#endif /* STK_CALI */
            memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));

            break;

        default:
            STK_ACC_ERR("unknown IOCTL: 0x%08X", cmd);
            error = -ENOIOCTLCMD;
            break;
    }

    return error;
}

#ifdef CONFIG_COMPAT
/**
 * @brief:
 */
static long stk_fops_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct stk832x_data *stk = i2c_get_clientdata(client);
    long error = 0;
    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_INIT:
            stk_reg_init(stk, STK_2G, STK832X_BWSEL_INIT_ODR);
            break;

        case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_READ_CHIPINFO failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_READ_SENSORDATA failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_READ_OFFSET:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_OFFSET, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_READ_OFFSET failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_READ_GAIN:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_GAIN, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_READ_GAIN failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_RAW_DATA, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_READ_RAW_DATA failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_SET_CALI failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_GET_CALI failed.");
                return error;
            }

            break;

        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (NULL == arg32)
            {
                error = -EINVAL;
                break;
            }

            error = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);

            if (error)
            {
                STK_ACC_ERR("GSENSOR_IOCTL_CLR_CALI failed.");
                return error;
            }

            break;

        default:
            STK_ACC_ERR("unknown IOCTL: 0x%08X", cmd);
            error = -ENOIOCTLCMD;
            break;
    }

    return error;
}
#endif

static struct file_operations stk_miscdevice_fops =
{
    .owner = THIS_MODULE,
    .open = stk_fops_open,
    .release = stk_fops_release,
    .unlocked_ioctl = stk_fops_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = stk_fops_compat_ioctl,
#endif
};

static struct miscdevice stk_miscdevice =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &stk_miscdevice_fops,
};
#endif
/**
 * @brief: Open data rerport to HAL.
 *      refer: drivers/misc/mediatek/accelerometer/inc/accel.h
 */
static int gsensor_open_report_data(int open)
{
    /* TODO. should queuq work to report event if  is_report_input_direct=true */
    return 0;
}

/**
 * @brief: Only enable not report event to HAL.
 *      refer: drivers/misc/mediatek/accelerometer/inc/accel.h
 */
static int gsensor_enable_nodata(int en)
{
    struct stk832x_data *stk = stk_data;

    if (en)
    {
        stk_set_enable(stk, 1);
        atomic_set(&stk->enabled_for_acc, 1);
    }
    else
    {
        stk_set_enable(stk, 0);
        atomic_set(&stk->enabled_for_acc, 0);
    }

    STK_ACC_LOG("enabled_for_acc is %d", en);
    return 0;
}

/**
 * @brief:
 */
static int gsensor_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    STK_ACC_FUN();
    return 0;
}

/**
 * @brief:
 */
static int gsensor_flush(void)
{
    STK_ACC_FUN();
//    return -1; /* error */
    return acc_flush_report();
}

/**
 * @brief:
 */
static int gsensor_set_delay(u64 delay_ns)
{
    struct stk832x_data *stk = stk_data;
    STK_ACC_LOG("delay= %d ms", (int)((int)delay_ns / 1000));
    stk_set_delay(stk, (int)((int)delay_ns / 1000));
    return 0;
}

/**
 * @brief:
 */
static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
    struct stk832x_data *stk = stk_data;
    int error, x_data, y_data, z_data;
    int acc[3] = {0};
    char buff[256];
    stk_read_accel_data(stk);
    sprintf(buff, "%04x %04x %04x", stk->xyz[0], stk->xyz[1], stk->xyz[2]);

    stk->xyz[0] += stk->cali_sw[0];
    stk->xyz[1] += stk->cali_sw[1];
    stk->xyz[2] += stk->cali_sw[2];
    /* remap coordinate */
    acc[stk->cvt.map[0]] = stk->cvt.sign[0] * stk->xyz[0];
    acc[stk->cvt.map[1]] = stk->cvt.sign[1] * stk->xyz[1];
    acc[stk->cvt.map[2]] = stk->cvt.sign[2] * stk->xyz[2];
    x_data = acc[0] * GRAVITY_EARTH_1000 / 1000;
    y_data = acc[1] * GRAVITY_EARTH_1000 / 1000;
    z_data = acc[2] * GRAVITY_EARTH_1000 / 1000;
    STK_ACC_LOG("report x = %d, y = %d, z = %d\n", acc[0], acc[1], acc[2]);
    sprintf(buff, "%04x %04x %04x", x_data, y_data, z_data);
    error = sscanf(buff, "%x %x %x", x, y, z);

    if (3 != error)
    {
        STK_ACC_ERR("Invalid argument");
        return -EINVAL;
    }

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static int stk_readCalibration(int *dat)
{
    struct stk832x_data *stk = stk_data;

    STK_ACC_LOG("ori x:%d, y:%d, z:%d", stk->cali_sw[STK_AXIS_X], stk->cali_sw[STK_AXIS_Y], stk->cali_sw[STK_AXIS_Z]);
    dat[stk->cvt.map[0]] = stk->cvt.sign[0] * stk->cali_sw[STK_AXIS_X];
    dat[stk->cvt.map[1]] = stk->cvt.sign[1] * stk->cali_sw[STK_AXIS_Y];
    dat[stk->cvt.map[2]] = stk->cvt.sign[2] * stk->cali_sw[STK_AXIS_Z];

    return 0;
}

static int stk_writeCalibration(int *dat)
{
    struct stk832x_data *stk = stk_data;
    int err = 0;
    int cali[STK_AXES_NUM];

    err = stk_readCalibration(cali);

    STK_ACC_LOG("raw cali_sw[%d][%d][%d] dat[%d][%d][%d]",
            cali[0], cali[1], cali[2], dat[0], dat[1], dat[2]);

    cali[STK_AXIS_X] = dat[STK_AXIS_X];
    cali[STK_AXIS_Y] = dat[STK_AXIS_Y];
    cali[STK_AXIS_Z] = dat[STK_AXIS_Z];

    stk->cali_sw[STK_AXIS_X] = stk->cvt.sign[STK_AXIS_X]*(cali[stk->cvt.map[STK_AXIS_X]]);
    stk->cali_sw[STK_AXIS_Y] = stk->cvt.sign[STK_AXIS_Y]*(cali[stk->cvt.map[STK_AXIS_Y]]);
    stk->cali_sw[STK_AXIS_Z] = stk->cvt.sign[STK_AXIS_Z]*(cali[stk->cvt.map[STK_AXIS_Z]]);
    STK_ACC_LOG("new cali_sw[%d][%d][%d]",
            stk->cali_sw[0], stk->cali_sw[1], stk->cali_sw[2]);

    mdelay(1);

    return err;
}

static int stk_factory_enable_sensor(bool enable, int64_t sample_ms)
{
    int en = (true == enable ? 1 : 0);
    if (gsensor_enable_nodata(en))
    {
        STK_ACC_ERR("enable sensor failed");
        return -1;
    }

    return 0;
}

static int stk_factory_get_data(int32_t data[3], int *status)
{
    return gsensor_get_data(&data[0], &data[1], &data[2], status);
}

static int stk_factory_get_raw_data(int32_t data[3])
{
    struct stk832x_data *stk = stk_data;
    stk_read_accel_rawdata(stk);
    data[0] = (int32_t)stk->xyz[0];
    data[1] = (int32_t)stk->xyz[1];
    data[2] = (int32_t)stk->xyz[2];
    return 0;
}

static int stk_factory_enable_cali(void)
{
#ifdef STK_CALI
    struct stk832x_data *stk = stk_data;
    stk_set_cali(stk);
#endif /* STK_CALI */
    return 0;
}

static int stk_factory_clear_cali(void)
{
    struct stk832x_data *stk = stk_data;
#ifdef STK_CALI
    stk_reset_cali(stk);
#endif /* STK_CALI */
    memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));

    return 0;
}

static int stk_factory_set_cali(int32_t data[3])
{
    int error = 0;
    struct stk832x_data *stk = stk_data;
    int cali[3] = {0, 0, 0};

#ifdef STK_CALI
    u8 xyz[3] = {0, 0, 0};
    atomic_set(&stk->cali_status, STK_K_RUNNING);
#endif /* STK_CALI */
    cali[0] = data[0] * stk->sensitivity / GRAVITY_EARTH_1000;
    cali[1] = data[1] * stk->sensitivity / GRAVITY_EARTH_1000;
    cali[2] = data[2] * stk->sensitivity / GRAVITY_EARTH_1000;

    STK_ACC_LOG("new x:%d, y:%d, z:%d", cali[0], cali[1], cali[2]);
#ifdef STK_CALI
    xyz[0] = (u8)cali[0];
    xyz[1] = (u8)cali[1];
    xyz[2] = (u8)cali[2];
    /* write cali to file */
    error = stk_write_cali_to_file(stk, xyz, STK_K_SUCCESS_FILE);

    if (error)
    {
        STK_ACC_ERR("failed to stk_write_cali_to_file, error=%d", error);
        return -1;
    }
#endif /* STK_CALI */
    
    error = stk_writeCalibration(cali);
    if (error)
    {
        STK_ACC_ERR("stk_writeCalibration failed!");
        return -1;
    }
#ifdef STK_CALI
    atomic_set(&stk->cali_status, STK_K_SUCCESS_FILE);
#endif /* STK_CALI */
    return 0;
}

static int stk_factory_get_cali(int32_t data[3])
{
    struct stk832x_data *stk = stk_data;

	int err = 0;
	int cali[3] = { 0 };

	err = stk_readCalibration(cali);

#if 1
   data[0] = (int32_t)(cali[0] * GRAVITY_EARTH_1000 / stk->sensitivity);
   data[1] = (int32_t)(cali[1] * GRAVITY_EARTH_1000 / stk->sensitivity);
   data[2] = (int32_t)(cali[2] * GRAVITY_EARTH_1000 / stk->sensitivity);
#else
    u8 cali[3] = {0, 0, 0};

    if (STK_K_RUNNING != atomic_read(&stk->cali_status))
    {
        stk_get_cali(stk, cali);
        data[0] = (int32_t)(cali[0] * GRAVITY_EARTH_1000 / stk->sensitivity);
        data[1] = (int32_t)(cali[1] * GRAVITY_EARTH_1000 / stk->sensitivity);
        data[2] = (int32_t)(cali[2] * GRAVITY_EARTH_1000 / stk->sensitivity);
    }
    else
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }
#endif
    STK_ACC_LOG("x:%d, y:%d, z:%d", data[0], data[1], data[2]);

    return 0;
}

static int stk_factory_do_self_test(void)
{
    struct stk832x_data *stk = stk_data;

    stk_selftest(stk);

    if (STK_SELFTEST_RESULT_NO_ERROR == atomic_read(&stk->selftest))
        return 0;
    else
        return -1;
}

static struct accel_factory_fops stk_factory_fops =
{
    .enable_sensor = stk_factory_enable_sensor,
    .get_data = stk_factory_get_data,
    .get_raw_data = stk_factory_get_raw_data,
    .enable_calibration = stk_factory_enable_cali,
    .clear_cali = stk_factory_clear_cali,
    .set_cali = stk_factory_set_cali,
    .get_cali = stk_factory_get_cali,
    .do_self_test = stk_factory_do_self_test,
};

static struct accel_factory_public stk_factory_device =
{
    .gain = 1,
    .sensitivity = 1,
    .fops = &stk_factory_fops,
};

/**
 * @brief: Proble function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] id: struct i2c_device_id *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int error = 0;
    struct stk832x_data *stk;
    struct acc_control_path stk_acc_control_path = {0};
    struct acc_data_path stk_acc_data_path = {0};
    STK_ACC_LOG("driver version:%s", STK_ACC_DRIVER_VERSION);
    /* kzalloc: allocate memory and set to zero. */
    stk = kzalloc(sizeof(struct stk832x_data), GFP_KERNEL);

    if (!stk)
    {
        STK_ACC_ERR("memory allocation error");
        return -ENOMEM;
    }

    error = get_accel_dts_func(client->dev.of_node, &stk->hw);

    if (0 != error)
    {
        STK_ACC_ERR("Dts info fail");
        goto err_free_mem;
    }

    client->addr = STK832X_SLAVE_ADDR;
    /* direction */
    error = hwmsen_get_convert(stk->hw.direction, &stk->cvt);

    if (error)
    {
        STK_ACC_ERR("invalid direction: %d", stk->hw.direction);
        goto err_free_mem;
    }

    stk_data = stk;
    stk->client = client;
    i2c_set_clientdata(client, stk);
    mutex_init(&stk->reg_lock);
    stk_data_initialize(stk);

    if (stk_get_pid(stk))
        goto err_free_mem;

    STK_ACC_LOG("PID 0x%x", stk->pid);

    if (STK8323_ID != stk->pid
            && STK8325_ID != stk->pid)
    {
        error = -EINVAL;
        STK_ACC_LOG("chip id not match");
        goto err_free_mem;
    }

#ifdef STK_INTERRUPT_MODE
    error = stk_interrupt_mode_setup(stk);

    if (0 > error)
        goto err_free_mem;

#elif defined STK_POLLING_MODE
    error = stk_polling_mode_setup(stk);

    if (0 > error)
        goto err_free_mem;

#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

    if (stk_reg_init(stk, STK_2G, STK832X_BWSEL_INIT_ODR))
    {
        STK_ACC_ERR("stk832x initialization failed");
        goto exit_stk_init_error;
    }
#if 0
    error = misc_register(&stk_miscdevice);

    if (error)
    {
        STK_ACC_ERR("stk832x misc_register failed");
        goto exit_misc_error;
    }
#endif
    error = stk_create_attr(&stk_acc_init_info.platform_diver_addr->driver);

    if (error)
    {
        STK_ACC_ERR("stk_create_attr failed");
        goto exit_misc_error;
    }

    /* MTK Android usage +++ */
    stk_acc_control_path.is_use_common_factory = false;
    /* factory */
    error = accel_factory_device_register(&stk_factory_device);

    if (error)
    {
        STK_ACC_ERR("accel_factory_device_register failed");
        goto exit_register_control_path;
    }

    stk_acc_control_path.open_report_data = gsensor_open_report_data;
    stk_acc_control_path.enable_nodata = gsensor_enable_nodata;
    stk_acc_control_path.is_support_batch = false;
    stk_acc_control_path.batch = gsensor_batch;
    stk_acc_control_path.flush = gsensor_flush;
    stk_acc_control_path.set_delay = gsensor_set_delay;
    stk_acc_control_path.is_report_input_direct = false;
    error = acc_register_control_path(&stk_acc_control_path);

    if (error)
    {
        STK_ACC_ERR("acc_register_control_path fail");
        goto exit_register_control_path;
    }

    stk_acc_data_path.get_data = gsensor_get_data;
    stk_acc_data_path.vender_div = 1000;
    error = acc_register_data_path(&stk_acc_data_path);

    if (error)
    {
        STK_ACC_ERR("acc_register_data_path fail");
        goto exit_register_control_path;
    }

    /* MTK Android usage --- */
    stk832x_init_flag = 0;
    hq_regiser_hw_info(HWID_GSENSOR,"stk8321");
    STK_ACC_LOG("Success");
    return 0;
exit_register_control_path:
    stk_create_attr_exit(&stk_acc_init_info.platform_diver_addr->driver);
exit_misc_error:
   //misc_deregister(&stk_miscdevice);
exit_stk_init_error:
#ifdef STK_INTERRUPT_MODE
    stk_interrupt_mode_exit(stk);
#elif defined STK_POLLING_MODE
    stk_polling_mode_exit(stk);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
err_free_mem:
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    stk832x_init_flag = -1;
    return error;
}

/**
 * @brief
 */
static int stk832x_i2c_remove(struct i2c_client *client)
{
    struct stk832x_data *stk = i2c_get_clientdata(client);
    accel_factory_device_deregister(&stk_factory_device);
    stk_create_attr_exit(&stk_acc_init_info.platform_diver_addr->driver);
    //misc_deregister(&stk_miscdevice);
#ifdef STK_INTERRUPT_MODE
    stk_interrupt_mode_exit(stk);
#elif defined STK_POLLING_MODE
    stk_polling_mode_exit(stk);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
/**
 * @brief
 */
static int stk832x_i2c_suspend(struct device *dev)
{
    STK_ACC_LOG();
    return 0;
}

/**
 * @brief
 */
static int stk832x_i2c_resume(struct device *dev)
{
    STK_ACC_LOG();
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id stk832x_i2c_id[] =
{
    {STK_ACC_DEV_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk832x_i2c_id);

static const struct of_device_id stk_acc_match[] =
{
    {.compatible = STK_ACC_DTS_NAME},
    {},
};
MODULE_DEVICE_TABLE(i2c, stk_acc_match);

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops stk_i2c_pm_ops =
{
    SET_SYSTEM_SLEEP_PM_OPS(stk832x_i2c_suspend, stk832x_i2c_resume)
};
#endif /* CONFIG_PM_SLEEP */

static struct i2c_driver stk832x_i2c_driver =
{
    .probe          = stk832x_i2c_probe,
    .remove         = stk832x_i2c_remove,
    .id_table       = stk832x_i2c_id,
    .driver         = {
        .name               = STK_ACC_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
        .pm                 = &stk_i2c_pm_ops,
#endif /* CONFIG_PM_SLEEP */
        .of_match_table     = stk_acc_match,
    },
};

/**
 * @brief:
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_acc_init(void)
{
    STK_ACC_FUN();

    if (i2c_add_driver(&stk832x_i2c_driver))
    {
        STK_ACC_ERR("Add i2c driver fail");
        return -1;
    }

    if ( -1 == stk832x_init_flag)
    {
        STK_ACC_ERR("stk832x init error");
        return -1;
    }

    return 0;
}

/**
 * @brief:
 *
 * @return: Success
 *          0: Success
 */
static int stk_acc_uninit(void)
{
    i2c_del_driver(&stk832x_i2c_driver);
    return 0;
}

#ifdef STK_STEP_COUNTER
/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 * Open data report to HAL
 */
static int stk_step_c_open_report_data(int open)
{
    STK_ACC_LOG(" open:%d", open);
    /* TODO. */
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 * Only enable and not report event to HAL.
 */
static int stk_step_c_enable_nodata(int en)
{
    struct stk832x_data *stk = stk_data;

    if (en)
    {
        stk_set_enable(stk, 1);
        stk_turn_step_counter(stk, 1);
    }
    else
    {
        stk_set_enable(stk, 0);
        stk_turn_step_counter(stk, 0);
    }

    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_c_enable_step_detect(int en)
{
    stk_step_c_enable_nodata(en);
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_c_enable_significant(int en)
{
    STK_ACC_LOG(" en:%d", en);
    /* TODO. */
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_c_set_delay(u64 delay_ns)
{
    STK_ACC_LOG(" delay_ns:%lld", delay_ns);
    /* TODO. */
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_d_set_delay(u64 delay_ns)
{
    STK_ACC_LOG(" delay_ns:%lld", delay_ns);
    /* TODO. */
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_get_data(uint32_t *value, int *status)
{
    struct stk832x_data *stk = stk_data;
    stk_read_step_data(stk);
    *value = stk->steps;
    STK_ACC_LOG(" value:%d, status=%d", *value, *status);
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_get_data_step_d(uint32_t *value, int *status)
{
    STK_ACC_LOG(" value:%d, status=%d", *value, *status);
    /* TODO. */
    return 0;
}

/**
 * drivers/misc/mediatek/step_counter/step_counter.h
 */
static int stk_step_get_data_significant(uint32_t *value, int *status)
{
    STK_ACC_LOG(" value:%d, status=%d", *value, *status);
    /* TODO. */
    return 0;
}

/**
 */
static int stk_step_c_local_init(void)
{
    int error = 0;
    struct step_c_control_path stk_step_ctl_path = {0};
    struct step_c_data_path stk_step_data_path = {0};
    STK_ACC_FUN();

    if (-1 == stk832x_init_flag)
    {
        STK_ACC_ERR("stk832x init fail");
        return error;
    }

    mutex_lock(&stk_step_init_mutex);
    error = stk_create_attr(&stk_step_c_init_info.platform_diver_addr->driver);

    if (error)
    {
        STK_ACC_ERR("stk_create_attr failed");
        goto err_create_attr;
    }

    stk_step_ctl_path.open_report_data = stk_step_c_open_report_data;
    stk_step_ctl_path.enable_nodata = stk_step_c_enable_nodata;
    stk_step_ctl_path.enable_step_detect = stk_step_c_enable_step_detect;
    stk_step_ctl_path.enable_significant = stk_step_c_enable_significant;
    stk_step_ctl_path.step_c_set_delay = stk_step_c_set_delay;
    stk_step_ctl_path.step_d_set_delay = stk_step_d_set_delay;
    stk_step_ctl_path.is_report_input_direct = false;
    error = step_c_register_control_path(&stk_step_ctl_path);

    if (error)
    {
        STK_ACC_ERR("failed in step_c_register_control_path");
        goto err_exit;
    }

    stk_step_data_path.get_data = stk_step_get_data;
    stk_step_data_path.get_data_step_d = stk_step_get_data_step_d;
    stk_step_data_path.get_data_significant = stk_step_get_data_significant;
    stk_step_data_path.vender_div = 1;
    error = step_c_register_data_path(&stk_step_data_path);

    if (error)
    {
        STK_ACC_ERR("failed in step_c_register_data_path");
        goto err_exit;
    }

    mutex_unlock(&stk_step_init_mutex);
    return 0;
err_exit:
    stk_create_attr_exit(&stk_step_c_init_info.platform_diver_addr->driver);
err_create_attr:
    mutex_unlock(&stk_step_init_mutex);
    return error;
}

/**
 */
static int stk_step_c_local_uninit(void)
{
    stk_create_attr_exit(&stk_step_c_init_info.platform_diver_addr->driver);
    return 0;
}

static struct step_c_init_info stk_step_c_init_info =
{
    .name = STK_STEP_C_DEV_NAME,
    .init = stk_step_c_local_init,
    .uninit = stk_step_c_local_uninit,
};
#endif /* STK_STEP_COUNTER */

/**
 * @brief:
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int __init stk832x_init(void)
{
    STK_ACC_FUN();
    acc_driver_add(&stk_acc_init_info);
#ifdef STK_STEP_COUNTER
    step_c_driver_add(&stk_step_c_init_info);
#endif /* STK_STEP_COUNTER */
    return 0;
}

module_init(stk832x_init);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk832x 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(STK_ACC_DRIVER_VERSION);
#endif /* defined CONFIG_OF && CONFIG_OF CONFIG_MTK_SENSOR_SUPPORT */
