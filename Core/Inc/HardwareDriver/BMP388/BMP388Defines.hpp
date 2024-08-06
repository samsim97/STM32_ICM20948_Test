#pragma once

#include <stdint.h>

#ifndef TRUE
#define TRUE         UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE        UINT8_C(0)
#endif

#define BMP3_ADDR_I2C_SEC                       UINT8_C(0x77)

#define BMP3_OK                                 INT8_C(0)

#define BMP3_LEN_P_T_DATA                       UINT8_C(6)

#define BMP3_ENABLE                             UINT8_C(0x01)

#define BMP3_NO_OVERSAMPLING                    UINT8_C(0x00)
#define BMP3_OVERSAMPLING_2X                    UINT8_C(0x01)
#define BMP3_OVERSAMPLING_4X                    UINT8_C(0x02)
#define BMP3_OVERSAMPLING_8X                    UINT8_C(0x03)
#define BMP3_OVERSAMPLING_16X                   UINT8_C(0x04)
#define BMP3_OVERSAMPLING_32X                   UINT8_C(0x05)

#define BMP3_ODR_200_HZ                         UINT8_C(0x00)
#define BMP3_ODR_100_HZ                         UINT8_C(0x01)
#define BMP3_ODR_50_HZ                          UINT8_C(0x02)
#define BMP3_ODR_25_HZ                          UINT8_C(0x03)
#define BMP3_ODR_12_5_HZ                        UINT8_C(0x04)
#define BMP3_ODR_6_25_HZ                        UINT8_C(0x05)
#define BMP3_ODR_3_1_HZ                         UINT8_C(0x06)
#define BMP3_ODR_1_5_HZ                         UINT8_C(0x07)
#define BMP3_ODR_0_78_HZ                        UINT8_C(0x08)
#define BMP3_ODR_0_39_HZ                        UINT8_C(0x09)
#define BMP3_ODR_0_2_HZ                         UINT8_C(0x0A)
#define BMP3_ODR_0_1_HZ                         UINT8_C(0x0B)
#define BMP3_ODR_0_05_HZ                        UINT8_C(0x0C)
#define BMP3_ODR_0_02_HZ                        UINT8_C(0x0D)
#define BMP3_ODR_0_01_HZ                        UINT8_C(0x0E)
#define BMP3_ODR_0_006_HZ                       UINT8_C(0x0F)
#define BMP3_ODR_0_003_HZ                       UINT8_C(0x10)
#define BMP3_ODR_0_001_HZ                       UINT8_C(0x11)

#define BMP3_SETTLE_TIME_PRESS                  UINT16_C(392)

#define BMP3_SETTLE_TIME_TEMP                   UINT16_C(313)

#define BMP3_ADC_CONV_TIME                      UINT16_C(2000)

#define BMP3_PRESS                              UINT8_C(1)
#define BMP3_TEMP                               UINT8_C(2)
#define BMP3_PRESS_TEMP                         UINT8_C(3)

#define BMP3_MIN_TEMP_DOUBLE                    -40.0f
#define BMP3_MAX_TEMP_DOUBLE                    85.0f
#define BMP3_MIN_PRES_DOUBLE                    30000.0f
#define BMP3_MAX_PRES_DOUBLE                    125000.0f

#define BMP3_W_MIN_TEMP                         INT8_C(3)
#define BMP3_W_MAX_TEMP                         INT8_C(4)
#define BMP3_W_MIN_PRES                         INT8_C(5)
#define BMP3_W_MAX_PRES                         INT8_C(6)

/**\name API error codes */
#define BMP3_E_NULL_PTR                         INT8_C(-1)
#define BMP3_E_COMM_FAIL                        INT8_C(-2)
#define BMP3_E_INVALID_ODR_OSR_SETTINGS         INT8_C(-3)
#define BMP3_E_CMD_EXEC_FAILED                  INT8_C(-4)
#define BMP3_E_CONFIGURATION_ERR                INT8_C(-5)
#define BMP3_E_INVALID_LEN                      INT8_C(-6)
#define BMP3_E_DEV_NOT_FOUND                    INT8_C(-7)
#define BMP3_E_FIFO_WATERMARK_NOT_REACHED       INT8_C(-8)

/**\name Register Address */
#define BMP3_REG_CHIP_ID                        UINT8_C(0x00)
#define BMP3_REG_ERR                            UINT8_C(0x02)
#define BMP3_REG_SENS_STATUS                    UINT8_C(0x03)
#define BMP3_REG_DATA                           UINT8_C(0x04)
#define BMP3_REG_EVENT                          UINT8_C(0x10)
#define BMP3_REG_INT_STATUS                     UINT8_C(0x11)
#define BMP3_REG_FIFO_LENGTH                    UINT8_C(0x12)
#define BMP3_REG_FIFO_DATA                      UINT8_C(0x14)
#define BMP3_REG_FIFO_WM                        UINT8_C(0x15)
#define BMP3_REG_FIFO_CONFIG_1                  UINT8_C(0x17)
#define BMP3_REG_FIFO_CONFIG_2                  UINT8_C(0x18)
#define BMP3_REG_INT_CTRL                       UINT8_C(0x19)
#define BMP3_REG_IF_CONF                        UINT8_C(0x1A)
#define BMP3_REG_PWR_CTRL                       UINT8_C(0x1B)
#define BMP3_REG_OSR                            UINT8_C(0X1C)
#define BMP3_REG_ODR                            UINT8_C(0x1D)
#define BMP3_REG_CONFIG                         UINT8_C(0x1F)
#define BMP3_REG_CALIB_DATA                     UINT8_C(0x31)
#define BMP3_REG_CMD                            UINT8_C(0x7E)

/**\name Error status macros */
#define BMP3_ERR_FATAL                          UINT8_C(0x01)
#define BMP3_ERR_CMD                            UINT8_C(0x02)
#define BMP3_ERR_CONF                           UINT8_C(0x04)

#define BMP3_STATUS_CMD_RDY_POS                 UINT8_C(0x04)
#define BMP3_STATUS_CMD_RDY_MSK                 UINT8_C(0x10)

#define BMP3_STATUS_DRDY_PRESS_MSK              UINT8_C(0x20)
#define BMP3_STATUS_DRDY_PRESS_POS              UINT8_C(0x05)

#define BMP3_STATUS_DRDY_TEMP_MSK               UINT8_C(0x40)
#define BMP3_STATUS_DRDY_TEMP_POS               UINT8_C(0x06)

#define BMP3_INT_STATUS_FWTM_MSK                UINT8_C(0x01)

#define BMP3_PRESS_EN_MSK                       UINT8_C(0x01)

#define BMP3_TEMP_EN_MSK                        UINT8_C(0x02)
#define BMP3_TEMP_EN_POS                        UINT8_C(0x01)

#define BMP3_INT_STATUS_FFULL_MSK               UINT8_C(0x02)
#define BMP3_INT_STATUS_FFULL_POS               UINT8_C(0x01)

#define BMP3_INT_STATUS_DRDY_MSK                UINT8_C(0x08)
#define BMP3_INT_STATUS_DRDY_POS                UINT8_C(0x03)

#define BMP3_ERR_CMD_MSK                        UINT8_C(0x02)
#define BMP3_ERR_CMD_POS                        UINT8_C(0x01)

#define BMP3_ERR_CONF_MSK                       UINT8_C(0x04)
#define BMP3_ERR_CONF_POS                       UINT8_C(0x02)

#define BMP3_ERR_FATAL_MSK                      UINT8_C(0x01)

#define BMP3_IIR_FILTER_MSK                     UINT8_C(0x0E)
#define BMP3_IIR_FILTER_POS                     UINT8_C(0x01)

#define BMP3_ODR_MSK                            UINT8_C(0x1F)

#define BMP3_PRESS_OS_MSK                       UINT8_C(0x07)

#define BMP3_TEMP_OS_MSK                        UINT8_C(0x38)
#define BMP3_TEMP_OS_POS                        UINT8_C(0x03)

#define BMP3_OP_MODE_MSK                        UINT8_C(0x30)
#define BMP3_OP_MODE_POS                        UINT8_C(0x04)

#define BMP3_I2C_WDT_SEL_MSK                    UINT8_C(0x04)
#define BMP3_I2C_WDT_SEL_POS                    UINT8_C(0x02)

#define BMP3_I2C_WDT_EN_MSK                     UINT8_C(0x02)
#define BMP3_I2C_WDT_EN_POS                     UINT8_C(0x01)

#define BMP3_INT_DRDY_EN_MSK                    UINT8_C(0x40)
#define BMP3_INT_DRDY_EN_POS                    UINT8_C(0x06)

#define BMP3_INT_OUTPUT_MODE_MSK                UINT8_C(0x01)

#define BMP3_INT_LEVEL_MSK                      UINT8_C(0x02)
#define BMP3_INT_LEVEL_POS                      UINT8_C(0x01)

#define BMP3_INT_LATCH_MSK                      UINT8_C(0x04)
#define BMP3_INT_LATCH_POS                      UINT8_C(0x02)

#define BMP3_CMD_RDY                            UINT8_C(0x10)

#define BMP3_SOFT_RESET                         UINT8_C(0xB6)

#define BMP3_LEN_CALIB_DATA                     UINT8_C(21)

#define BMP3_CHIP_ID                            UINT8_C(0x50)
#define BMP390_CHIP_ID                          UINT8_C(0x60)

/**\name Power mode macros */
#define BMP3_MODE_SLEEP                         UINT8_C(0x00)
#define BMP3_MODE_FORCED                        UINT8_C(0x01)
#define BMP3_MODE_NORMAL                        UINT8_C(0x03)

/**\name Macros to select the which sensor settings are to be set by the user.
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
#define BMP3_SEL_PRESS_EN                       UINT16_C(1 << 1)
#define BMP3_SEL_TEMP_EN                        UINT16_C(1 << 2)
#define BMP3_SEL_DRDY_EN                        UINT16_C(1 << 3)
#define BMP3_SEL_PRESS_OS                       UINT16_C(1 << 4)
#define BMP3_SEL_TEMP_OS                        UINT16_C(1 << 5)
#define BMP3_SEL_IIR_FILTER                     UINT16_C(1 << 6)
#define BMP3_SEL_ODR                            UINT16_C(1 << 7)
#define BMP3_SEL_OUTPUT_MODE                    UINT16_C(1 << 8)
#define BMP3_SEL_LEVEL                          UINT16_C(1 << 9)
#define BMP3_SEL_LATCH                          UINT16_C(1 << 10)
#define BMP3_SEL_I2C_WDT_EN                     UINT16_C(1 << 11)
#define BMP3_SEL_I2C_WDT                        UINT16_C(1 << 12)
#define BMP3_SEL_ALL                            UINT16_C(0x7FF)

/*! Power control settings */
#define BMP3_POWER_CNTL                         UINT16_C(0x0006)

/*! Odr and filter settings */
#define BMP3_ODR_FILTER                         UINT16_C(0x00F0)

/*! Interrupt control settings */
#define BMP3_INT_CTRL                           UINT16_C(0x0708)

/*! Advance settings */
#define BMP3_ADV_SETT                           UINT16_C(0x1800)


#define BMP3_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BMP3_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                 (bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMP3_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

enum bmp3_intf {
    /*! SPI interface */
    BMP3_SPI_INTF,
    /*! I2C interface */
    BMP3_I2C_INTF
};

/********************************************************/
/**\name Macro definitions */

/**
 * BMP3_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMP3_INTF_RET_TYPE
#define BMP3_INTF_RET_TYPE int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMP3_INTF_RET_SUCCESS
#define BMP3_INTF_RET_SUCCESS                   INT8_C(0)
#endif

/*!
 * @brief bmp3 sensor structure which comprises of un-compensated temperature
 * and pressure data.
 */
struct bmp3_uncomp_data
{
    /*! un-compensated pressure */
    uint64_t pressure;

    /*! un-compensated temperature */
    int64_t temperature;
};

/*!
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data
{
    /*! Quantized Trim Variables */

    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
};

/*!
 * @brief Register Trim Variables
 */
struct bmp3_reg_calib_data
{
    /*! Trim Variables */

    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
};

struct bmp3_calib_data
{
    /*! Quantized data */
    struct bmp3_quantized_calib_data quantized_calib_data;

    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};

struct bmp3_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! Interface Selection
     * For SPI, interface = BMP3_SPI_INTF
     * For I2C, interface = BMP3_I2C_INTF
     **/
    enum bmp3_intf intf;

    /*! To store interface pointer error */
    BMP3_INTF_RET_TYPE intf_rslt;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Read function pointer */
    //bmp3_read_fptr_t read;

    /*! Write function pointer */
    //bmp3_write_fptr_t write;

    /*! Delay function pointer */
    //bmp3_delay_us_fptr_t delay_us;

    /*! Trim data */
    struct bmp3_calib_data calib_data;
};

struct bmp3_data
{
    /*! Compensated temperature */
    double temperature;

    /*! Compensated pressure */
    double pressure;

};

/*!
 * @brief bmp3 odr and filter settings
 */
struct bmp3_odr_filter_settings
{
    /*! Pressure oversampling */
    uint8_t press_os;

    /*! Temperature oversampling */
    uint8_t temp_os;

    /*! IIR filter */
    uint8_t iir_filter;

    /*! Output data rate */
    uint8_t odr;
};

/*!
 * @brief bmp3 interrupt pin settings
 */
struct bmp3_int_ctrl_settings
{
    /*! Output mode */
    uint8_t output_mode;

    /*! Active high/low */
    uint8_t level;

    /*! Latched or Non-latched */
    uint8_t latch;

    /*! Data ready interrupt */
    uint8_t drdy_en;
};

/*!
 * @brief bmp3 advance settings
 */
struct bmp3_adv_settings
{
    /*! I2C watchdog enable */
    uint8_t i2c_wdt_en;

    /*! I2C watchdog select */
    uint8_t i2c_wdt_sel;
};

struct bmp3_settings
{
    /*! Power mode which user wants to set */
    uint8_t op_mode;

    /*! Enable/Disable pressure sensor */
    uint8_t press_en;

    /*! Enable/Disable temperature sensor */
    uint8_t temp_en;

    /*! ODR and filter configuration */
    struct bmp3_odr_filter_settings odr_filter;

    /*! Interrupt configuration */
    struct bmp3_int_ctrl_settings int_settings;

    /*! Advance settings */
    struct bmp3_adv_settings adv_settings;
};

/*!
 * @brief bmp3 interrupt status flags
 */
struct bmp3_int_status
{
    /*! Fifo watermark interrupt */
    uint8_t fifo_wm;

    /*! Fifo full interrupt */
    uint8_t fifo_full;

    /*! Data ready interrupt */
    uint8_t drdy;
};

/*!
 * @brief bmp3 sensor status flags
 */
struct bmp3_sens_status
{
    /*! Command ready status */
    uint8_t cmd_rdy;

    /*! Data ready for pressure */
    uint8_t drdy_press;

    /*! Data ready for temperature */
    uint8_t drdy_temp;
};

/*!
 * @brief bmp3 error status flags
 */
struct bmp3_err_status
{
    /*! Fatal error */
    uint8_t fatal;

    /*! Command error */
    uint8_t cmd;

    /*! Configuration error */
    uint8_t conf;
};

struct bmp3_status
{
    /*! Interrupt status */
    struct bmp3_int_status intr;

    /*! Sensor status */
    struct bmp3_sens_status sensor;

    /*! Error status */
    struct bmp3_err_status err;

    /*! Power on reset status */
    uint8_t pwr_on_rst;
};
