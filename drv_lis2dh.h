#ifndef DRV_LIS2DH_H__
#define DRV_LIS2DH_H__

#include <stdio.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "string.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#define LIS2DH_ADDR         0x19
#define WHO_AM_I            0x0F
#define I_AM_LIS2DH         0x33

#define LIS2DH_CTRL_REG1    0x20
#define LIS2DH_CTRL_REG3    0x22
#define LIS2DH_CTRL_REG4    0x23
#define LIS2DH_CTRL_REG5    0x24
#define LIS2DH_CTRL_REG6    0x25
#define LIS2DH_OUT_X_L      0x28
#define LIS2DH_OUT_X_H      0x29
#define LIS2DH_OUT_Y_L      0x2A
#define LIS2DH_OUT_Y_H      0x2B
#define LIS2DH_OUT_Z_L      0x2C
#define LIS2DH_OUT_Z_H      0x2D
#define LIS2DH_FIFO_CTRL_REG  0x2E

#define LIS2DH_RESOLUTION_8B        0x01      // Different resolution mode => Low Power
#define LIS2DH_RESOLUTION_10B       0x02      // Normal mode (10b)
#define LIS2DH_RESOLUTION_12B       0x03      // High Resolution mode (12b)
#define LIS2DH_RESOLUTION_MAXVALUE  0x03

#define LIS2DH_LPEN_MASK    0x08
#define LIS2DH_HR_MASK      0x08
#define LIS2DH_FS_MASK      0x30
#define LIS2DH_ODR_MASK     0xF0
#define LIS2DH_BOOT_MASK    0x80

// FIFO_CTRL_REG masks
#define LIS2DH_FIFO_EN_MASK     0x40
#define LIS2DH_FM_MASK          0xC0
#define LIS2DH_FM_SHIFT         6
#define LIS2DH_FM_BYPASS        0x00      // No FIFO at all, the acceleration are not stored in FIFO
#define LIS2DH_FM_FIFO          0x01      // FIFO is used until being full after that no more data are added until clearing it by switching to bypass
#define LIS2DH_FM_STREAM        0x02      // FIFO is used and when full the older data are replaced by the new one.
#define LIS2DH_FM_STREAMFIFO    0x03      // In this mode the Interrupt Generator will automatically swicth the mode from STREAM to FiFo
#define LIS2DH_FM_MAXVALUE      0x03 

#define LIS2DH_I1_OVERRUN         0x02      // FiFo Overrun on INT1
#define LIS2DH_I1_INTERRUPT_NONE  0x00
#define LIS2DH_LIR_INT1_MASK  0x08

#define LIS2DH_I2_INTERRUPT_NONE  0x00
#define LIS2DH_I2_MASK            0xF8       // Mask for interrupt

#define LIS2DH12_RANGE_2GA	0x00
#define LIS2DH12_RANGE_4GA	0x10
#define LIS2DH12_RANGE_8GA	0x20
#define LIS2DH12_RANGE_16GA	0x30

#define LIS2DH_FS_SHIFT     4
#define LIS2DH_FS_SCALE_2G  0x00
#define LIS2DH_FS_SCALE_4G  0x01
#define LIS2DH_FS_SCALE_8G  0x02
#define LIS2DH_FS_SCALE_16G 0x03
#define LIS2DH_FS_MAXVALUE  0x03

#define LIS2DH_ODR_SHIFT      4
#define LIS2DH_ODR_POWER_DOWN 0x00
#define LIS2DH_ODR_1HZ        0x01
#define LIS2DH_ODR_10HZ       0x02
#define LIS2DH_ODR_25HZ       0x03
#define LIS2DH_ODR_50HZ       0x04
#define LIS2DH_ODR_100HZ      0x05
#define LIS2DH_ODR_200HZ      0x06
#define LIS2DH_ODR_400HZ      0x07
#define LIS2DH_ODR_1620HZ     0x08
#define LIS2DH_ODR_1344HZ     0x09
#define LIS2DH_ODR_5376HZ     0x0A
#define LIS2DH_ODR_MAXVALUE   0x0A

#define LIS2DH_FIFO_SRC_REG   0x2F
#define LIS2DH_OVRN_FIFO_MASK    0x40

#define I2C_TX_LEN_MAX      24    /**< Maximal number of concurrent bytes for I2C transmission. */

#define INT1_PIN      25

enum
{
    DRV_ACC_STATUS_CODE_SUCCESS,            ///< Successful.
    DRV_ACC_STATUS_CODE_INVALID_PARAM,      ///< Invalid parameters.
    DRV_ACC_STATUS_WRONG_DEVICE,            ///< Wrong device at I2C (TWI) address.
    DRV_ACC_STATUS_UNINITALIZED,            ///< The driver is unitialized, please initialize.
};

/* Mode for LM75B. */
#define NORMAL_MODE 0U

#define CS 27
#define SDA 29
#define SCL 30

static uint8_t m_scale;
static uint8_t m_resolution;
static uint8_t m_frequency;
static uint8_t m_fifo_mode;
static int16_t m_buffer[32][3];

#define FIFO_FULL_SIZE 32

static ret_code_t lis2dh_read_reg(uint8_t reg, uint8_t * const p_content, uint8_t size);

static int16_t lis2dh_read_regs(const uint8_t msb_register, const uint8_t lsb_register);

static uint8_t lis2dh_read_masked_reg(const uint8_t register_addr, const uint8_t mask);

static ret_code_t lis2dh_write_regs(uint8_t first_reg, uint8_t const * const p_contents, uint8_t num_regs);

static ret_code_t lis2dh_write_masked_register_8(const uint8_t register_addr, const uint8_t mask, const bool value) ;

static ret_code_t lis2dh_write_masked_register_I(const int register_addr, const int mask, const int value) ;

int16_t lis2dh_getAxisX(void) ;

int16_t lis2dh_getAxisY(void) ;

int16_t lis2dh_getAxisZ(void);

void lis2dh_readXYZ(int16_t* ax, int16_t* ay, int16_t* az);

ret_code_t lis2dh_mG_conversion(int16_t * ax, int16_t * ay, int16_t * az);

static ret_code_t lis2dh_verify_id(void);

static ret_code_t lis2dh_set_high_resolution_mode(bool hr) ;

bool lis2dh_is_high_resolution_mode() ;

static ret_code_t lis2dh_enable_low_power(void) ;

static ret_code_t lis2dh_disable_low_power(void) ;

static ret_code_t lis2dh_set_data_rate(uint8_t data_rate) ;

ret_code_t lis2dh_set_fifo_mode(uint8_t fifoMode) ;

uint8_t lis2dh_get_fifo_mode() ;

static ret_code_t lis2dh_enable_fifo(bool enable) ;

static ret_code_t lis2dh_enable_INT1_overrun(bool enable);

static ret_code_t lis2dh_set_acceleration_scale(uint8_t scale) ;

static ret_code_t lis2dh_set_resolution_mode(uint8_t res) ;

static ret_code_t lis2dh_disable_all_interrupt();

uint8_t lis2dh_get_acceleration_scale() ;

uint8_t lis2dh_get_data_rate(void) ;

bool lis2dh_is_low_power_enabled(void) ;

uint8_t lis2dh_get_resolution_mode();

ret_code_t lis2dh_reboot() ;

void lis2dh_read_settings() ;

ret_code_t lis2dh_read_fifo(int16_t m_buffer[32][3]) ;

ret_code_t lis2dh_init(int resolution, int frequency, int scale);

void twi_init (void);

uint8_t lis2dh_fifo_restart(void) ;





































#endif // DRV_LIS2DH_H__