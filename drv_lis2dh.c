#include "drv_lis2dh.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
uint8_t reg[1] = {WHO_AM_I};
static uint8_t m_sample;
static uint8_t cmd_read;

/**@brief LIS3DH accelerometer configuration struct.
 */
typedef struct
{
    uint8_t                      twi_addr;        ///< TWI (I2C) address of device.
    nrf_drv_twi_t        const * p_twi_instance;  ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const * p_twi_cfg;       ///< The TWI configuration to use while the driver is enabled.
    uint8_t                      cpu_wake_pin;    ///< Pin on the nRF used to wake from sleep.
} drv_acc_cfg_t;

static struct
{
    drv_acc_cfg_t p_cfg;
} m_acc;


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    nrf_gpio_cfg_output(CS);
    nrf_gpio_pin_set(CS);

    const nrf_drv_twi_config_t twi_lis2dh_config = {
       .scl                = SCL,
       .sda                = SDA, 
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lis2dh_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static ret_code_t lis2dh_read_reg(uint8_t reg, uint8_t * const p_content, uint8_t size)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx( &m_twi,
                               LIS2DH_ADDR,
                               &reg,
                               1,
                               true );

    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx( &m_twi,
                               LIS2DH_ADDR,
                               p_content,
                               size);
    
    APP_ERROR_CHECK(err_code);

    return DRV_ACC_STATUS_CODE_SUCCESS;
}

static int16_t lis2dh_read_regs(const uint8_t msb_register, const uint8_t lsb_register)
{
    ret_code_t err_code;

    uint8_t msb; 
    uint8_t lsb; 

    err_code = lis2dh_read_reg(msb_register, &msb, 1);
    APP_ERROR_CHECK(err_code);

    err_code = lis2dh_read_reg(lsb_register, &lsb, 1);
    APP_ERROR_CHECK(err_code);

    return (((int16_t)msb) << 8) | lsb;
}

static uint8_t lis2dh_read_masked_reg(const uint8_t register_addr, const uint8_t mask) 
{
    ret_code_t err_code;

    uint8_t data;
     
    err_code = lis2dh_read_reg(register_addr, &data, 1);
    APP_ERROR_CHECK(err_code);

    return (data & mask);
}

static ret_code_t lis2dh_write_regs(uint8_t first_reg, uint8_t const * const p_contents, uint8_t num_regs)
{
    ret_code_t  err_code;
    uint8_t     s_tx_buf[I2C_TX_LEN_MAX];

    // Check if supplied data fits in buffer.
    if ( (1 + num_regs) > I2C_TX_LEN_MAX)
    {
        return DRV_ACC_STATUS_CODE_INVALID_PARAM;
    }

    // Multiple read bit cleared by default.
    first_reg &= ~BIT_7;
    if (num_regs > 1)
    {
        // Setting multiple read bit (successive read).
        first_reg |= BIT_7;
    }

    // Data to send: Register address + contents.
    s_tx_buf[0] = first_reg;
    memcpy(&s_tx_buf[1], p_contents, num_regs);

    // Perform SPI transfer.
    err_code = nrf_drv_twi_tx( &m_twi,
                               LIS2DH_ADDR,
                               s_tx_buf,
                               num_regs + 1,
                               false );
    APP_ERROR_CHECK(err_code);

    return DRV_ACC_STATUS_CODE_SUCCESS;
}

/**
 * Change one bit of the given register
 * when value is true, the bit is forced to 1
 * when value is false, the bit is forced to 0
 */
static ret_code_t lis2dh_write_masked_register_8(const uint8_t register_addr, const uint8_t mask, const bool value) 
{
    ret_code_t err_code;
    uint8_t data;
    uint8_t combo;

    err_code = lis2dh_read_reg(register_addr, &data, 1);
    APP_ERROR_CHECK(err_code);

    if(value) {
        combo = (mask | data);
    } else {
        combo = ((~mask) & data);
    }
    
    err_code = lis2dh_write_regs(register_addr, &combo, 1);
    APP_ERROR_CHECK(err_code);

    return DRV_ACC_STATUS_CODE_SUCCESS; 
}

/**
 * Change a register content. The mask is applied to the value. 
 * The conten of the register is read then clear before
 * the value is applied.
 * Value is not shift
 */
static ret_code_t lis2dh_write_masked_register_I(const int register_addr, const int mask, const int value) 
{
    ret_code_t err_code;
    uint8_t data;

    err_code = lis2dh_read_reg(register_addr, &data, 1);
    APP_ERROR_CHECK(err_code);

    uint8_t masked_value = (( data & ~mask) | (mask & value)); 

    err_code = lis2dh_write_regs(register_addr, &masked_value, 1);
    APP_ERROR_CHECK(err_code);

    return DRV_ACC_STATUS_CODE_SUCCESS;
}

int16_t lis2dh_getAxisX(void) 
{
    return lis2dh_read_regs(LIS2DH_OUT_X_H, LIS2DH_OUT_X_L);
}

int16_t lis2dh_getAxisY(void) 
{
    return lis2dh_read_regs(LIS2DH_OUT_Y_H, LIS2DH_OUT_Y_L);
}

int16_t lis2dh_getAxisZ(void) 
{
    return lis2dh_read_regs(LIS2DH_OUT_Z_H, LIS2DH_OUT_Z_L);
}

void lis2dh_readXYZ(int16_t* ax, int16_t* ay, int16_t* az) //read x, y, z data
{
    *ax = lis2dh_getAxisX();
    *ay = lis2dh_getAxisY();
    *az = lis2dh_getAxisZ();

//    NRF_LOG_INFO("Acceleration raw x= 0x%X - y= 0x%X - z= 0x%X ", *ax, *ay, *az);
//    NRF_LOG_FLUSH();

    if ( m_resolution == LIS2DH_RESOLUTION_8B ) 
    {
       *ax>>= 8;
       *ay>>= 8;
       *az>>= 8;
    } else 
    {
       int shift = (m_resolution == LIS2DH_RESOLUTION_10B)?6:4;
       *ax>>= shift;
       *ay>>= shift;
       *az>>= shift;
    }

//    NRF_LOG_INFO("Acceleration raw shifted x= 0x%X - y= 0x%X - z= 0x%X ", *ax, *ay, *az);
//    NRF_LOG_FLUSH();

    int16_t maxValue;
    
    switch ( m_resolution ) 
    {
     case LIS2DH_RESOLUTION_8B:
          maxValue = 128;
          break;
    case LIS2DH_RESOLUTION_10B:
          maxValue = 512;
          break;
    case LIS2DH_RESOLUTION_12B:
          maxValue = 2048;
          break;
    }

    switch ( m_scale ) 
    {
      case LIS2DH_FS_SCALE_2G:
           *ax = (*ax*2000)/maxValue; 
           *ay = (*ay*2000)/maxValue; 
           *az = (*az*2000)/maxValue; 
           break;
      case LIS2DH_FS_SCALE_4G:
           *ax = (*ax*4000)/maxValue; 
           *ay = (*ay*4000)/maxValue; 
           *az = (*az*4000)/maxValue; 
           break;
      case LIS2DH_FS_SCALE_8G:
           *ax = (*ax*8000)/maxValue; 
           *ay = (*ay*8000)/maxValue; 
           *az = (*az*8000)/maxValue; 
           break;
      case LIS2DH_FS_SCALE_16G:
           *ax = (*ax*16000)/maxValue; 
           *ay = (*ay*16000)/maxValue; 
           *az = (*az*16000)/maxValue; 
           break;   
    }

}

ret_code_t lis2dh_mG_conversion(int16_t * ax, int16_t * ay, int16_t * az) {

  if ( m_resolution > LIS2DH_RESOLUTION_MAXVALUE || m_scale > LIS2DH_FS_MAXVALUE ) return false;

  int16_t maxValue;
  
  bool    ret = true;
  switch ( m_resolution ) {
     case LIS2DH_RESOLUTION_8B:
          maxValue = 128;
          break;
    case LIS2DH_RESOLUTION_10B:
          maxValue = 512;
          break;
    case LIS2DH_RESOLUTION_12B:
          maxValue = 2048;
          break;
  }

  switch ( m_scale) {
    case LIS2DH_FS_SCALE_2G:
         *ax = (*ax*2000)/maxValue; 
         *ay = (*ay*2000)/maxValue; 
         *az = (*az*2000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_4G:
         *ax = (*ax*4000)/maxValue; 
         *ay = (*ay*4000)/maxValue; 
         *az = (*az*4000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_8G:
         *ax = (*ax*8000)/maxValue; 
         *ay = (*ay*8000)/maxValue; 
         *az = (*az*8000)/maxValue; 
         break;
    case LIS2DH_FS_SCALE_16G:
         *ax = (*ax*16000)/maxValue; 
         *ay = (*ay*16000)/maxValue; 
         *az = (*az*16000)/maxValue; 
         break;            
  }

//  NRF_LOG_INFO("Acceleration mG x= %d mg - y= %d mg - z= %d mg", *ax, *ay, *az);
//  NRF_LOG_FLUSH();

  return NRF_SUCCESS;
}

static ret_code_t lis2dh_verify_id(void)
{
    ret_code_t  err_code;
    uint8_t     reg_val;

    err_code = lis2dh_read_reg(WHO_AM_I, &reg_val, 1);
    APP_ERROR_CHECK(err_code);

    return (reg_val == I_AM_LIS2DH);
}

//***************resolution mode************************************************

static ret_code_t lis2dh_set_high_resolution_mode(bool hr) 
{
  if ( hr) m_resolution = LIS2DH_RESOLUTION_12B;
  
  ret_code_t  err_code;

  err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK,hr); 
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

bool lis2dh_is_high_resolution_mode() 
{
  return ( lis2dh_read_masked_reg(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 );
}


//***************low power mode************************************************

static ret_code_t lis2dh_enable_low_power(void) 
{
  ret_code_t  err_code;

  err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, true); 
  APP_ERROR_CHECK(err_code);

  err_code = lis2dh_set_high_resolution_mode(false);
  APP_ERROR_CHECK(err_code);
   
  return NRF_SUCCESS;
}

static ret_code_t lis2dh_disable_low_power(void) 
{
  ret_code_t  err_code;

  err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, false); 
  APP_ERROR_CHECK(err_code);
   
  return NRF_SUCCESS;
}

//***************data rate************************************************

static ret_code_t lis2dh_set_data_rate(uint8_t data_rate) 
{
    ret_code_t  err_code;

    if ( data_rate > LIS2DH_ODR_MAXVALUE ) return NRF_ERROR_INVALID_PARAM;

    m_frequency = data_rate;
    data_rate <<= LIS2DH_ODR_SHIFT;
    
    err_code = lis2dh_write_masked_register_I(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK, data_rate);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

//***************FIFO mode************************************************

ret_code_t lis2dh_set_fifo_mode(uint8_t fifoMode) 
{
    ret_code_t  err_code;

    if(fifoMode > LIS2DH_FM_MAXVALUE) 
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_fifo_mode = fifoMode;
    fifoMode = fifoMode << LIS2DH_FM_SHIFT;

    err_code = lis2dh_write_masked_register_I(LIS2DH_FIFO_CTRL_REG, LIS2DH_FM_MASK, fifoMode);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS; 
}

/**
 * Get the fifoMode from register
 */
uint8_t lis2dh_get_fifo_mode() 
{
   uint8_t v = lis2dh_read_masked_reg(LIS2DH_FIFO_CTRL_REG, LIS2DH_FM_MASK);
   v >>= LIS2DH_FM_SHIFT;
   return v;
}

/**
 * Enable / Disable Fifo
 */
static ret_code_t lis2dh_enable_fifo(bool enable) 
{
   ret_code_t  err_code;

   err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG5, LIS2DH_FIFO_EN_MASK, enable);
   APP_ERROR_CHECK(err_code);

   return NRF_SUCCESS;
}


/**
 * Enable / Disable IT1 interrupt when FIFO full
 */
static ret_code_t lis2dh_enable_INT1_overrun(bool enable) 
{
   ret_code_t  err_code;

   err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG3, LIS2DH_I1_OVERRUN, enable);
   APP_ERROR_CHECK(err_code);

//   err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG5, LIS2DH_LIR_INT1_MASK, enable);
//   APP_ERROR_CHECK(err_code);

   return NRF_SUCCESS;
}


/**
 * Set the accelerations scale
 */
static ret_code_t lis2dh_set_acceleration_scale(uint8_t scale) 
{
    ret_code_t  err_code;

    if(scale > LIS2DH_FS_MAXVALUE) {
        return false;
    }

    m_scale = scale;
    scale = scale << LIS2DH_FS_SHIFT;

    err_code = lis2dh_write_masked_register_I(LIS2DH_CTRL_REG4, LIS2DH_FS_MASK, scale);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

/**
 * Set the expected acceleration resolution in bit
 * Possible option : LIS2DH_RESOLUTION_XXB (8,10,12)
 * This is changing the LowPower mode and HighResolution mode
 */
static ret_code_t lis2dh_set_resolution_mode(uint8_t res) 
{
  ret_code_t err_code;

  switch (res) {
    default:
    case LIS2DH_RESOLUTION_8B:
          err_code = lis2dh_set_high_resolution_mode(false);
          APP_ERROR_CHECK(err_code);
          err_code = lis2dh_enable_low_power();
          APP_ERROR_CHECK(err_code);
          m_resolution = LIS2DH_RESOLUTION_8B;
      break;
    case LIS2DH_RESOLUTION_10B:
          err_code = lis2dh_disable_low_power();
          APP_ERROR_CHECK(err_code);
          err_code = lis2dh_set_high_resolution_mode(false);
          APP_ERROR_CHECK(err_code);
          m_resolution = LIS2DH_RESOLUTION_10B;
      break;
    case LIS2DH_RESOLUTION_12B:
          err_code = lis2dh_disable_low_power();
          APP_ERROR_CHECK(err_code);
          err_code = lis2dh_set_high_resolution_mode(true);
          APP_ERROR_CHECK(err_code);
          m_resolution = LIS2DH_RESOLUTION_12B;
      break;
  }

  return NRF_SUCCESS;
}

/**
 * Disable all interrupts
 */
static ret_code_t lis2dh_disable_all_interrupt()
{
  ret_code_t err_code;

  err_code = lis2dh_write_regs(LIS2DH_CTRL_REG3,LIS2DH_I1_INTERRUPT_NONE, 1);
  APP_ERROR_CHECK(err_code);

  err_code = lis2dh_write_masked_register_I(LIS2DH_CTRL_REG6, LIS2DH_I2_MASK, LIS2DH_I2_INTERRUPT_NONE);
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static bool lis2dh_is_fifo_full() {
  return ( lis2dh_read_masked_reg(LIS2DH_FIFO_SRC_REG, LIS2DH_OVRN_FIFO_MASK) > 0 ); 
}

/////////////////////Getter functions 
uint8_t lis2dh_get_acceleration_scale() 
{
    uint8_t v = lis2dh_read_masked_reg(LIS2DH_CTRL_REG4,LIS2DH_FS_MASK);
    v >>= LIS2DH_FS_SHIFT;
    return v;
}

uint8_t lis2dh_get_data_rate(void) 
{
    uint8_t v = lis2dh_read_masked_reg(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK);
    v >>= LIS2DH_ODR_SHIFT;
    return v;
}

bool lis2dh_is_low_power_enabled(void) 
{
    return (    lis2dh_read_masked_reg(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK) != 0 
             && !lis2dh_is_high_resolution_mode() );
}

uint8_t lis2dh_get_resolution_mode() 
{
  if ( lis2dh_is_high_resolution_mode() ) return LIS2DH_RESOLUTION_12B;
  else if ( lis2dh_is_low_power_enabled() ) return LIS2DH_RESOLUTION_8B;
  else return LIS2DH_RESOLUTION_10B;
}

ret_code_t lis2dh_reboot() 
{
  ret_code_t err_code;

  err_code = lis2dh_write_masked_register_8(LIS2DH_CTRL_REG5,LIS2DH_BOOT_MASK,true); 
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

void lis2dh_read_settings() 
{
  uint8_t data;
  data = lis2dh_get_resolution_mode();
  switch(data)
  {
    case LIS2DH_RESOLUTION_8B:
        NRF_LOG_INFO("Resolution : 8B");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_RESOLUTION_10B:
        NRF_LOG_INFO("Resolution : 10B ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_RESOLUTION_12B:
        NRF_LOG_INFO("Resolution : 12B ");
        NRF_LOG_FLUSH();
        break;
  }

  data = lis2dh_get_data_rate();
  switch(data)
  {
    case LIS2DH_ODR_1HZ:
        NRF_LOG_INFO("Frequency : 1Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_10HZ:
        NRF_LOG_INFO("Frequency : 10Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_25HZ:
        NRF_LOG_INFO("Frequency : 25Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_50HZ:
        NRF_LOG_INFO("Frequency : 50Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_100HZ:
        NRF_LOG_INFO("Frequency : 100Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_200HZ:
        NRF_LOG_INFO("Frequency : 200Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_400HZ:
        NRF_LOG_INFO("Frequency : 400Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_1620HZ:
        NRF_LOG_INFO("Frequency : 1620Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_1344HZ:
        NRF_LOG_INFO("Frequency : 1344Hz ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_ODR_5376HZ:
        NRF_LOG_INFO("Resolution : 5376Hz ");
        NRF_LOG_FLUSH();
        break;
  }

  data = lis2dh_get_acceleration_scale();
  
  switch(data)
  {
    case LIS2DH_FS_SCALE_2G:
        NRF_LOG_INFO("Scale: 2G ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FS_SCALE_4G:
        NRF_LOG_INFO("Scale: 4G ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FS_SCALE_8G:
        NRF_LOG_INFO("Scale: 8G ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FS_SCALE_16G:
        NRF_LOG_INFO("Scale: 16G ");
        NRF_LOG_FLUSH();
        break;
  }

  data = lis2dh_get_fifo_mode();
  switch(data)
  {
    case LIS2DH_FM_BYPASS:
        NRF_LOG_INFO("FiFo Mode : BYPASS \r\n");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FM_FIFO:
        NRF_LOG_INFO("FiFo Mode : FIFO \r\n ");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FM_STREAM:
        NRF_LOG_INFO("FiFo Mode : STREAM \r\n");
        NRF_LOG_FLUSH();
        break;

    case LIS2DH_FM_STREAMFIFO:
        NRF_LOG_INFO("FiFo Mode : STREAMFIFO \r\n");
        NRF_LOG_FLUSH();
        break;

  }
 
}


/**
 * Read 16b data from the Fifo in bust/sequential mode
 * The data are store in a int8/6_t[][3] regarding the RESOLUTION MODE
 * Empty the fifo is the buffer size is larger. maxSz is the max number
 * of X,Y,Z triple
 * Return the number of tripe read from Fifo 
 */ 
ret_code_t lis2dh_read_fifo(int16_t m_buffer[32][3]) 
{
  ret_code_t err_code;

  int16_t (*buffer16)[3] = m_buffer;

  //int16_t buffer16[32][3]; 
  //int16_t (*buffer16)[3] = (int16_t (*)[3]) _buffer;  
  int k = 0;
  int transfert_size = 196;
  uint8_t const data[transfert_size];
  uint8_t const registerAdrr[1]={0x80|LIS2DH_OUT_X_L};

  //Read the most quick Fifo
  err_code = nrf_drv_twi_tx( &m_twi,
                               LIS2DH_ADDR,
                               registerAdrr,
                               1,
                               true );
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_twi_rx( &m_twi,
                               LIS2DH_ADDR,
                               data,
                               sizeof(data));
  APP_ERROR_CHECK(err_code);
   
  for ( int i = 0 ; i < transfert_size/6 ; i++ ) 
  {
     if ( m_resolution == LIS2DH_RESOLUTION_8B )
     {
        buffer16[i][0] = (int8_t)data[(6*i)+1];
        buffer16[i][1] = (int8_t)data[(6*i)+3];
        buffer16[i][2] = (int8_t)data[(6*i)+5];
     }else{
       int shift = ( m_resolution == LIS2DH_RESOLUTION_10B)?6:4;
        
       buffer16[i][0] = data[(6*i)];
       buffer16[i][0] += (int16_t)data[(6*i) + 1] << 8;
       buffer16[i][0] >>= shift;
       buffer16[i][1] = data[(6*i) + 2];
       buffer16[i][1] += (int16_t)data[(6*i) + 3] << 8;
       buffer16[i][1] >>= shift;
       buffer16[i][2] = data[(6*i) + 4];
       buffer16[i][2] += (int16_t)data[(6*i) + 5] << 8;
       buffer16[i][2] >>= shift;           
     }  

//     NRF_LOG_INFO("buffer16[%d][0] = 0x%X ", i, buffer16[i][0]);
//     NRF_LOG_FLUSH();
//     NRF_LOG_INFO("buffer16[%d][1] = 0x%X ", i, buffer16[i][1]);
//     NRF_LOG_FLUSH();
//     NRF_LOG_INFO("buffer16[%d][2] = 0x%X ", i, buffer16[i][2]);
//     NRF_LOG_FLUSH();
  }

  return NRF_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
//                          HIGH LEVEL FUNCTIONS                             //
///////////////////////////////////////////////////////////////////////////////

ret_code_t lis2dh_init(int resolution, int frequency, int scale)
{
    ret_code_t err_code;

    // Check correct ID.
    if (!lis2dh_verify_id())
    {
        return DRV_ACC_STATUS_WRONG_DEVICE;
    }

    //err_code &= lis2dh_reboot();

    err_code &= lis2dh_set_data_rate(frequency);
    err_code &= lis2dh_set_resolution_mode(resolution);
    err_code &= lis2dh_set_acceleration_scale(scale);
    err_code &= lis2dh_enable_INT1_overrun(true);
    err_code &= lis2dh_enable_fifo(true);
    err_code &= lis2dh_set_fifo_mode(LIS2DH_FM_BYPASS);
    //err_code &= lis2dh_set_fifo_mode(LIS2DH_FM_FIFO);
    APP_ERROR_CHECK(err_code);
   
    //lis2dh_read_settings();
    //nrf_delay_ms(10);

    return NRF_SUCCESS;
}

/**
 * Get the fifoMode from register
 */
uint8_t lis2dh_fifo_restart(void) 
{
    ret_code_t err_code;

    err_code = lis2dh_set_fifo_mode(LIS2DH_FM_BYPASS);
    APP_ERROR_CHECK(err_code);

    err_code = lis2dh_set_fifo_mode(LIS2DH_FM_FIFO);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

uint32_t lis2dh_config(lis2dh_cfg_t * p_cfg)
{
    uint32_t err_code;

    if (p_cfg == NULL)
    {
        return NRF_ERROR_NULL;
    }

    m_lis2dh.resolution      = p_cfg->resolution;
    m_lis2dh.frequency       = p_cfg->frequency;
    m_lis2dh.scale           = p_cfg->scale;

    NRF_LOG_INFO("lis2dh config_default");
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("resolution: %d, frequency: %d, scale: %d \r\n", m_lis2dh.resolution, m_lis2dh.frequency, m_lis2dh.scale);
    NRF_LOG_FLUSH();

    return NRF_SUCCESS;
}

ret_code_t lis2dh_enable()
{
    ret_code_t err_code;

    NRF_LOG_INFO("lis2dh enable \n");
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("resolution: %d, frequency: %d, scale: %d", m_lis2dh.resolution, m_lis2dh.frequency, m_lis2dh.scale);
    NRF_LOG_FLUSH();

    err_code = lis2dh_init(m_lis2dh.resolution, m_lis2dh.frequency, m_lis2dh.scale);
    APP_ERROR_CHECK(err_code);

    err_code = lis2dh_set_fifo_mode(LIS2DH_FM_FIFO);
    APP_ERROR_CHECK(err_code);

    m_lis2dh.running = true;
    
    return NRF_SUCCESS;
}

ret_code_t lis2dh_disable()
{
    ret_code_t err_code;

    err_code = lis2dh_init(LIS2DH_RESOLUTION_8B, LIS2DH_ODR_POWER_DOWN, LIS2DH_FS_SCALE_2G);
    APP_ERROR_CHECK(err_code);

    m_lis2dh.running = false;
}