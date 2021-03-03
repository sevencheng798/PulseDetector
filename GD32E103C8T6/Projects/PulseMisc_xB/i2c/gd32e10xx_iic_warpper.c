/*-------gd32e103c_iic_warpper.c-----------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "gd32e10xx_iic_warpper.h"

#define I2C_MEM_ADD_MSB(__ADDRESS__)              ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))

/* A lock that allow to operator i2c interface. */
QueueHandle_t g_Hal_I2C_operator_request_lock = NULL;

HAL_StatusTypeDef HAL_I2C_Master_Transmit
(
    uint32_t i2c_periph,
    uint16_t DevAddress,
    uint8_t *pSndData,
    uint16_t length,
    uint32_t timeout
);

HAL_StatusTypeDef HAL_I2C_Master_Transmit_request
(
    uint32_t i2c_periph,
    uint16_t DevAddress,
    uint16_t ComRegAddr, 
    uint16_t ComRegAddrSize,
    uint8_t *pSndData, 
    uint16_t length,
    uint32_t timeout
);

HAL_StatusTypeDef HAL_I2C_Master_Receive(
  uint32_t i2c_periph,
  uint16_t DevAddress,
  uint8_t *pRecvData,
  uint16_t length,
  uint32_t timeout
);

static HAL_StatusTypeDef I2C_RequestMemoryWrite(uint32_t i2c_periph, uint16_t ComRegAddr, uint16_t ComRegAddrSize, uint32_t Timeout, uint32_t Tickstart) ;
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(uint32_t hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);

void rcu_config(void);
void gpio_config(void);
void i2c_config(void);

#ifdef GD_SOFT_SIMULATION_IIC

#define IIC_SCL_PIN                         GPIO_PIN_6
#define IIC_SCL_GPIO_PORT                   GPIOB
#define IIC_SCL_GPIO_CLK                    RCU_GPIOB

#define IIC_SDA_PIN                         GPIO_PIN_7
#define IIC_SDA_GPIO_PORT                   GPIOB
#define IIC_SDA_GPIO_CLK                    RCU_GPIOB

#define SDA_IN()  {gpio_init(IIC_SDA_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, IIC_SDA_PIN);}
#define SDA_OUT() {gpio_init(IIC_SDA_GPIO_PORT,  GPIO_MODE_OUT_PP,  GPIO_OSPEED_50MHZ,  IIC_SDA_PIN);}

static void I2C_SoftDelay(uint32_t nCount);
static void iic_delay_us(int us);
static void iic_scl_h(void);
static void iic_scl_l(void);
static void iic_sda_write(unsigned char bit);
static unsigned char iic_sda_read(void);
static void iic_start(void);
static void iic_stop(void);
static uint8_t iic_wait_ack(void);
static void iic_ack(uint8_t ack_nack);
static uint8_t iic_read_byte(uint8_t ack);
static void iic_write_byte(uint8_t data);
static void iic_soft_init();
HAL_StatusTypeDef I2C_SoftWriteNBytes(uint16_t addr, uint8_t reg, uint8_t len, uint8_t * data);
HAL_StatusTypeDef I2C_SoftReadNBytes(uint16_t addr, uint16_t reg, uint8_t len, uint8_t *buf);
void I2C_SoftInit();

#endif

/**
// | S | SlaveAddr | 0 | A | CommandData | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
**/
HAL_StatusTypeDef SendCommand
(
    uint16_t DevAddress,
    uint8_t *pComData,
    uint16_t length
) {
    return HAL_I2C_Master_Transmit(I2C0, DevAddress, pComData, length, 0xFF);
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit
(
    uint32_t i2c_periph,
    uint16_t DevAddress,
    uint8_t *pSndData,
    uint16_t length,
    uint32_t timeout
) {
#ifdef GD_SOFT_SIMULATION_IIC
	return I2C_SoftWriteNBytes(DevAddress, 0, length, pSndData);
#else
  return HAL_I2C_Master_Transmit_request(i2c_periph, DevAddress, 0, 0, pSndData, length, timeout);
#endif
}

/**
// | S | SlaveAddr | 0 | A | ComReg | A | CommandData | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
**/
HAL_StatusTypeDef SendRegCommand
(
    uint16_t DevAddress,
    uint16_t ComRegAddr, 
    uint16_t ComRegAddrSize,
    uint8_t *pComData, 
    uint16_t Size
) {
#ifdef GD_SOFT_SIMULATION_IIC
	return I2C_SoftWriteNBytes(DevAddress, ComRegAddr, Size, pComData);
#else    
    return HAL_I2C_Master_Transmit_request(I2C0, DevAddress, ComRegAddr, ComRegAddrSize, pComData, Size, 0xFF);
#endif
}

/**
// | S | SlaveAddr | 0 | A | CommandData | A | S | SlaveAddr | 1 | A | data[7:0] | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
 */
HAL_StatusTypeDef ReceiveCommand
(
    uint16_t DevAddress, 
    uint8_t *pComData,
    uint16_t length
){
#ifdef GD_SOFT_SIMULATION_IIC
    return I2C_SoftReadNBytes(DevAddress, 0, length, pComData);
#else 
    return HAL_I2C_Master_Receive(I2C0, DevAddress, pComData, length, 0xFF);
#endif
}

HAL_StatusTypeDef ReceiveRegCommand
(
    uint16_t DevAddress, 
    uint16_t ComRegAddr, 
    uint16_t ComRegAddrSize,
    uint8_t *pComData,
    uint16_t length
){
#ifdef GD_SOFT_SIMULATION_IIC
    return I2C_SoftReadNBytes(DevAddress, ComRegAddr, length, pComData);
#else 
    return HAL_I2C_Master_Receive(I2C0, DevAddress, pComData, length, 0xFF);
#endif
}


HAL_StatusTypeDef HAL_I2C_Master_Receive(
  uint32_t i2c_periph,
  uint16_t DevAddress,
  uint8_t *pRecvData,
  uint16_t length,
  uint32_t timeout
){
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t i = 0;
    uint32_t tickstart;

    /* Request lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreTake(g_Hal_I2C_operator_request_lock, portMAX_DELAY);
   
    do {        
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    
    /* send a NACK for the next data byte which will be received into the shift register */
    i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
    /* wait until I2C bus is idle */
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_I2CBSY, SET, timeout, tickstart) != HAL_OK)
    {
       status = HAL_BUSY;
       break;
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_SBSEND, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, DevAddress, I2C_RECEIVER);
    /* disable ACK before clearing ADDSEND bit */
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    /* wait until ADDSEND bit is set */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_ADDSEND, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    
    /* Wait until the last data byte is received into the shift register */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_BTC, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }
    
    for(i=0; i< length; i++) {
      /* wait until the RBNE bit is set */
      //while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
      if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_RBNE, RESET, timeout, tickstart) != HAL_OK)
      {
        break;
      }
      /* read a data from I2C_DATA */
      pRecvData[i++] = i2c_data_receive(I2C0);
    }
    
    /* send a stop condition */
    i2c_stop_on_bus(I2C0);

    /* wait until stop condition generate */ 
    while(I2C_CTL0(I2C0)&0x0200);
    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    
    /* Set flag success */
    status = HAL_OK;
    }while(0);
    
    /* Release lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreGive(g_Hal_I2C_operator_request_lock);
    
    return status;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit_request
(
    uint32_t i2c_periph,
    uint16_t DevAddress,
    uint16_t ComRegAddr, 
    uint16_t ComRegAddrSize,
    uint8_t *pSndData,
    uint16_t length,
    uint32_t timeout
) {
    uint32_t i;
    uint32_t tickstart;
    HAL_StatusTypeDef status = HAL_ERROR;
    
    /* Request lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreTake(g_Hal_I2C_operator_request_lock, portMAX_DELAY);
      
    do {
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    
     /* wait until I2C bus is idle */
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_I2CBSY, SET, timeout, tickstart) != HAL_OK)
    {
       status = HAL_BUSY;
       break;
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_SBSEND, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, DevAddress, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_ADDSEND, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }

    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_TBE, RESET, timeout, tickstart) != HAL_OK)
    {
       break;
    }

    /* Master sends target device address followed by internal memory address for write request. */
    if(ComRegAddrSize != 0)
      I2C_RequestMemoryWrite(I2C0, ComRegAddr, ComRegAddrSize, timeout, tickstart);
    
    for(i=0; i<length; i++){
        /* data transmission */
        i2c_data_transmit(I2C0, pSndData[i]);
        /* wait until the TBE bit is set */
        //while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
        if(I2C_WaitOnFlagUntilTimeout(I2C0, I2C_FLAG_TBE, RESET, timeout, tickstart) != HAL_OK)
        {
           break;
        }
    }

    /* send a stop condition to I2C bus */
    //i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    //while(I2C_CTL0(I2C0)&0x0200);

    status = HAL_OK;
    }while(0);

    /* No matter what the status is, a stop condition must be sent*/
    /* send a stop condition to I2C bus */
    if(status != HAL_BUSY) {
        i2c_stop_on_bus(I2C0);
        /* wait until stop condition generate */
        while(I2C_CTL0(I2C0)&0x0200);
    }
    
    /* Release lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreGive(g_Hal_I2C_operator_request_lock);
    
    return status;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  */
static HAL_StatusTypeDef I2C_RequestMemoryWrite(uint32_t i2c_periph, uint16_t ComRegAddr, uint16_t ComRegAddrSize, uint32_t Timeout, uint32_t Tickstart) 
{
    if(ComRegAddrSize == 1)  // mem regierster addr is 8bit
    {
        /* data transmission */
        i2c_data_transmit(i2c_periph, ComRegAddr);

    } else  // mem regierster addr is 16bit
    {
        /* data transmission */
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_MSB(ComRegAddr));
        /* wait until the TBE bit is set */
        //while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
        if(I2C_WaitOnFlagUntilTimeout(i2c_periph, I2C_FLAG_TBE, RESET, Timeout, Tickstart) != HAL_OK)
        {
           return HAL_TIMEOUT;
        }
        /* data transmission */
        i2c_data_transmit(i2c_periph, I2C_MEM_ADD_LSB(ComRegAddr));        
    }
    
    /* wait until the TBE bit is set */
    //while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    if(I2C_WaitOnFlagUntilTimeout(i2c_periph, I2C_FLAG_TBE, RESET, Timeout, Tickstart) != HAL_OK)
    {
       return HAL_TIMEOUT;
    }
    
    return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(uint32_t hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* wait until I2C bus is idle */
  while(i2c_flag_get(hi2c, (i2c_flag_enum)Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        /* Process Unlocked */
        //__HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C0_Init(void)
{
#ifndef GD_SOFT_SIMULATION_IIC
  /* USER CODE BEGIN I2C2_Init 1 */
  /* RCU configure */
  rcu_config();
  /* GPIO configure */
  gpio_config();
  /* I2C configure */
  i2c_config();
#else
  I2C_SoftInit();
#endif  
  /* Create i2c operator lock */
  if(NULL == g_Hal_I2C_operator_request_lock)
     g_Hal_I2C_operator_request_lock = xSemaphoreCreateMutex();
  
  /* USER CODE END I2C2_Init 1 */
}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      configure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

#ifdef GD_SOFT_SIMULATION_IIC
static void I2C_SoftDelay(uint32_t nCount)
{
    nCount=nCount*4;
    for(; nCount != 0; nCount--);
}

static void iic_delay_us(int us)
{
    I2C_SoftDelay(us);
}

static void iic_scl_h(void)
{
    //GPIO_BOP(IIC_SCL_GPIO_PORT) = IIC_SCL_PIN;
    gpio_bit_set(IIC_SCL_GPIO_PORT, IIC_SCL_PIN);
}

static void iic_scl_l(void)
{
    //GPIO_BC(IIC_SCL_GPIO_PORT) = IIC_SCL_PIN;
    gpio_bit_reset(IIC_SCL_GPIO_PORT, IIC_SCL_PIN);
}

static void iic_sda_write(unsigned char bit)
{
    if (bit == 0)
        gpio_bit_reset(IIC_SDA_GPIO_PORT, IIC_SDA_PIN);
    else
        gpio_bit_set(IIC_SDA_GPIO_PORT, IIC_SDA_PIN);
}

static unsigned char iic_sda_read(void)
{
    if (RESET == gpio_input_bit_get(IIC_SDA_GPIO_PORT, IIC_SDA_PIN))
        return 0x00;
    else
        return 0x01;


}

static void iic_start(void)
{
    SDA_OUT();
    iic_sda_write(1);
    iic_delay_us(1);

    iic_scl_h();
    iic_delay_us(4);

    iic_sda_write(0);
    iic_delay_us(4);

    iic_scl_l();
    iic_delay_us(1);
}


static void iic_stop(void)
{
    SDA_OUT();
    iic_scl_l();
    iic_delay_us(1);

    iic_sda_write(0);
    iic_delay_us(4);

    iic_scl_h();
    iic_delay_us(4);

    iic_sda_write(1);
    iic_delay_us(1);
}

static uint8_t iic_wait_ack(void)
{
    uint32_t retry = 0;
    SDA_IN();
    iic_sda_write(0x01);
    iic_delay_us(1);
    iic_scl_h();
    iic_delay_us(1);

    while (iic_sda_read() == 0x01)
    {
        retry++;
        //iic_delay_us(1);
        if (retry > 1000)   break;
    }

    if (iic_sda_read() == 0)
    {
        iic_scl_l();
        iic_delay_us(1);
        return 0;
    }
    else
    {
        //printf("wait ack failed \r\n");
        iic_stop();
        return 1;
    }
}

static void iic_ack(uint8_t ack_nack)
{
    iic_scl_l();
    SDA_OUT();
    iic_sda_write(ack_nack);
    iic_delay_us(1);
    iic_scl_h();
    iic_delay_us(4);
    //if(!iic_sda_read()) printf("set ack\r\n");
    iic_scl_l();
    iic_delay_us(1);
}


static uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, data;
    SDA_IN();
    
    for (i = 0, data = 0; i < 8; i++)
    {
        data <<= 1;
        iic_scl_h();
        iic_delay_us(1);
        data |= iic_sda_read();
        iic_scl_l();
        iic_delay_us(1);
    }

    return data;
}

static void iic_write_byte(uint8_t data)
{
    int i;
    SDA_OUT();
    iic_scl_l();
    for (i = 0; i < 8; i++)
    {
        iic_sda_write((data & 0x80) >> 7);
        iic_delay_us(1);
        iic_scl_h();
        iic_delay_us(1);
        iic_scl_l();
        iic_delay_us(1);
        data <<= 1;
    }
}

static void iic_soft_init() {
    /* Enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(IIC_SCL_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, IIC_SCL_PIN);
    gpio_init(IIC_SDA_GPIO_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, IIC_SDA_PIN);

    /* connect PB7 to I2C0_SDA */	
    gpio_bit_set(GPIOB, GPIO_PIN_7);   // IIC IDLE
    /* connect PB6 to I2C0_SCL */
    gpio_bit_set(GPIOB, GPIO_PIN_6);   // IIC IDLE
		  
}

HAL_StatusTypeDef I2C_SoftWriteNBytes(uint16_t addr, uint8_t reg, uint8_t len, uint8_t * data){
    HAL_StatusTypeDef result = HAL_ERROR;
    int i;
    /* Current only support 8bits addr,
     * A 16 bit variable is designed to be compatible with the user interface.
     */
    uint8_t slaveAddr = (addr & 0xff);

        /* Request lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
        xSemaphoreTake(g_Hal_I2C_operator_request_lock, portMAX_DELAY);
    do {
    /* Generate Start */
    iic_start();

    /* Send write addr that low bit is 0 */	
    iic_write_byte(slaveAddr);	   
    if(iic_wait_ack()) break;

    /* Specified a write register */	
    if(reg != 0) {
      iic_write_byte(reg);
      if(iic_wait_ack() == 1) break;	
    }

    for(i=0;i<len;i++)
    {
        iic_write_byte(data[i]);
        if(iic_wait_ack()) break; 
    }

    /* Generate STOP */
    iic_stop();
    
    result = HAL_OK;
    }while(0);
    /* Release lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreGive(g_Hal_I2C_operator_request_lock);
    
    return result;
}

HAL_StatusTypeDef I2C_SoftReadNBytes(uint16_t addr, uint16_t reg, uint8_t len, uint8_t *buf)
{
    HAL_StatusTypeDef result = HAL_ERROR;
    int i;
    /* Current only support 8bits addr,
     * A 16 bit variable is designed to be compatible with the user interface.
     */
    uint8_t slaveAddr = (addr & 0xff);
    uint8_t slaveReg = (reg & 0xff);
        
    /* Request lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
        xSemaphoreTake(g_Hal_I2C_operator_request_lock, portMAX_DELAY);
    
    do {
	/* Generate Start */
	iic_start();

	/* Send write addr that low bit is 0 */
	iic_write_byte(slaveAddr & 0xfe);	   
	if(iic_wait_ack()) {
          break; 
        }
        
	if(slaveReg != 0) {
	    iic_write_byte(slaveReg);
	    if(iic_wait_ack()) break;
	}
        
        
	/* Re-start */
	iic_start();
        
	/* Send read addr that low bit is 1 */
        iic_write_byte(slaveAddr);  // | 1 read op.         	   
	if(iic_wait_ack()) break; 
        
        for(i=0;i<len-1;i++) 
	{
           *buf=iic_read_byte(1);	
            buf++;
            iic_ack(0);
        }
        
        /* Send a noack */
	*buf = iic_read_byte(0);

        iic_ack(1);
        
        /* Generate STOP */
        iic_stop();
        
        result = HAL_OK;
    }while(0);

    /* Release lock */
    if(NULL != g_Hal_I2C_operator_request_lock)
      xSemaphoreGive(g_Hal_I2C_operator_request_lock);
    
    return result;
}

void I2C_SoftInit() {
    iic_soft_init();
}

#endif

