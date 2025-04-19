#include "stm32g4Board.h"

DRV8317 :: DRV8317()
{

}

void DRV8317 :: init(void)
{
  HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(1000);
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void DRV8317 :: setPWM(float d_a, float d_b, float d_c){

  d_a = d_a > 0.9 ? 0.9 : d_a;
  d_b = d_b > 0.9 ? 0.9 : d_b;
  d_c = d_c > 0.9 ? 0.9 : d_c;

  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_c * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_b * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_a * htim1.Instance->ARR);
  __enable_irq();
}



MT6835 :: MT6835()
{

}

void MT6835::init(void)
{

}

HAL_StatusTypeDef MT6835::readRawAngle(void){

  uint8_t tx_buf[2] = {CMD_CONT_READ_ANGLE, ADDR_ANGLE_START};
  uint8_t rx_buf[4] = {0};

  // 拉低CSN
  HAL_GPIO_WritePin(MT6835_CS_PORT, MT6835_CS_PIN, GPIO_PIN_RESET);

  // 发送连续读取命令（0xA0 + 0x03）
  HAL_SPI_Transmit(&hspi1, tx_buf, 2, SPI_TIMEOUT);

  // 接收4字节数据（0x003~0x006）
  HAL_SPI_Receive(&hspi1, rx_buf, 4, SPI_TIMEOUT);

  // 拉高CSN
  HAL_GPIO_WritePin(MT6835_CS_PORT, MT6835_CS_PIN, GPIO_PIN_SET);

  if ( rx_buf[3] == crc_table(rx_buf,3))
  {
    _rawAngle = ((rx_buf[0] << 16) | 
    (rx_buf[1] << 8) | 
    (rx_buf[2] & 0xF8)) >> 3; // 取高21位

    _status = rx_buf[2] & 0x03;
  }
  else
  {
    return HAL_ERROR;
  }

  // {
  //   memcpy(_mt6835.data, rx_buf, 4);
    
  // }



  // char buffer[20];
  // sprintf(buffer, "float: %f\n", ((angle_raw / (float)(1 << 21)) * 360.0f));
  // printf("angle: %s\r\n", buffer);
  
  return HAL_OK;
}

/**
 * @brief  CRC校验：X8+X2+X+1
 * @param  data  数据指针
 * @param  len   数据长度
 * @return CRC校验值
 */
uint8_t MT6835::crc_table(const uint8_t *data, uint8_t len) {
  if (data == NULL) {
      return 0; // 处理空指针情况
  }
  uint8_t crc = 0x00; // 初始CRC值

  for (size_t i = 0; i < len; i++) {
      crc ^= data[i]; // 与数据异或
      crc = crc8_table[crc]; // 查表更新CRC
  }

  return crc;
}

float MT6835::getAngle(void)
{

  return (_rawAngle / (float)(1 << 21)) * 360.0f;
}

float MT6835::getRadian(void)
{
  return (_rawAngle / (float)(1 << 21)) * 2.0f * PI;
}

float MT6835::getVelocity(void)
{
  return _velocity;
}

void MT6835::update(uint32_t frequency) 
{
  static float lastAngle = 0;
  float nowAngle = 0;
  float err = 0;
  readRawAngle();
  nowAngle = getAngle();
  err = nowAngle - lastAngle;

  if (abs(err) > 180)
  {
    err = err < 0 ? 360 - lastAngle + nowAngle : nowAngle - 360 - lastAngle;
  }
  
 _velocity =  (err * frequency) / 6;
 lastAngle = nowAngle;

}


