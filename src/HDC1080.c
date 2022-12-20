/*
* Based on XuanThiep/HDC1080-With-STM32
*/

#include "hdc1080.h"
#include "main.h"

nrf_drv_twi_t *_hdc1080_m_twi;

// static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
// {
//   uint8_t *i2c_address = handle;
//   ret_code_t err_code;
//   uint8_t buffer[1 + len];
//   memcpy(buffer, &reg, 1);
//   memcpy(buffer + 1, bufp, len);
//   err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, buffer, len + 1, true);
//   if (err_code == NRF_SUCCESS)
//   {
//     NRF_LOG_INFO("Device Address and Register Address and Data sent");
//   }
//   NRF_LOG_FLUSH();
//   return 0;
// }

/**
 * @brief Initializes the HDC1080. Sets clock halt bit to 0 to start timing.
 * @param m_twi User TWI handle pointer.
 * @param Temp_Reso Temperature_Resolution in bits
 * @param Humi_Reso Humidity_Resolution in bits
 */
void hdc1080_init(nrf_drv_twi_t *m_twi, Temp_Reso Temperature_Resolution_x_bit, Humi_Reso Humidity_Resolution_x_bit)
{
  /* Temperature and Humidity are acquired in sequence, Temperature first
   * Default:   Temperature resolution = 14 bit,
   *            Humidity resolution = 14 bit
   */
  /* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
  _hdc1080_m_twi = m_twi;
  ret_code_t err_code;
  uint16_t config_reg_value = 0x1000;
  uint8_t data_send[2];

  if (Temperature_Resolution_x_bit == Temperature_Resolution_11_bit)
  {
    config_reg_value |= (1 << 10); // 11 bit
  }

  switch (Humidity_Resolution_x_bit)
  {
  case Humidity_Resolution_11_bit:
    config_reg_value |= (1 << 8);
    break;
  case Humidity_Resolution_8_bit:
    config_reg_value |= (1 << 9);
    break;
  }

  data_send[0] = (config_reg_value >> 8);
  data_send[1] = (config_reg_value & 0x00ff);
  uint8_t buffer[1 + 2];
  memset(buffer, Configuration_register_add, 1);
  memcpy(buffer + 1, data_send, 2);
  err_code = nrf_drv_twi_tx(_hdc1080_m_twi, HDC_1080_ADD, buffer, 2 + 1, true);
  if (err_code == NRF_SUCCESS)
  {
    //NRF_LOG_INFO("Device Address and Register Address and Data sent");
  }
  NRF_LOG_FLUSH();
  nrf_delay_ms(1);
}

/**
 * @brief Measure temperature and humudity. 15ms delay in function
 * @param float Temperature
 * @param uint8_t Humidity
 * @return 0
 */
uint8_t hdc1080_start_measurement(float *temperature, uint8_t *humidity)
{
  uint8_t receive_data[4];
  uint16_t temp_x, humi_x;
  uint8_t send_data = Temperature_register_add;
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(_hdc1080_m_twi, HDC_1080_ADD, &send_data, sizeof(send_data), true);
  nrf_delay_ms(15);
  if (NRF_SUCCESS == err_code)
  {
    err_code = nrf_drv_twi_rx(_hdc1080_m_twi, HDC_1080_ADD, &receive_data, sizeof(receive_data));
  }

  temp_x = ((receive_data[0] << 8) | receive_data[1]);
  humi_x = ((receive_data[2] << 8) | receive_data[3]);

  *temperature = ((temp_x / 65536.0) * 165.0) - 40.0;
  *humidity = (uint8_t)((humi_x / 65536.0) * 100.0);

  return 0;
}