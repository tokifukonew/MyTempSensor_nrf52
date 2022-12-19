#include "main.h"


void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN_NUMBER,
       .sda                = SDA_PIN_NUMBER,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
}

void twi_scanner (void)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    NRF_LOG_INFO("TWI scanner started.");
    NRF_LOG_FLUSH();
    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }
}


int main(void)
{
    ret_code_t err_code;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    nrf_gpio_cfg_output(LED_1);

    twi_init();
    twi_scanner();


    DS1307_Init(&m_twi);
    /*
    DS1307_SetTimeZone(+3, 00);
    DS1307_SetDate(19);
    DS1307_SetMonth(12);
    DS1307_SetYear(2022);
    DS1307_SetDayOfWeek(2);
    DS1307_SetHour(12);
    DS1307_SetMinute(51);
    DS1307_SetSecond(00);
    */
    volatile float temp;
    volatile uint8_t humi;

    hdc1080_init(&m_twi, Temperature_Resolution_14_bit,Humidity_Resolution_14_bit);
    
    uint8_t date = DS1307_GetDate();
    uint8_t month = DS1307_GetMonth();
    uint16_t year = DS1307_GetYear();
    //uint8_t dow = DS1307_GetDayOfWeek();
    uint8_t hour = DS1307_GetHour();
    uint8_t minute = DS1307_GetMinute();
    uint8_t second = DS1307_GetSecond();
    //int8_t zone_hr = DS1307_GetTimeZoneHour();
    //uint8_t zone_min = DS1307_GetTimeZoneMin();
    
    while (true)
    {
        NRF_LOG_INFO("LED toggle!");
        NRF_LOG_FLUSH();
        nrf_gpio_pin_toggle(LED_1);


	date = DS1307_GetDate();
	month = DS1307_GetMonth();
	year = DS1307_GetYear();
	//dow = DS1307_GetDayOfWeek();
	hour = DS1307_GetHour();
	minute = DS1307_GetMinute();
	second = DS1307_GetSecond();
	//zone_hr = DS1307_GetTimeZoneHour();
	//zone_min = DS1307_GetTimeZoneMin();
        NRF_LOG_INFO("%04d-%02d-%02d %02d:%02d:%02d%", year, month, date, hour, minute, second);
        NRF_LOG_FLUSH();

        hdc1080_start_measurement((float*)&temp,(uint8_t*)&humi);
        NRF_LOG_INFO("Temp " NRF_LOG_FLOAT_MARKER "*C Humidity %d%%\r\n", NRF_LOG_FLOAT(temp), humi);
        NRF_LOG_FLUSH();

        nrf_delay_ms(500);
    }
}
