#ifndef _main
#define _main

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

//LOG - /components/ibraries/log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"

#include "DS1307.h"  
#include "HDC1080.h"
#include "SSD1306.h"

#define TWI_INSTANCE_ID     0
#define TWI_ADDRESSES      127

#define DS1307_ADDR        (0x68)

#define DS1307_SEC (0x00)
#define DS1307_MIN (0x01)
#define DS1307_HRS (0x02)
#define DS1307_DAY (0x03)
#define DS1307_DATE (0x04)
#define DS1307_MONTH (0x05)
#define DS1307_YEAR (0x06)


static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


#endif