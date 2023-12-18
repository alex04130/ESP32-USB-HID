#ifndef __SK_BQ4050_HID__
#define __SK_BQ4050_HID__
#include "bq4050.h"
#include "esp32ups_hid.h"

#define Battery_RTE_Limit 5
#define Battery_Shutdown_Limit 2
uint16_t bq_BattState_u16(bool ACIN, bool NVDC_charge);
#endif