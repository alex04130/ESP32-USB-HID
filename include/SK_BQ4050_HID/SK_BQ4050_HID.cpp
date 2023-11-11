#include "bq4050.h"
#include "esp32ups_hid.h"
#include "SK_BQ4050_HID.h"

uint16_t bq_BattState_u16()
{
    uint16_t ret = 0, battStatus_total = 0;
    uint8_t battStatus[2];
    ESP_ERROR_CHECK(bq4050_register_read(0x16, battStatus, 2));
    if (battStatus[0] & 0x40)
    {
        ret |= 1 << PRESENTSTATUS_DISCHARGING;
        ret |= ((battStatus[0] & 0x10) ? 1 << PRESENTSTATUS_FULLDISCHARGE : 0x00);
    }
    else
    {
        ret |= 1 << PRESENTSTATUS_ACPRESENT;
        ret |= 1 << PRESENTSTATUS_CHARGING;
        ret |= ((battStatus[0] & 0x20) ? 1 << PRESENTSTATUS_FULLCHARGE : 0x00);
    }
    ESP_ERROR_CHECK(bq4050_register_read(0x12, battStatus, 2));
    battStatus_total = (battStatus[1] << 8) + battStatus[0];
    if (battStatus_total < Battery_RTE_Limit)
    {
        ret |= 1 << PRESENTSTATUS_SHUTDOWNREQ;
    }
    if (battStatus_total <= Battery_Shutdown_Limit)
    {
        ret |= 1 << PRESENTSTATUS_SHUTDOWNIMNT;
    }
    ret |= 1 << PRESENTSTATUS_BATTPRESENT;
    return ret;
}