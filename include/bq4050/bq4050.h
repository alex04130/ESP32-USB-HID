#ifndef __BQ4050_H__
#define __BQ4050_H__

#include <bq4050.h>
#include "driver/i2c.h"
#include "esp_log.h"

// BQ4050配置
#define BQ4050_ADDR (0x16 >> 0)

namespace BQ4050
{
    static i2c_port_t I2C_MASTER;
    static int I2C_MASTER_TIMEOUT;
    static const char *BQ4050_TAG = "BQ4050";
}

void bq_Init();
uint8_t bq_BattState();                  // Return CHG/DSG(0xF?/0x0?), OK/Bad(0x?0/0x?F)
uint16_t bq_GetAdvState();               // Return XDSG/XCHG/PF/SS/FC/FD/TAPR/VCT/CB/SMTH/SU/IN/VDQ/FCCX/EDV2/EDV1
uint16_t bq_GetVoltage();                // Unit: mV
int16_t bq_GetCurrent();                 // Unit: mA
uint8_t bq_GetRSOC();                    // Unit: %
uint16_t bq_GetT2E();                    // Unit: min
uint16_t bq_GetT2F();                    // Unit: min
uint16_t bq_GetPackTemp();               // Unit: 0.1K
uint8_t bq_GetMaxErr();                  // Unit: %
uint8_t bq_GetHealth();                  // Unit: %
uint16_t bq_GetCellVolt(uint8_t cellNo); // Unit: mV
uint16_t bq_GetCycleCnt();
esp_err_t bq4050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t bq4050_register_write(uint8_t *data, size_t len);
void ESP_I2C_ERROR_CHECK(esp_err_t errcode);

void bq_GetLifetimeBlock(uint8_t blockNo, uint8_t *blockBuf);
void bq_MACReadBlock(uint8_t *inBlock, uint8_t inLen, uint8_t *outBlock, uint8_t outLen);

#endif