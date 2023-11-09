
#include <bq4050.h>
#include "driver/i2c.h"

esp_err_t bq4050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BQ4050::I2C_MASTER, BQ4050_ADDR, &reg_addr, 1, data, len, BQ4050::I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
}
esp_err_t bq4050_register_write(uint8_t *data, size_t len)
{
    return i2c_master_write_to_device(BQ4050::I2C_MASTER, BQ4050_ADDR, data, len, BQ4050::I2C_MASTER_TIMEOUT / portTICK_PERIOD_MS);
}

// inblock前面必须加0x44地址
// inLen的长度不包含0x44地址标识符
void bq_MACReadBlock(uint8_t *inBlock, uint8_t inLen, uint8_t *outBlock, uint8_t outLen)
{
    uint8_t retryCnt = 100;
    while (1)
    {
        bq4050_register_write(inBlock, inLen + 1);
        bq4050_register_read(0x44, outBlock, outLen);
        if (inBlock[2] == outBlock[1] && inBlock[3] == outBlock[2])
        {
            break;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

uint8_t bq_BattState() // Return CHG/DSG(0xC?/0x0?), OK/Bad(0x0?/0x3?), TCTDFCFD(bit3 2 1 0)
{
    uint8_t battStatus[2], battDataBuf[7], ret = 0x00;
    uint8_t battCmd[4] = {0x44, 0x02, 0x50, 0x00}; // SafetyAlert MAC Cmd
    ESP_ERROR_CHECK(bq4050_register_read(0x16, battStatus, 2));
    ret |= (((battStatus[0] & 0x40) == 0x40) ? 0x00 : 0xc0);
    bq_MACReadBlock(battCmd, 3, battDataBuf, 7);
    battDataBuf[6] &= 0x0f; // Clear RSVD
    battDataBuf[5] &= 0xfd;
    battDataBuf[4] &= 0x7a;
    battDataBuf[3] &= 0xbf;
    if ((battDataBuf[3] | battDataBuf[4] | battDataBuf[5] | battDataBuf[6]) != 0)
    {
        ret |= 0x30;
        return ret;
    }
    battCmd[2] = 0x54;
    bq_MACReadBlock(battCmd, 3, battDataBuf, 7);
    if ((battDataBuf[4] & 0x60) != 0)
    {
        ret |= 0x30;
    }
    battCmd[2] = 0x51;
    bq_MACReadBlock(battCmd, 3, battDataBuf, 7);
    battDataBuf[6] &= 0x0f; // Clear RSVD
    battDataBuf[5] &= 0xd5;
    battDataBuf[4] &= 0x7f;
    battDataBuf[3] &= 0xff;
    if ((battDataBuf[3] | battDataBuf[4] | battDataBuf[5] | battDataBuf[6]) != 0)
    {
        ret |= 0x30;
    }
    battCmd[2] = 0x56;
    bq_MACReadBlock(battCmd, 3, battDataBuf, 5);
    ret |= (battDataBuf[3] & 0x0f);
    return ret;
}

uint16_t bq_GetAdvState()
{ // Return XDSG/XCHG/PF/SS  FC/FD/TAPR/VCT  CB/SMTH/SU/IN  VDQ/FCCX/EDV2/EDV1
    uint16_t ret = 0x0000;
    uint8_t battDataBuf[7], battCmd[4] = {0x44, 0x02, 0x54, 0x00}; // OperationStatus MAC Cmd
    bq_MACReadBlock(battCmd, 3, battDataBuf, 7);
    ret |= ((battDataBuf[3] & 0x40) == 0x40) ? (uint16_t)1 << 6 : 0;  // SMTH
    ret |= ((battDataBuf[4] & 0x08) == 0x08) ? (uint16_t)1 << 12 : 0; // SS
    ret |= ((battDataBuf[4] & 0x10) == 0x10) ? (uint16_t)1 << 13 : 0; // PF
    ret |= ((battDataBuf[4] & 0x20) == 0x20) ? (uint16_t)1 << 15 : 0; // XDSG
    ret |= ((battDataBuf[4] & 0x40) == 0x40) ? (uint16_t)1 << 14 : 0; // XCHG
    ret |= ((battDataBuf[6] & 0x10) == 0x10) ? (uint16_t)1 << 7 : 0;  // CB
    battCmd[2] = 0x55;                                                // ChargingStatus MAC Cmd
    bq_MACReadBlock(battCmd, 3, battDataBuf, 6);
    ret |= ((battDataBuf[3] & 0x10) == 0x10) ? (uint16_t)1 << 4 : 0; // IN
    ret |= ((battDataBuf[3] & 0x20) == 0x20) ? (uint16_t)1 << 5 : 0; // SU
    ret |= ((battDataBuf[3] & 0x80) == 0x80) ? (uint16_t)1 << 8 : 0; // VCT
    ret |= ((battDataBuf[4] & 0x80) == 0x80) ? (uint16_t)1 << 9 : 0; // TAPR
    battCmd[2] = 0x56;                                               // GaugingStatus MAC Cmd
    bq_MACReadBlock(battCmd, 3, battDataBuf, 5);
    ret |= ((battDataBuf[3] & 0x01) == 0x01) ? (uint16_t)1 << 10 : 0; // FD
    ret |= ((battDataBuf[3] & 0x02) == 0x02) ? (uint16_t)1 << 11 : 0; // FC
    ret |= ((battDataBuf[4] & 0x04) == 0x04) ? (uint16_t)1 << 2 : 0;  // FCCX
    ret |= ((battDataBuf[4] & 0x20) == 0x20) ? (uint16_t)1 << 0 : 0;  // EDV1
    ret |= ((battDataBuf[4] & 0x40) == 0x40) ? (uint16_t)1 << 1 : 0;  // EDV2
    ret |= ((battDataBuf[4] & 0x80) == 0x80) ? (uint16_t)1 << 3 : 0;  // VDQ

    return ret;
}
void bq_GetLifetimeBlock(uint8_t blockNo, uint8_t *blockBuf)
{
    uint8_t blockBufLen = 3, battCmd[4] = {0x44, 0x02, 0x60, 0x00}; // LifetimeDataBlockN MAC Cmd
    battCmd[2] += (blockNo - 1);
    switch (blockNo)
    {
    case 1:
    case 4:
        blockBufLen += 32;
        break;
    case 2:
        blockBufLen += 8;
        break;
    case 3:
        blockBufLen += 16;
        break;
    case 5:
        blockBufLen += 20;
        break;
    default:
        return;
    }
    bq_MACReadBlock(battCmd, 3, blockBuf, blockBufLen);
}

uint16_t bq_GetVoltage()
{ // Unit: mV
    uint8_t battBuf[2];
    uint16_t battVolt;
    ESP_ERROR_CHECK(bq4050_register_read(0x09, battBuf, 2));
    battVolt = (battBuf[1] << 8) + battBuf[0];
    return battVolt;
}
int16_t bq_GetCurrent()
{ // Unit: mA
    uint8_t battBuf[2];
    int16_t battCurrent;
    ESP_ERROR_CHECK(bq4050_register_read(0x0A, battBuf, 2));
    battCurrent = uint8_t(battBuf[0] << 8) & battBuf[1];
    return battCurrent;
}

uint8_t bq_GetRSOC()
{ // Unit: %
    uint8_t battBuf[2];
    ESP_ERROR_CHECK(bq4050_register_read(0x0D, battBuf, 2));
    return battBuf[0];
}

uint16_t bq_GetT2E()
{ // Unit: min
    uint8_t battBuf[2];
    uint16_t battT2E;
    ESP_ERROR_CHECK(bq4050_register_read(0x12, battBuf, 2));
    battT2E = (battBuf[1] << 8) + battBuf[0];
    return battT2E;
}

uint16_t bq_GetT2F()
{ // Unit: min
    uint8_t battBuf[2];
    uint16_t battT2F;
    ESP_ERROR_CHECK(bq4050_register_read(0x13, battBuf, 2));
    battT2F = (battBuf[1] << 8) + battBuf[0];
    return battT2F;
}

uint16_t bq_GetPackTemp()
{ // Unit: 0.1K
    uint8_t battBuf[2];
    uint16_t battPackTemp;
    ESP_ERROR_CHECK(bq4050_register_read(0x08, battBuf, 2));
    battPackTemp = (battBuf[1] << 8) + battBuf[0];
    return battPackTemp;
}

uint8_t bq_GetMaxErr()
{ // Unit: %
    uint8_t battBuf[2];
    ESP_ERROR_CHECK(bq4050_register_read(0x0C, battBuf, 2));
    return battBuf[0];
}

uint8_t bq_GetHealth()
{ // Unit: %
    uint8_t battBuf[2];
    uint16_t battFCC, battDC;
    float battHealth;
    ESP_ERROR_CHECK(bq4050_register_read(0x10, battBuf, 2));
    battFCC = (battBuf[1] << 8) + battBuf[0];
    ESP_ERROR_CHECK(bq4050_register_read(0x18, battBuf, 2));
    battDC = (battBuf[1] << 8) + battBuf[0];
    battHealth = ((float)battFCC * 100.0f / (float)battDC);
    return (battHealth >= 100.0f ? 100 : (uint8_t)battHealth);
}
uint8_t *bq_GetHealth_Pointer()
{ // Unit: %
    uint8_t battBuf[2];
    uint16_t battFCC, battDC;
    uint8_t battHealth;
    ESP_ERROR_CHECK(bq4050_register_read(0x10, battBuf, 2));
    battFCC = (battBuf[1] << 8) + battBuf[0];
    ESP_ERROR_CHECK(bq4050_register_read(0x18, battBuf, 2));
    battDC = (battBuf[1] << 8) + battBuf[0];
    battHealth = ((float)battFCC * 100.0f / (float)battDC);
    battHealth = battHealth >= 100 ? 100 : battHealth;
    return &battHealth;
}

uint16_t bq_GetCellVolt(uint8_t cellNo)
{ // Unit: mV
    uint8_t battBuf[2], cmd = 0x40;
    uint16_t battCellVolt;
    if (cellNo > 4 || cellNo < 1)
        return 0;
    cmd -= cellNo;
    ESP_ERROR_CHECK(bq4050_register_read(cmd, battBuf, 2));
    battCellVolt = (battBuf[1] << 8) + battBuf[0];
    return battCellVolt;
}

uint16_t bq_GetCycleCnt()
{
    uint8_t battBuf[2];
    uint16_t battCycleCnt;
    ESP_ERROR_CHECK(bq4050_register_read(0x17, battBuf, 2));
    battCycleCnt = (battBuf[1] << 8) + battBuf[0];
    return battCycleCnt;
}
