#define __DEBUG_BQ4050__

// I2C设置
#define I2C_MASTER_SCL_IO 38        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 39        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#include <cstdlib>
#include <iostream>
#include <string>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "esp32ups_hid.h"
#ifndef __DEBUG_BQ4050__
#include "bq4050.h"
#include "SK_BQ4050_HID.h"
#endif
#include "gpio_cxx.hpp"
#include <thread>

using namespace std;
using namespace idf;

#ifdef __DEBUG_BQ4050__

#define BQ4050_ADDR 0x0B
#define __SK_BQ4050_HID__
#define Battery_RTE_Limit 5
#define Battery_Shutdown_Limit 2

static esp_err_t Check_I2C_Error(esp_err_t err)
{
    switch (err)
    {
    case ESP_OK: 
        break;
    case ESP_ERR_INVALID_ARG: 
        ESP_LOGE(TAG, "I2C parameter error");
        break;
    case ESP_FAIL:
        ESP_LOGE(TAG, "I2C no slave ACK");
        break;
    case ESP_ERR_INVALID_STATE: 
        ESP_LOGE(TAG, "I2C driver not installed or not master");
        break;
    case ESP_ERR_TIMEOUT:  
        ESP_LOGE(TAG, "I2C timeout");
        break;
    default:
        ESP_LOGE(TAG, "I2C error %d", err);
    }
    return err;
}

esp_err_t bq4050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BQ4050_ADDR << 1 , ACK_CHECK);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BQ4050_ADDR << 1 | 1, ACK_CHECK);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, ACK_VALUE);
    }
    i2c_master_read_byte(cmd, &data[len - 1], NACK_VALUE);
    i2c_master_stop(cmd);
    err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
    i2c_cmd_link_delete(cmd);
    return err;
}

uint16_t bq_GetVoltage()
{ // Unit: mV
    uint8_t battBuf[2];
    uint16_t battVolt;
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x09), battBuf, 2));
    battVolt = (battBuf[1] << 8) + battBuf[0];
    return battVolt;
}

uint8_t bq_GetRSOC()
{ // Unit: %
    uint8_t battBuf[2];
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x0D), battBuf, 2));
    return battBuf[0];
}

uint16_t bq_GetT2E()
{ // Unit: min
    uint8_t battBuf[2];
    uint16_t battT2E;
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x12), battBuf, 2));
    battT2E = (battBuf[1] << 8) + battBuf[0];
    return battT2E;
}

uint16_t bq_GetT2F()
{ // Unit: min
    uint8_t battBuf[2];
    uint16_t battT2F;
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x13), battBuf, 2));
    battT2F = (battBuf[1] << 8) + battBuf[0];
    return battT2F;
}

uint16_t bq_BattState_u16()
{
    uint16_t ret = 0, battStatus_total = 0;
    uint8_t battStatus[2];
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x16), battStatus, 2));
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
    ESP_ERROR_CHECK(bq4050_register_read(uint8_t(0x12), battStatus, 2));
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

#endif

#define SetBitTrue(a, b) a |= (1 << b)
#define SetBitFalse(a, b) a &= ~(1 << b)
#define GetBit(a, b) a & (1 << b)

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define ITF_NUM_TOTAL 1

#define __USB_HID_ENABLE__

#ifdef __SK_BQ4050_HID__
// 固定参数
uint16_t BatteryVoltage = 0;
const unsigned char BatteryDeviceChemistry = IDEVICECHEMISTRY;
const unsigned char BatteryOEMVendor = IOEMVENDOR;
uint16_t ManufactureDate = 100;
const unsigned char BatteryDesignCapacity = 100;
const unsigned char BatteryFullChargeCapacity = 100;
unsigned char BatteryWarnCapacityLimit = 10;
unsigned char BatteryRemnCapacityLimit = 5;
unsigned char BatteryCapacityGranularity1 = 1;
unsigned char BatteryCapacityGranularity2 = 1;
// 实时参数
uint16_t BatteryCurrentStatus = 0;
uint8_t BatteryCurrentCapacity = 0;
uint8_t BatteryRunTimeToEmpty = 0;
uint8_t BatteryRunTimeToFull = 0;
// 上一个参数
uint16_t BatteryPrevStatus = 0;
uint8_t BatteryPrevCapacity = 0;
#else
// 固定参数
uint16_t BatteryVoltage = 7000;
const unsigned char BatteryDeviceChemistry = IDEVICECHEMISTRY;
const unsigned char BatteryOEMVendor = IOEMVENDOR;
uint16_t ManufactureDate = 100;
const unsigned char BatteryDesignCapacity = 100;
const unsigned char BatteryFullChargeCapacity = 100;
unsigned char BatteryWarnCapacityLimit = 10;
unsigned char BatteryRemnCapacityLimit = 5;
unsigned char BatteryCapacityGranularity1 = 1;
unsigned char BatteryCapacityGranularity2 = 1;
// 实时参数
uint16_t BatteryCurrentStatus = 0;
uint8_t BatteryCurrentCapacity = 70;
uint8_t BatteryRunTimeToEmpty = 1000;
uint8_t BatteryRunTimeToFull = 1000;
// 上一个参数
uint16_t BatteryPrevStatus = 0;
uint8_t BatteryPrevCapacity = 70;
#endif

static const char *TAG = "SKele-DC";

unsigned char BatteryRechargable = 1;
unsigned char BatteryCapacityMode = 2; // units are in %%

#ifdef __USB_HID_ENABLE__

tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = 0x01, //    0x01
    .bcdUSB = 0x0110,        //    USB1.1

    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, //    64

    .idVendor = 0xb238,
    .idProduct = 0x0180,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01};

const char *string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x08, 0x04},     // 0: 支持语言：中文 (0x0809)
    "深空电子",               // 1: 制造商
    "深空电子HID通用UPS项目", // 2: 产品
    "1",                      // 3: 串行、芯片ID
    "深空电子HID",            // 4: HID
    "深空电子HID接口",        // 5: 配置描述符
};

uint8_t const desc_configuration[] = {
    //    配置描述符
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0x04, TUSB_DESC_TOTAL_LEN, 0, 100),

    //    接口描述符、HID描述符、端点描述符
    TUD_HID_INOUT_DESCRIPTOR(1, 0x04, 0, sizeof(ESP32UPS::desc_hid_report), 0x01, 0x81, 64, 10)};

#endif

#ifdef __SK_BQ4050_HID__
static esp_err_t i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SCL_IO,
        .scl_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{
            .clk_speed = I2C_MASTER_FREQ_HZ},
    };
#ifndef __DEBUG_BQ4050__
    BQ4050::I2C_MASTER = I2C_MASTER_NUM;
    BQ4050::I2C_MASTER_TIMEOUT = I2C_MASTER_TIMEOUT_MS;
#endif
    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
#else
uint16_t BattState_u16(bool charging)
{
    uint16_t ret = 0;
    if (charging)
    {
        ret |= 1 << PRESENTSTATUS_ACPRESENT;
        ret |= 1 << PRESENTSTATUS_CHARGING;
        ret |= ((BatteryCurrentCapacity == 100) ? 1 << PRESENTSTATUS_FULLCHARGE : 0x00);
    }
    else
    {
        ret |= 1 << PRESENTSTATUS_DISCHARGING;
        ret |= ((BatteryCurrentCapacity == 0) ? 1 << PRESENTSTATUS_FULLDISCHARGE : 0x00);
    }
    if (BatteryCurrentCapacity < 5)
    {
        ret |= 1 << PRESENTSTATUS_SHUTDOWNREQ;
    }
    if (BatteryCurrentCapacity <= 2)
    {
        ret |= 1 << PRESENTSTATUS_SHUTDOWNIMNT;
    }
    ret |= 1 << PRESENTSTATUS_BATTPRESENT;
    return ret;
}
#endif

extern "C" void app_main()
{
#ifdef __USB_HID_ENABLE__
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_descriptor,
        .string_descriptor_count = sizeof(string_descriptor) / sizeof(string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
        .self_powered = false,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
#endif
#ifdef __SK_BQ4050_HID__
    const GPIO_Output ChargingLED(GPIONum(37));
    const GPIO_Output PowerlossLED(GPIONum(36));
    ESP_ERROR_CHECK(i2c_master_init());
    BatteryVoltage = bq_GetVoltage();
    BatteryCurrentCapacity = bq_GetRSOC();
    BatteryRunTimeToEmpty = bq_GetT2E();
    BatteryRunTimeToFull = bq_GetT2F();
#else
    const GPIOInput persentup(GPIONum(40));
    const GPIOInput persentdown(GPIONum(41));
    const GPIOInput charging(GPIONum(42));
#endif
    ESP_LOGI(TAG, "initialization DONE!\n:)");
    while (1)
    {
#ifdef __SK_BQ4050_HID__
        BatteryCurrentCapacity = bq_GetRSOC();
        BatteryRunTimeToEmpty = bq_GetT2E();
        BatteryCurrentStatus = bq_BattState_u16();
#ifdef __USB_HID_ENABLE__
        if ((BatteryCurrentCapacity != BatteryPrevCapacity) || (BatteryCurrentStatus != BatteryPrevStatus))
        {
            tud_hid_report(HID_PD_REMAININGCAPACITY, &BatteryCurrentCapacity, sizeof(BatteryCurrentCapacity));
            tud_hid_report(HID_PD_PRESENTSTATUS, &BatteryCurrentStatus, sizeof(BatteryCurrentStatus));
            if (BatteryCurrentStatus & (1 << PRESENTSTATUS_DISCHARGING))
                tud_hid_report(HID_PD_RUNTIMETOEMPTY, &BatteryRunTimeToEmpty, sizeof(BatteryRunTimeToEmpty));
            BatteryPrevCapacity = BatteryCurrentCapacity;
            BatteryPrevStatus = BatteryCurrentStatus;
        }
#endif
        if (BatteryCurrentCapacity < 10)
            PowerlossLED.set_high();
        else
            PowerlossLED.set_low();
        if (BatteryCurrentStatus & (1 << PRESENTSTATUS_DISCHARGING))
            ChargingLED.set_high();
        else
            ChargingLED.set_low();
        ESP_LOGI(TAG, "Battery Current Capacity :%d\nBattery Current Status :%d\nBattery RunTime To Empty :%d\n", BatteryCurrentCapacity, BatteryCurrentStatus, BatteryRunTimeToEmpty);
#else
        if (persentup.get_level() == GPIOLevel::HIGH)
        {
            BatteryCurrentCapacity++;
        }
        if (persentdown.get_level() == GPIOLevel::HIGH)
        {
            BatteryCurrentCapacity--;
        }
        if (charging.get_level() == GPIOLevel::HIGH)
        {
            BatteryCurrentStatus = BattState_u16(true);
        }
        else
        {
            BatteryCurrentStatus = BattState_u16(false);
        }
        if ((BatteryCurrentCapacity != BatteryPrevCapacity) || (BatteryCurrentStatus != BatteryPrevStatus))
        {
            tud_hid_report(HID_PD_REMAININGCAPACITY, &BatteryCurrentCapacity, sizeof(BatteryCurrentCapacity));
            tud_hid_report(HID_PD_PRESENTSTATUS, &BatteryCurrentStatus, sizeof(BatteryCurrentStatus));
            if (BatteryCurrentStatus & (1 << PRESENTSTATUS_DISCHARGING))
                tud_hid_report(HID_PD_RUNTIMETOEMPTY, &BatteryRunTimeToEmpty, sizeof(BatteryRunTimeToEmpty));
            BatteryPrevCapacity = BatteryCurrentCapacity;
            BatteryPrevStatus = BatteryCurrentStatus;
        }
        ESP_LOGI(TAG, "persentup.get_level() :%d\npersentdown.get_level() :%d\ncharging.get_level() :%d\n", int(persentup.get_level()), int(persentdown.get_level()), int(charging.get_level()));
        ESP_LOGI(TAG, "Battery Current Capacity :%d\nBattery Current Status :%d\n", BatteryCurrentCapacity, BatteryCurrentStatus);
#endif
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

#ifdef __USB_HID_ENABLE__

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    switch (int(report_type))
    {
    case HID_REPORT_TYPE_FEATURE:
    {
        switch (report_id)
        {
        case HID_PD_PRESENTSTATUS:
        {
#ifdef __SK_BQ4050_HID__
            BatteryCurrentStatus = bq_BattState_u16();
            buffer[0] = BatteryCurrentStatus & 0x00ff;
            buffer[1] = BatteryCurrentStatus >> 8 & 0x00ff;
            return 2;
#else
            buffer[0] = BatteryCurrentStatus & 0x00ff;
            buffer[1] = BatteryCurrentStatus >> 8 & 0x00ff;
            return 2;
#endif
        }
        case HID_PD_VOLTAGE:
        {
#ifdef __SK_BQ4050_HID__
            BatteryVoltage = bq_GetVoltage();
            buffer[0] = BatteryVoltage & 0x00ff;
            buffer[1] = BatteryVoltage >> 8 & 0x00ff;
            return 2;
#else
            buffer[0] = BatteryVoltage & 0x00ff;
            buffer[1] = BatteryVoltage >> 8 & 0x00ff;
            return 2;
#endif
        }
        case HID_PD_DESIGNCAPACITY:
        {
            buffer[0] = BatteryDesignCapacity;
            return 1;
        }
        case HID_PD_IDEVICECHEMISTRY:
        {
            buffer[0] = BatteryDeviceChemistry;
            return 1;
        }
        case HID_PD_SERIAL:
        {
            buffer[0] = 3;
            return 1;
        }
        case HID_PD_IOEMINFORMATION:
        {
            buffer[0] = BatteryOEMVendor;
            return 1;
        }
        case HID_PD_IPRODUCT:
        {
            buffer[0] = IPRODUCT;
            return 1;
        }
        case HID_PD_MANUFACTURER:
        {
            buffer[0] = IMANUFACTURER;
            return 1;
        }
        case HID_PD_MANUFACTUREDATE:
        {
            buffer[0] = ManufactureDate & 0x00ff;
            buffer[1] = ManufactureDate >> 8 & 0x00ff;
            return 2;
        }
        case HID_PD_FULLCHRGECAPACITY:
        {
            buffer[0] = BatteryFullChargeCapacity;
            return 1;
        }
        case HID_PD_WARNCAPACITYLIMIT:
        {
            buffer[0] = BatteryWarnCapacityLimit;
            return 1;
        }
        case HID_PD_REMNCAPACITYLIMIT:
        {
            buffer[0] = BatteryRemnCapacityLimit;
            return 1;
        }
        case HID_PD_REMAININGCAPACITY:
        {
            ESP_LOGI(TAG, "RESVIED REQUEST ON iRemaining\n");
#ifdef __SK_BQ4050_HID__

            BatteryCurrentCapacity = bq_GetRSOC();
            buffer[0] = BatteryCurrentCapacity;
            return 1;
#else
            buffer[0] = BatteryCurrentCapacity;
            return 1;
#endif
        }
        case HID_PD_RUNTIMETOEMPTY:
        {
#ifdef __SK_BQ4050_HID__

            BatteryRunTimeToEmpty = bq_GetT2E();
            buffer[0] = BatteryRunTimeToEmpty & 0x00ff;
            buffer[1] = BatteryRunTimeToEmpty >> 8 & 0x00ff;
            return 2;
#else
            buffer[0] = BatteryRunTimeToEmpty & 0x00ff;
            buffer[1] = BatteryRunTimeToEmpty >> 8 & 0x00ff;
            return 2;
#endif
        }
        case HID_PD_CPCTYGRANULARITY1:
        {
            buffer[0] = BatteryCapacityGranularity1;
            return 1;
        }
        case HID_PD_CPCTYGRANULARITY2:
        {
            buffer[0] = BatteryCapacityGranularity2;
            return 1;
        }
        case HID_PD_CAPACITYMODE:
        {
            buffer[0] = BatteryCapacityMode;
            return 1;
        }
        }
    }
    }
    ESP_LOGI(TAG, "Get_report Request: %d, type=%s , id: %d , len:%d\n",
             instance,
             (report_type == HID_REPORT_TYPE_INVALID ? "HID_REPORT_TYPE_INVALID" : (report_type == HID_REPORT_TYPE_INPUT ? "HID_REPORT_TYPE_INPUT" : (report_type == HID_REPORT_TYPE_OUTPUT ? "HID_REPORT_TYPE_OUTPUT" : "HID_REPORT_TYPE_FEATURE"))),
             report_id,
             reqlen);
    for (int i = 0; i < reqlen; i++)
    {
        ESP_LOGI(TAG, "Report %d:%d\n", i, buffer[-i]);
    }
    ESP_LOGI(TAG, "end REQUSET\n");
    return 0;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return ESP32UPS::desc_hid_report;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    ESP_LOGI(TAG, "Get_report Request: %d, type=%s , id: %d , len:%d\n",
             instance,
             (report_type == HID_REPORT_TYPE_INVALID ? "HID_REPORT_TYPE_INVALID" : (report_type == HID_REPORT_TYPE_INPUT ? "HID_REPORT_TYPE_INPUT" : (report_type == HID_REPORT_TYPE_OUTPUT ? "HID_REPORT_TYPE_OUTPUT" : "HID_REPORT_TYPE_FEATURE"))),
             report_id,
             bufsize);
    for (int i = 0; i < bufsize; i++)
    {
        ESP_LOGI(TAG, "Report %d:%d\n", i, buffer[i]);
    }
    ESP_LOGI(TAG, "end REQUSET\n");
}

#endif