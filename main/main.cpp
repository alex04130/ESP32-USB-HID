
// I2C设置
#define I2C_MASTER_SCL_IO 38        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 39        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
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
#include "bq4050.h"
#include "esp32ups_hid.h"
#include "SK_BQ4050_HID.h"
#include "gpio_cxx.hpp"
#include <thread>

using namespace std;
using namespace idf;

#define SetBitTrue(a, b) a |= (1 << b)
#define SetBitFalse(a, b) a &= ~(1 << b)
#define GetBit(a, b) a & (1 << b)

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define ITF_NUM_TOTAL 1

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
// Physical parameters
const uint16_t iConfigVoltage = 1380;
uint16_t iVoltage = 1300, iPrevVoltage = 0;
uint16_t iRunTimeToEmpty = 0, iPrevRunTimeToEmpty = 0;
uint16_t iAvgTimeToFull = 7200;
uint16_t iAvgTimeToEmpty = 7200;
uint16_t iRemainTimeLimit = 600;
int16_t iDelayBe4Reboot = -1;
int16_t iDelayBe4ShutDown = -1;
uint16_t IMANUFACTURERDATE = 100; // Parameters for ACPI compliancy
const unsigned char iDesignCapacity = 100;
unsigned char iWarnCapacityLimit = 10; // warning at 10%
unsigned char iRemnCapacityLimit = 5;  // low at 5%
const unsigned char bCapacityGranularity1 = 1;
const unsigned char bCapacityGranularity2 = 1;
unsigned char iFullChargeCapacity = 100;

unsigned char iRemaining = 70, iPrevRemaining = 0;

int iRes = 0;

const unsigned char bDeviceChemistry = IDEVICECHEMISTRY;
const unsigned char bOEMVendor = IOEMVENDOR;
uint16_t iPresentStatus = 0, iPreviousStatus = 0;
#endif

static const char *TAG = "SKele-DC";

unsigned char BatteryRechargable = 1;
unsigned char BatteryCapacityMode = 2; // units are in %%

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

    BQ4050::I2C_MASTER = I2C_MASTER_NUM;
    BQ4050::I2C_MASTER_TIMEOUT = I2C_MASTER_TIMEOUT_MS;

    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

extern "C" void app_main()
{
    const GPIO_Output ChargingLED(GPIONum(37));
    const GPIO_Output PowerlossLED(GPIONum(36));
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
    ESP_LOGI(TAG, "initialization DONE!\n:)");
#ifdef __SK_BQ4050_HID__
    ESP_ERROR_CHECK(i2c_master_init());
    BatteryVoltage = bq_GetVoltage();
    BatteryCurrentCapacity = bq_GetRSOC();
    BatteryRunTimeToEmpty = bq_GetT2E();
    BatteryRunTimeToFull = bq_GetT2F();
#else
#endif

    while (1)
    {
#ifdef __SK_BQ4050_HID__
        BatteryCurrentCapacity = bq_GetRSOC();
        BatteryRunTimeToEmpty = bq_GetT2E();
        BatteryCurrentStatus = bq_BattState_u16();
        if ((BatteryCurrentCapacity != BatteryPrevCapacity) || (BatteryCurrentStatus != BatteryPrevStatus))
        {
            tud_hid_report(HID_PD_REMAININGCAPACITY, &BatteryCurrentCapacity, sizeof(BatteryCurrentCapacity));
            tud_hid_report(HID_PD_PRESENTSTATUS, &BatteryCurrentStatus, sizeof(BatteryCurrentStatus));
            if (BatteryCurrentStatus & (1 << PRESENTSTATUS_DISCHARGING))
                tud_hid_report(HID_PD_RUNTIMETOEMPTY, &BatteryRunTimeToEmpty, sizeof(BatteryRunTimeToEmpty));
            BatteryCurrentCapacity = BatteryPrevCapacity;
            BatteryCurrentStatus = BatteryPrevStatus;
        }
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
#endif
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

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
