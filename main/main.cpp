

// I2C设置
#define BQ4050_I2C_MASTER_SCL_IO 38     /*!< GPIO number used for I2C master clock */
#define BQ4050_I2C_MASTER_SDA_IO 39     /*!< GPIO number used for I2C master data  */
#define BQ4050_I2C_MASTER_NUM I2C_NUM_0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define BQ4050_I2C_MASTER_FREQ_HZ 40000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define BQ4050_I2C_MASTER_TIMEOUT_MS 14000
#define SW7203_I2C_MASTER_SCL_IO 45     /*!< GPIO number used for I2C master clock */
#define SW7203_I2C_MASTER_SDA_IO 48     /*!< GPIO number used for I2C master data  */
#define SW7203_I2C_MASTER_NUM I2C_NUM_1 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define SW7203_I2C_MASTER_FREQ_HZ 40000 /*!< I2C master clock frequency */
#define SW7203_I2C_MASTER_TIMEOUT_MS 14000

#define SW7203_ADDR 0x3C

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
#include "bq4050.h"
#include "SK_BQ4050_HID.h"
#include "gpio_cxx.hpp"
#include "driver/gpio.h"
#include <thread>

using namespace std;
using namespace idf;

#define SetBitTrue(a, b) a |= (1 << b)
#define SetBitFalse(a, b) a &= ~(1 << b)
#define GetBit(a, b) a & (1 << b)

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define ITF_NUM_TOTAL 1
#define __USB_HID_ENABLE__

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

bool AC_IN = false;
bool NVDC_BAT_charge = false;

static const char *TAG = "SKele-DC";

unsigned char BatteryRechargable = 1;
unsigned char BatteryCapacityMode = 2; // units are in %%

int err_code = 0;
const GPIO_Output ErrLED(GPIONum(40));
const GPIO_Output ChargingLED(GPIONum(37));
const GPIO_Output PowerlossLED(GPIONum(36));

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
    "Li-on",                  // 4: HID
    "深空电子HID接口",        // 5: 配置描述符
};

uint8_t const desc_configuration[] = {
    //    配置描述符
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0x04, TUSB_DESC_TOTAL_LEN, 0, 100),

    //    接口描述符、HID描述符、端点描述符
    TUD_HID_INOUT_DESCRIPTOR(1, 0x04, 0, sizeof(ESP32UPS::desc_hid_report), 0x01, 0x81, 64, 10)};

#endif

static esp_err_t bq4050_i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BQ4050_I2C_MASTER_SDA_IO,
        .scl_io_num = BQ4050_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{
            .clk_speed = BQ4050_I2C_MASTER_FREQ_HZ},
    };

    BQ4050::I2C_MASTER = BQ4050_I2C_MASTER_NUM;
    BQ4050::I2C_MASTER_TIMEOUT = BQ4050_I2C_MASTER_TIMEOUT_MS;

    i2c_param_config(BQ4050_I2C_MASTER_NUM, &conf);

    return i2c_driver_install(BQ4050_I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static esp_err_t sw7203_i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SW7203_I2C_MASTER_SDA_IO,
        .scl_io_num = SW7203_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{
            .clk_speed = SW7203_I2C_MASTER_FREQ_HZ},
    };

    i2c_param_config(SW7203_I2C_MASTER_NUM, &conf);

    return i2c_driver_install(SW7203_I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t sw7203_register_read(uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(SW7203_I2C_MASTER_NUM, SW7203_ADDR, &reg_addr, 1, data, 1, SW7203_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
esp_err_t sw7203_register_write(uint8_t *data)
{
    return i2c_master_write_to_device(SW7203_I2C_MASTER_NUM, SW7203_ADDR, data, 2, SW7203_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
void sw7203_check_error(esp_err_t errcode)
{
    if (errcode != ESP_OK)
    {
        ESP_LOGI(TAG, "SW7203 I2C error\n:)");
        ESP_ERROR_CHECK_WITHOUT_ABORT(err_code);
        while (1)
        {
            ErrLED.set_high();
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ErrLED.set_low();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}
#ifndef __SW7203_DEBUG__
void sw7203_start_charge()
{
    unsigned int VBUS_Voltage = 0;
    uint8_t data = 0;
    sw7203_check_error(sw7203_register_read(0x11, &data));
    VBUS_Voltage = data;
    VBUS_Voltage = VBUS_Voltage << 4;
    sw7203_check_error(sw7203_register_read(0x12, &data));
    VBUS_Voltage &= (data & 0b00001111);
    VBUS_Voltage *= 75;
    VBUS_Voltage -= 7500;
    VBUS_Voltage = (VBUS_Voltage - 40000) / 1000;
    uint8_t VBUS_BitMask = 0b01111111, VBUS_Limit = VBUS_Voltage;
    uint8_t cmd[3][2] = {{0x38, uint8_t(VBUS_Limit & VBUS_BitMask)}, {0x19, 0b00000100}, {0x0D, 0b00010000}};
    sw7203_register_write(cmd[0]);
    sw7203_register_write(cmd[1]);
    sw7203_register_write(cmd[2]);
}
void sw7203_stop_charge()
{
    uint8_t cmd[2][2] = {{0x04, 0b00000100}, {0x19, 0b00000000}};
    sw7203_register_write(cmd[0]);
    sw7203_register_write(cmd[1]);
}
void IRAM_ATTR sw7203_irq_func(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    ESP_LOGI(TAG, "INT ARG:%d", gpio_num);
    uint8_t data = 0;
    sw7203_check_error(sw7203_register_read(0x04, &data));
    if (data & 0x40)
    {
        ESP_LOGI(TAG, "VSYS voltage limit exceeded");
    }
    if (data & 0x20)
    {
        ESP_LOGI(TAG, "Battery charge time limit exceeded");
    }
    if (data & 0x10)
    {
        ESP_LOGI(TAG, "Battery fullcharged");
    }
    if (data & 0x08)
    {
        ESP_LOGI(TAG, "DCIN moved in");
        AC_IN = true;
        sw7203_start_charge();
    }
    if (data & 0x04)
    {
        ESP_LOGI(TAG, "DCIN moved out");
        AC_IN = false;
        sw7203_stop_charge();
    }
    sw7203_check_error(sw7203_register_read(0x05, &data));
    if (data & 0x80)
    {
        ESP_LOGI(TAG, "SW7203 over temperature");
    }
    if (data & 0x10)
    {
        ESP_LOGI(TAG, "VBAT voltage limit exceeded");
    }
    if (data & 0x08)
    {
        ESP_LOGI(TAG, "VBAT voltage limit subceeded");
    }
    if (data & 0x01)
    {
        ESP_LOGI(TAG, "VBUS power limit exceeded");
    }
    sw7203_check_error(sw7203_register_read(0x06, &data));
    if (data & 0x02)
    {
        NVDC_BAT_charge = true;
    }
    else
    {
        NVDC_BAT_charge = false;
    }
    uint8_t cmd[2][2] = {{0x04, 0b11111111}, {0x05, 0b11111111}};
    sw7203_register_write(cmd[0]);
    sw7203_register_write(cmd[1]);
}
#endif
extern "C" void app_main()
{
    ErrLED.set_high();
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

    if ((err_code = tinyusb_driver_install(&tusb_cfg)) != ESP_OK)
    {
        ESP_LOGI(TAG, "TinyUSB driver error\n:)");
        ESP_ERROR_CHECK_WITHOUT_ABORT(err_code);
        while (1)
        {
            ErrLED.set_high();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ErrLED.set_low();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "USB initialization DONE");
#endif
    uint8_t sw7203_config_data[][2] = {
        {0x02, 0b00000011},
        // 中断使能1 0使能 1禁止 7:NULL 6:VSYS过压中断 5:充电超时中断 4:充电充满中断
        // 3: 适配器插入中断 2:适配器移出中断 1:A2负载接入中断 0:A1负载接入中断
        {0x03, 0b01000110},
        {0x04, 0b11111111},
        {0x05, 0b11111111},
        {0x0D, 0b00000000},
        {0x0F, 0b00000001},
        {0x10, 0b01000001},
        {0x18, 0b00000011},
        {0x19, 0b00000000},
        {0x20, 0b10000100},
        {0x21, 0b11111111},
        {0x22, 0b10100000},
        {0x26, 0b01011001},
        {0x27, 0b01010101},
        {0x28, 0b00000100},
        {0x30, 0b00000011},
        {0x31, 0b00000000},
        {0x32, 0b11010000},
        {0x34, 0b10101010},
        {0x35, 0b00000000},
        {0x36, 0b01011111},
        {0x37, 0b00001111},
        {0x38, 0b00000100},
        {0x39, 0b01111111},
        {0x3A, 0b00010011},
#ifdef __SW7203_DEBUG__
        {0x40, 0b01000011},
#else
        {0x40, 0b00000011},
#endif
        {0x41, 0b00000100},
        {0x42, 0b00100101}};
    if ((err_code = bq4050_i2c_master_init()) != ESP_OK)
    {
        ESP_LOGI(TAG, "BQ4050 driver error\n:)");
        ESP_ERROR_CHECK_WITHOUT_ABORT(err_code);
        while (1)
        {
            ErrLED.set_high();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ErrLED.set_low();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    BatteryVoltage = bq_GetVoltage();
    BatteryCurrentCapacity = bq_GetRSOC();
    BatteryRunTimeToEmpty = bq_GetT2E();
    BatteryRunTimeToFull = bq_GetT2F();
    ESP_LOGI(TAG, "BQ4050 initialization DONE!\n:)");
#ifndef __SW7203_DEBUG__
    gpio_config_t SW7203_IRQ_gpio_config = {
        .pin_bit_mask = 1ull << 47,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&SW7203_IRQ_gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_47, sw7203_irq_func, (void *)GPIO_NUM_47);
    gpio_intr_enable(GPIO_NUM_47);
    ESP_LOGI(TAG, "SW7203 intrupt initialization DONE!\n:)");
#endif
    if ((err_code = sw7203_i2c_master_init()) != ESP_OK)
    {
        ESP_LOGI(TAG, "SW7203 driver error\n:)");
        ESP_ERROR_CHECK_WITHOUT_ABORT(err_code);
        while (1)
        {
            ErrLED.set_high();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ErrLED.set_low();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    for (int i = 0; i < 27; i++)
    {
        if ((err_code = sw7203_register_write(sw7203_config_data[i])) != ESP_OK)
        {
            ESP_LOGI(TAG, "SW7203 I2C error\n:)");
            ESP_ERROR_CHECK_WITHOUT_ABORT(err_code);
            while (1)
            {
                ErrLED.set_high();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                ErrLED.set_low();
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
    }
    ESP_LOGI(TAG, "SW7203 initialization DONE!\n:)");
    ESP_LOGI(TAG, "initialization DONE!\n:)");
    ErrLED.set_low();

    while (1)
    {
        BatteryCurrentCapacity = bq_GetRSOC();
        BatteryRunTimeToEmpty = bq_GetT2E();
        BatteryCurrentStatus = bq_BattState_u16(AC_IN, NVDC_BAT_charge);
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
        if (BatteryCurrentStatus & (1 << PRESENTSTATUS_CHARGING))
            ChargingLED.set_high();
        else
            ChargingLED.set_low();
        ESP_LOGI(TAG, "Battery Current Capacity :%d\nBattery Current Status :%d\nBattery RunTime To Empty :%d\n", BatteryCurrentCapacity, BatteryCurrentStatus, BatteryRunTimeToEmpty);

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
            BatteryCurrentStatus = bq_BattState_u16(AC_IN, NVDC_BAT_charge);
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
            BatteryCurrentCapacity = bq_GetRSOC();
            buffer[0] = BatteryCurrentCapacity;
            return 1;
        }
        case HID_PD_RUNTIMETOEMPTY:
        {

            BatteryRunTimeToEmpty = bq_GetT2E();
            buffer[0] = BatteryRunTimeToEmpty & 0x00ff;
            buffer[1] = BatteryRunTimeToEmpty >> 8 & 0x00ff;
            return 2;
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
    // ESP_LOGI(TAG, "Get_report Request: %d, type=%s , id: %d , len:%d\n",
    //          instance,
    //          (report_type == HID_REPORT_TYPE_INVALID ? "HID_REPORT_TYPE_INVALID" : (report_type == HID_REPORT_TYPE_INPUT ? "HID_REPORT_TYPE_INPUT" : (report_type == HID_REPORT_TYPE_OUTPUT ? "HID_REPORT_TYPE_OUTPUT" : "HID_REPORT_TYPE_FEATURE"))),
    //          report_id,
    //          bufsize);
    // for (int i = 0; i < bufsize; i++)
    // {
    //     ESP_LOGI(TAG, "Report %d:%d\n", i, buffer[i]);
    // }
    // ESP_LOGI(TAG, "end REQUSET\n");
}

#endif