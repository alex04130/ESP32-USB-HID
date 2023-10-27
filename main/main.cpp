#include <iostream>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

using namespace std;

static const char *TAG = "SKele-DC";

const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "TinyUSB",               // 1: Manufacturer
    "TinyUSB Device",        // 2: Product
    "123456",                // 3: Serials, should use chip ID
    "Example HID interface", // 4: HID
};

typedef struct
{
    tusb_desc_device_t *descriptor;   //    设备描述符结构体
    const char **string_descriptor;   //    字符串描述符
    const uint8_t *config_descriptor; //    配置描述符数组
    bool external_phy;                //    外部PHY，一般为false
} tinyusb_config_t;

typedef struct TU_ATTR_PACKED
{
    uint8_t bLength;         //    设备描述符的字节数大小
    uint8_t bDescriptorType; //    描述符类型，设备描述符为0x01
    uint16_t bcdUSB;         //    USB版本号

    uint8_t bDeviceClass;    //    USB分配的设备类代码，0x01~0xfe为标准设备类，0xff为厂商自定义类型
    uint8_t bDeviceSubClass; //    USB分配的子类代码
    uint8_t bDeviceProtocol; //    USB分配的设备协议代码
    uint8_t bMaxPacketSize0; //    端点0的最大信息包大小

    uint16_t idVendor;  //    制造商ID
    uint16_t idProduct; //    产品ID

    uint16_t bcdDevice; //    设备出厂编号

    uint8_t iManufacturer; //    制造商的字符串描述符索引
    uint8_t iProduct;      //    产品的字符串描述符索引
    uint8_t iSerialNumber; //    设备序列号的字符串描述符索引

    uint8_t bNumConfigurations; //    可能的配置数量
} tusb_desc_device_t;

tusb_desc_device_t descriptor_kconfig = {
    .bLength = sizeof(descriptor_kconfig),
    .bDescriptorType = TUSB_DESC_INTERFACE_POWER, //    0x01
    .bcdUSB = 0x0200,                             //    USB2.0

    .bDeviceClass = TUSB_CLASS_HID,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, //    64

    .idVendor = 0xCafe,
    .idProduct = USB_TUSB_PID, //    0x4010
    .bcdDevice = 0x0001,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01};

const char *string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: 支持语言：英语 (0x0409)
    "Skele",              // 1: 制造商
    "Skele-DC",           // 2: 产品
    "123456",             // 3: 串行、芯片ID
    "TinyUSB HID"         // 4: HID
    "What is it?"         // 5: 配置描述符
};

uint8_t const desc_configuration[] = {
    0x09, // 字节数
    0x02, // 以字节为单位的描述符大小
    0xff,
    0xff, // 配置返回的数据总长度
    0x01, // 配置支持的接口数量
    0x01, // Get Configuration 和Set Configuration请求的配置值
    0x05, // 字符串描述符索引
    0x20, // 配置特性
    0x00  // 设备从总线获取的最大功耗
};

uint8_t const desc_hid_report[] = {
    0x05, 0x84,                     // USAGE_PAGE (Power Device)
    0x09, 0x04,                     // USAGE (UPS)
    0xA1, 0x01,                     // COLLECTION (Application)
    0x09, 0x24,                     //   USAGE (Sink)
    0xA1, 0x02,                     //   COLLECTION (Logical)
    0x75, 0x08,                     //     REPORT_SIZE (8)
    0x95, 0x01,                     //     REPORT_COUNT (1)
    0x15, 0x00,                     //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,               //     LOGICAL_MAXIMUM (255)
    0x85, HID_PD_IPRODUCT,          //     REPORT_ID (1)
    0x09, 0xFE,                     //     USAGE (iProduct)
    0x79, IPRODUCT,                 //     STRING INDEX (2)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_SERIAL,            //     REPORT_ID (2)
    0x09, 0xFF,                     //     USAGE (iSerialNumber)
    0x79, ISERIAL,                  //  STRING INDEX (3)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_MANUFACTURER,      //     REPORT_ID (3)
    0x09, 0xFD,                     //     USAGE (iManufacturer)
    0x79, IMANUFACTURER,            //     STRING INDEX (1)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x05, 0x85,                     //     USAGE_PAGE (Battery System) ====================
    0x85, HID_PD_RECHARGEABLE,      //     REPORT_ID (6)
    0x09, 0x8B,                     //     USAGE (Rechargable)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_IDEVICECHEMISTRY,  //     REPORT_ID (31)
    0x09, 0x89,                     //     USAGE (iDeviceChemistry)
    0x79, IDEVICECHEMISTRY,         //     STRING INDEX (4)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_IOEMINFORMATION,   //     REPORT_ID (32)
    0x09, 0x8F,                     //     USAGE (iOEMInformation)
    0x79, IOEMVENDOR,               //     STRING INDEX (5)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_CAPACITYMODE,      //     REPORT_ID (22)
    0x09, 0x2C,                     //     USAGE (CapacityMode)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_CPCTYGRANULARITY1, //     REPORT_ID (16)
    0x09, 0x8D,                     //     USAGE (CapacityGranularity1)
    0x26, 0x64, 0x00,               //     LOGICAL_MAXIMUM (100)
    0xB1, 0x22,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_CPCTYGRANULARITY2, //     REPORT_ID (24)
    0x09, 0x8E,                     //     USAGE (CapacityGranularity2)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_FULLCHRGECAPACITY, //     REPORT_ID (14)
    0x09, 0x67,                     //     USAGE (FullChargeCapacity)
    0xB1, 0x83,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_DESIGNCAPACITY,    //     REPORT_ID (23)
    0x09, 0x83,                     //     USAGE (DesignCapacity)
    0xB1, 0x83,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_REMAININGCAPACITY, //     REPORT_ID (12)
    0x09, 0x66,                     //     USAGE (RemainingCapacity)
    0x81, 0xA3,                     //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x66,                     //     USAGE (RemainingCapacity)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_WARNCAPACITYLIMIT, //     REPORT_ID (15)
    0x09, 0x8C,                     //     USAGE (WarningCapacityLimit)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_REMNCAPACITYLIMIT, //     REPORT_ID (17)
    0x09, 0x29,                     //     USAGE (RemainingCapacityLimit)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_MANUFACTUREDATE,   //     REPORT_ID (9)
    0x09, 0x85,                     //     USAGE (ManufacturerDate)
    0x75, 0x10,                     //     REPORT_SIZE (16)
    0x27, 0xFF, 0xFF, 0x00, 0x00,   //     LOGICAL_MAXIMUM (65534)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_AVERAGETIME2FULL,  //     REPORT_ID (26)
    0x09, 0x6A,                     //     USAGE (AverageTimeToFull)
    0x27, 0xFF, 0xFF, 0x00, 0x00,   //     LOGICAL_MAXIMUM (65534)
    0x66, 0x01, 0x10,               //     UNIT (Seconds)
    0x55, 0x00,                     //     UNIT_EXPONENT (0)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_AVERAGETIME2EMPTY, //     REPORT_ID (28)
    0x09, 0x69,                     //     USAGE (AverageTimeToEmpty)
    0x81, 0xA3,                     //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x69,                     //     USAGE (AverageTimeToEmpty)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_RUNTIMETOEMPTY,    //     REPORT_ID (13)
    0x09, 0x68,                     //     USAGE (RunTimeToEmpty)
    0x81, 0xA3,                     //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x68,                     //     USAGE (RunTimeToEmpty)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_REMAINTIMELIMIT,   //     REPORT_ID (8)
    0x09, 0x2A,                     //     USAGE (RemainingTimeLimit)
    0x75, 0x10,                     //     REPORT_SIZE (16)
    0x27, 0x64, 0x05, 0x00, 0x00,   //     LOGICAL_MAXIMUM (1380)
    0x16, 0x78, 0x00,               //     LOGICAL_MINIMUM (120)
    0x81, 0x22,                     //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x2A,                     //     USAGE (RemainingTimeLimit)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x05, 0x84,                     //     USAGE_PAGE (Power Device) ====================
    0x85, HID_PD_DELAYBE4SHUTDOWN,  //     REPORT_ID (18)
    0x09, 0x57,                     //     USAGE (DelayBeforeShutdown)
    0x16, 0x00, 0x80,               //     LOGICAL_MINIMUM (-32768)
    0x27, 0xFF, 0x7F, 0x00, 0x00,   //     LOGICAL_MAXIMUM (32767)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_DELAYBE4REBOOT,    //     REPORT_ID (19)
    0x09, 0x55,                     //     USAGE (DelayBeforeReboot)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_CONFIGVOLTAGE,     //     REPORT_ID (10)
    0x09, 0x40,                     //     USAGE (ConfigVoltage)
    0x15, 0x00,                     //     LOGICAL_MINIMUM (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,   //     LOGICAL_MAXIMUM (65535)
    0x67, 0x21, 0xD1, 0xF0, 0x00,   //     UNIT (Centivolts)
    0x55, 0x05,                     //     UNIT_EXPONENT (5)
    0xB1, 0x23,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Nonvolatile, Bitfield)
    0x85, HID_PD_VOLTAGE,           //     REPORT_ID (11)
    0x09, 0x30,                     //     USAGE (Voltage)
    0x81, 0xA3,                     //     INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x30,                     //     USAGE (Voltage)
    0xB1, 0xA3,                     //     FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x85, HID_PD_AUDIBLEALARMCTRL,  //     REPORT_ID (20)
    0x09, 0x5A,                     //     USAGE (AudibleAlarmControl)
    0x75, 0x08,                     //     REPORT_SIZE (8)
    0x15, 0x01,                     //     LOGICAL_MINIMUM (1)
    0x25, 0x03,                     //     LOGICAL_MAXIMUM (3)
    0x65, 0x00,                     //     UNIT (0)
    0x55, 0x00,                     //     UNIT_EXPONENT (0)
    0x81, 0x22,                     //     INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x5A,                     //     USAGE (AudibleAlarmControl)
    0xB1, 0xA2,                     //     FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x02,                     //     USAGE (PresentStatus)
    0xA1, 0x02,                     //     COLLECTION (Logical)
    0x85, HID_PD_PRESENTSTATUS,     //       REPORT_ID (7)
    0x05, 0x85,                     //       USAGE_PAGE (Battery System) =================
    0x09, 0x44,                     //       USAGE (Charging)
    0x75, 0x01,                     //       REPORT_SIZE (1)
    0x15, 0x00,                     //       LOGICAL_MINIMUM (0)
    0x25, 0x01,                     //       LOGICAL_MAXIMUM (1)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x44,                     //       USAGE (Charging)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x45,                     //       USAGE (Discharging)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x45,                     //       USAGE (Discharging)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0xD0,                     //       USAGE (ACPresent)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0xD0,                     //       USAGE (ACPresent)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0xD1,                     //       USAGE (BatteryPresent)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0xD1,                     //       USAGE (BatteryPresent)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x42,                     //       USAGE (BelowRemainingCapacityLimit)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x42,                     //       USAGE (BelowRemainingCapacityLimit)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x43,                     //       USAGE (RemainingTimeLimitExpired)
    0x81, 0xA2,                     //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x43,                     //       USAGE (RemainingTimeLimitExpired)
    0xB1, 0xA2,                     //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x4B,                     //       USAGE (NeedReplacement)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x4B,                     //       USAGE (NeedReplacement)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0xDB,                     //       USAGE (VoltageNotRegulated)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0xDB,                     //       USAGE (VoltageNotRegulated)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x46,                     //       USAGE (FullyCharged)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x46,                     //       USAGE (FullyCharged)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x47,                     //       USAGE (FullyDischarged)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x47,                     //       USAGE (FullyDischarged)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x05, 0x84,                     //       USAGE_PAGE (Power Device) =================
    0x09, 0x68,                     //       USAGE (ShutdownRequested)
    0x81, 0xA2,                     //       INPUT (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x68,                     //       USAGE (ShutdownRequested)
    0xB1, 0xA2,                     //       FEATURE (Data, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x69,                     //       USAGE (ShutdownImminent)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x69,                     //       USAGE (ShutdownImminent)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x73,                     //       USAGE (CommunicationLost)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x73,                     //       USAGE (CommunicationLost)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x09, 0x65,                     //       USAGE (Overload)
    0x81, 0xA3,                     //       INPUT (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Bitfield)
    0x09, 0x65,                     //       USAGE (Overload)
    0xB1, 0xA3,                     //       FEATURE (Constant, Variable, Absolute, No Wrap, Linear, No Preferred, No Null Position, Volatile, Bitfield)
    0x95, 0x02,                     //       REPORT_COUNT (2)
    0x81, 0x01,                     //       INPUT (Constant, Array, Absolute)
    0xB1, 0x01,                     //       FEATURE (Constant, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Nonvolatile, Bitfield)
    0xC0,                           //     END_COLLECTION
    0xC0,                           //   END_COLLECTION
    0xC0                            // END_COLLECTION
};

extern "C" void app_main()
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {};

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    ESP_LOGI(TAG, "initialization DONE!\n:)");

    while (1)
    {
        cout << "Hello World!" << endl;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
