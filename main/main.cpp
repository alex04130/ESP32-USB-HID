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

#define SetBitTrue(a, b) a |= (1 << b)
#define SetBitFalse(a, b) a &= ~(1 << b)
#define GetBit(a, b) a & (1 << b)

#define HID_PD_IPRODUCT 0x01     // FEATURE ONLY
#define HID_PD_SERIAL 0x02       // FEATURE ONLY
#define HID_PD_MANUFACTURER 0x03 // FEATURE ONLY
#define IDEVICECHEMISTRY 0x04
#define IOEMVENDOR 0x05

#define HID_PD_RECHARGEABLE 0x06  // FEATURE ONLY
#define HID_PD_PRESENTSTATUS 0x07 // INPUT OR FEATURE(required by Windows)
#define HID_PD_REMAINTIMELIMIT 0x08
#define HID_PD_MANUFACTUREDATE 0x09
#define HID_PD_CONFIGVOLTAGE 0x0A     // 10 FEATURE ONLY
#define HID_PD_VOLTAGE 0x0B           // 11 INPUT (NA) OR FEATURE(implemented)
#define HID_PD_REMAININGCAPACITY 0x0C // 12 INPUT OR FEATURE(required by Windows)
#define HID_PD_RUNTIMETOEMPTY 0x0D
#define HID_PD_FULLCHRGECAPACITY 0x0E // 14 INPUT OR FEATURE. Last Full Charge Capacity
#define HID_PD_WARNCAPACITYLIMIT 0x0F
#define HID_PD_CPCTYGRANULARITY1 0x10
#define HID_PD_REMNCAPACITYLIMIT 0x11
#define HID_PD_DELAYBE4SHUTDOWN 0x12 // 18 FEATURE ONLY
#define HID_PD_DELAYBE4REBOOT 0x13
#define HID_PD_AUDIBLEALARMCTRL 0x14 // 20 FEATURE ONLY
#define HID_PD_CURRENT 0x15          // 21 FEATURE ONLY
#define HID_PD_CAPACITYMODE 0x16
#define HID_PD_DESIGNCAPACITY 0x17
#define HID_PD_CPCTYGRANULARITY2 0x18
#define HID_PD_AVERAGETIME2FULL 0x1A
#define HID_PD_AVERAGECURRENT 0x1B
#define HID_PD_AVERAGETIME2EMPTY 0x1C

#define HID_PD_IDEVICECHEMISTRY 0x1F // Feature
#define HID_PD_IOEMINFORMATION 0x20  // Feature

// PresenStatus dynamic flags
#define PRESENTSTATUS_CHARGING 0x00
#define PRESENTSTATUS_DISCHARGING 0x01
#define PRESENTSTATUS_ACPRESENT 0x02
#define PRESENTSTATUS_BATTPRESENT 0x03
#define PRESENTSTATUS_BELOWRCL 0x04
#define PRESENTSTATUS_RTLEXPIRED 0x05
#define PRESENTSTATUS_NEEDREPLACE 0x06
#define PRESENTSTATUS_VOLTAGENR 0x07
#define PRESENTSTATUS_FULLCHARGE 0x08
#define PRESENTSTATUS_FULLDISCHARGE 0x09
#define PRESENTSTATUS_SHUTDOWNREQ 0x0A
#define PRESENTSTATUS_SHUTDOWNIMNT 0x0B
#define PRESENTSTATUS_COMMLOST 0x0C
#define PRESENTSTATUS_OVERLOAD 0x0D

// 字符串描述符位置
#define IPRODUCT 2
#define ISERIAL 3
#define IMANUFACTURER 1

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define ITF_NUM_TOTAL 1

// Physical parameters
const uint16_t iConfigVoltage = 1380;
uint16_t iVoltage = 1300, iPrevVoltage = 0;
uint16_t iRunTimeToEmpty = 0, iPrevRunTimeToEmpty = 0;
uint16_t iAvgTimeToFull = 7200;
uint16_t iAvgTimeToEmpty = 7200;
uint16_t iRemainTimeLimit = 600;
int16_t iDelayBe4Reboot = -1;
int16_t iDelayBe4ShutDown = -1;
uint16_t IMANUFACTURERDATE = 100;

// Parameters for ACPI compliancy
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

static const char *TAG = "SKele-DC";

unsigned char bRechargable = 1;
unsigned char bCapacityMode = 2; // units are in %%

uint16_t iPresentStatus = 0, iPreviousStatus = 0;

const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "TinyUSB",               // 1: Manufacturer
    "TinyUSB Device",        // 2: Product
    "123456",                // 3: Serials, should use chip ID
    "Example HID interface", // 4: HID
};

// typedef struct
// {
//     tusb_desc_device_t *descriptor;   //    设备描述符结构体
//     const char **string_descriptor;   //    字符串描述符
//     const uint8_t *config_descriptor; //    配置描述符数组
//     bool external_phy;                //    外部PHY，一般为false
// } tinyusb_config_t;

// typedef struct TU_ATTR_PACKED
// {
//     uint8_t bLength;         //    设备描述符的字节数大小
//     uint8_t bDescriptorType; //    描述符类型，设备描述符为0x01
//     uint16_t bcdUSB;         //    USB版本号

//     uint8_t bDeviceClass;    //    USB分配的设备类代码，0x01~0xfe为标准设备类，0xff为厂商自定义类型
//     uint8_t bDeviceSubClass; //    USB分配的子类代码
//     uint8_t bDeviceProtocol; //    USB分配的设备协议代码
//     uint8_t bMaxPacketSize0; //    端点0的最大信息包大小

//     uint16_t idVendor;  //    制造商ID
//     uint16_t idProduct; //    产品ID

//     uint16_t bcdDevice; //    设备出厂编号

//     uint8_t iManufacturer; //    制造商的字符串描述符索引
//     uint8_t iProduct;      //    产品的字符串描述符索引
//     uint8_t iSerialNumber; //    设备序列号的字符串描述符索引

//     uint8_t bNumConfigurations; //    可能的配置数量
// } tusb_desc_device_t;

tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_INTERFACE_POWER, //    0x01
    .bcdUSB = 0x0110,                             //    USB2.0

    .bDeviceClass = TUSB_CLASS_HID,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, //    64

    .idVendor = 0x303A,
    .idProduct = 0x4002, //    0x4010
    .bcdDevice = 0x100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01};

const char *string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x08, 0x04},     // 0: 支持语言：中文 (0x0809)
    "Skele",                  // 1: 制造商
    "Skele-DC",               // 2: 产品
    "Skele-DC1",              // 3: 串行、芯片ID
    "SKeleUSB HID"            // 4: HID
    "SKele USB HID interface" // 5: 配置描述符
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

uint8_t const desc_configuration[] = {
    //    配置描述符
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0x04, TUSB_DESC_TOTAL_LEN, 0, 100),

    //    接口描述符、HID描述符、端点描述符
    TUD_HID_INOUT_DESCRIPTOR(1, 0x04, 0, sizeof(desc_hid_report), 0x01, 0x81, 64, 10)};

extern "C" void app_main()
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
        .self_powered = false,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    ESP_LOGI(TAG, "initialization DONE!\n:)");

    while (1)
    {
        ESP_LOGI(TAG, "still on");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
            buffer[0] = iPresentStatus & 0x00ff;
            buffer[1] = iPresentStatus >> 8 & 0x00ff;
            return 2;
        }
        case HID_PD_VOLTAGE:
        {
            buffer[0] = iVoltage & 0x00ff;
            buffer[1] = iVoltage >> 8 & 0x00ff;
            return 2;
        }
        case HID_PD_DESIGNCAPACITY:
        {
            buffer[0] = iDesignCapacity;
            return 1;
        }
        case HID_PD_IDEVICECHEMISTRY:
        {
            buffer[0] = bDeviceChemistry;
            return 1;
        }
        case HID_PD_SERIAL:
        {
            buffer[0] = 3;
            return 1;
        }
        case HID_PD_IOEMINFORMATION:
        {
            buffer[0] = bOEMVendor;
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
            buffer[0] = IMANUFACTURERDATE & 0x00ff;
            buffer[1] = IMANUFACTURERDATE >> 8 & 0x00ff;
            return 2;
        }
        case HID_PD_FULLCHRGECAPACITY:
        {
            buffer[0] = iFullChargeCapacity;
            return 1;
        }
        case HID_PD_WARNCAPACITYLIMIT:
        {
            buffer[0] = iWarnCapacityLimit;
            return 1;
        }
        case HID_PD_REMNCAPACITYLIMIT:
        {
            buffer[0] = iRemnCapacityLimit;
            return 1;
        }
        case HID_PD_REMAININGCAPACITY:
        {
            buffer[0] = iRemaining;
            return 1;
        }
        case HID_PD_RUNTIMETOEMPTY:
        {
            buffer[0] = iRunTimeToEmpty & 0x00ff;
            buffer[1] = iRunTimeToEmpty >> 8 & 0x00ff;
            return 2;
        }
        case HID_PD_CPCTYGRANULARITY1:
        {
            buffer[0] = bCapacityGranularity1;
            return 1;
        }
        case HID_PD_CPCTYGRANULARITY2:
        {
            buffer[0] = bCapacityGranularity2;
            return 1;
        }
        case HID_PD_CAPACITYMODE:
        {
            buffer[0] = bCapacityMode;
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
    return desc_hid_report;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
}
