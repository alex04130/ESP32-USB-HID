#include <iostream>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

using namespace std;

extern "C" void app_main()
{
    cout << "inited!" << endl;

    while (1)
    {
        cout << "Hello World!" << endl;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
