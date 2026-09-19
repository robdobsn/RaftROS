/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main entry point
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCoreApp.h"
#include "RegisterSysMods.h"
#include "RegisterWebServer.h"
#include "MainSysMod.h"
#include "RaftROS.h"
#include "BusI2C.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Board-specific: GPIO that switches power to the I2C/STEMMA QT connector,
// driven high before the I2C bus is set up.  21 = TFT_I2C_POWER on the
// Adafruit ESP32-S3 TFT Feather (also powers its TFT).  Set to -1 for boards
// whose I2C connector is always powered.
static constexpr int BOARD_I2C_POWER_PIN = 21;

// Create the app
RaftCoreApp raftCoreApp;

// Entry point
extern "C" void app_main(void)
{
    // Power-cycle the I2C connector so devices start from a clean reset (a
    // brief chip reset may not discharge the rail) and are present for the
    // first bus scan
    if (BOARD_I2C_POWER_PIN >= 0)
    {
        gpio_reset_pin((gpio_num_t)BOARD_I2C_POWER_PIN);
        gpio_set_direction((gpio_num_t)BOARD_I2C_POWER_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)BOARD_I2C_POWER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level((gpio_num_t)BOARD_I2C_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Register SysMods from RaftSysMods library
    RegisterSysMods::registerSysMods(raftCoreApp.getSysManager());

    // Register WebServer from RaftWebServer library
    RegisterSysMods::registerWebServer(raftCoreApp.getSysManager());

    // Register BusI2C
    raftBusSystem.registerBus("I2C", BusI2C::createFn);

    // Register RaftROS SysMod for ROS 2 node discovery
    raftCoreApp.registerSysMod("RaftROS", RaftROS::create, true);

    // Register sysmod
    raftCoreApp.registerSysMod("MainSysMod", MainSysMod::create, true);

    // Loop forever
    while (1)
    {
        // Loop the app
        raftCoreApp.loop();
    }
}
