#pragma once

#include <array>

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"
#include "buzzer.h"
#include "led.h"
#include "telemetry.h"
#include "MIDAS/finite-state-machines/fsm.h"
#include "b2b_interface.h"

#include "MIDAS/hardware/sensors.h"

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    LowGSensor low_g;
    LowGLSMSensor low_g_lsm;
    HighGSensor high_g;
    BarometerSensor barometer;
    ContinuitySensor continuity;
    VoltageSensor voltage;
    OrientationSensor orientation;
    MagnetometerSensor magnetometer;
    Pyro pyro;
    GPSSensor gps;
};

/**
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    BuzzerController buzzer;
    LEDController led;
    Telemetry tlm;
    B2BInterface b2b;
};

[[noreturn]] void begin_systems(RocketSystems* config);
