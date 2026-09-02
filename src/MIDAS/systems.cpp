#include "systems.h"
#include "hal.h"

#include "sammy.h"

#define ENABLE_TELEM

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */

DECLARE_THREAD(gps, RocketSystems *arg)
{
    while (true)
    {
        if (arg->sensors.gps.valid())
        {
            GPS reading = arg->sensors.gps.read();
            arg->rocket_data.sam_gps.update(reading);
        }
        // GPS waits internally
        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(buzzer, RocketSystems *arg)
{
    while (true)
    {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

/*
 * Thread for the sam turret. 
 * Calculates the desired angles and sends them to the motor board.
 * Currently Kalman is used for Up and GPS is used for East and North in ENU.
 */
DECLARE_THREAD(sammy, RocketSystems *arg)
{   
    Sammy::SamState sam{};
    
    while (true)
    {
        sam.track(
            Sammy::GPS::fromMIDAS(arg->rocket_data.sam_gps.getRecent()), 
            Sammy::GPS::fromMIDAS(arg->rocket_data.rocket_gps.getRecent()), 
            arg->rocket_data.rocket_kalman.getRecent()
        );

        // send desired angles
        esp_err_t result = esp_now_send(
            Sammy::broadcastAddress.data(), 
            (uint8_t*)sam.currAngles(), 
            sizeof(Sammy::Angles)
        );
        if (result != ESP_OK) 
            Serial.println("Error sending data");
        THREAD_SLEEP(10);
    }
}

DECLARE_THREAD(telemetry, RocketSystems *arg)
{
    while (true)
    {
        TelemetryPacket packet;
        if (arg->tlm.receive(&packet, 2000)) {
            GPS gps_data;
            gps_data.latitude = packet.lat;
            gps_data.longitude = packet.lon;
            gps_data.altitude = packet.alt;
            arg->rocket_data.rocket_gps.update(gps_data);

            KalmanData kf_data;
            kf_data.position.px = packet.kf_px;
            arg->rocket_data.rocket_kalman.update(kf_data);
        }
        THREAD_SLEEP(1);
    }
}

#define INIT_SYSTEM(s)               \
    do                               \
    {                                \
        ErrorCode code = (s).init(); \
        if (code != NoError)         \
        {                            \
            return code;             \
        }                            \
    } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode init_systems(RocketSystems &systems)
{
    gpioDigitalWrite(LED_ORANGE, HIGH);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.tlm);
    INIT_SYSTEM(systems.sensors.gps);
    return NoError;
}
#undef INIT_SYSTEM

/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
[[noreturn]] void begin_systems(RocketSystems *config)
{
    Serial.println("Starting Systems...");
    ErrorCode init_error_code = init_systems(*config);
    if (init_error_code != NoError)
    {
        // todo some message probably
        Serial.print("Had Error: ");
        Serial.print((int)init_error_code);
        Serial.print("\n");
        Serial.flush();
        update_error_LED(init_error_code);
        while (true) {}
    }

    START_THREAD(gps, SENSOR_CORE, config, 2);
    START_THREAD(buzzer, SENSOR_CORE, config, 1);
    START_THREAD(telemetry, SENSOR_CORE, config, 4);
    START_THREAD(sammy, SENSOR_CORE, config, 3);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
    while (true)
    {
        // Serial.print("Running (Log Latency: ");
        // Serial.print(config->rocket_data.log_latency.getLatency());
        // Serial.println(")");
        THREAD_SLEEP(1000);
    }
}