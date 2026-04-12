#include "systems.h"
#include "hal.h"

#include <TCAL9539.h>
#include <esp_now.h>

#define ENABLE_TELEM
#define GPS_TOL 5e-5

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

// WGS84 constants
constexpr double a = 6378137.0;
constexpr double f = 1.0 / 298.257223563;
constexpr double e2 = 2*f - f*f;

struct Angles {
    float azimuth = 0.0;
    float elevation = 0.0;
} angles;

struct GPS_dbl {
    double lat, lon, alt;

    static GPS_dbl fromFixed(const GPS& gps) {
        return { gps.latitude * 1.0e-7, gps.longitude * 1.0e-7, gps.altitude};
    }

    bool isValid() const {
        return lat != 0.0 || lon != 0.0;
    }
};

struct Turret {
    double lat, lon, alt = 0.0;
    double ecef_x, ecef_y, ecef_z;

    // Trig cache
    double sin_lat, cos_lat;
    double sin_lon, cos_lon;
} turret;

enum class TrackingPhase {
    SWEEP,
    LOCK
};

void SamCalcs(GPS_dbl& sam_gps) {
    turret.lat = sam_gps.lat;
    turret.lon = sam_gps.lon;
    turret.alt = sam_gps.alt;

    turret.sin_lat = std::sin(turret.lat);
    turret.cos_lat = std::cos(turret.lat);
    turret.sin_lon = std::sin(turret.lon);
    turret.cos_lon = std::cos(turret.lon);

    double N = a / std::sqrt(1.0 - e2 * turret.sin_lat * turret.sin_lat);
    turret.ecef_x = (N + turret.alt) * turret.cos_lat * turret.cos_lon;
    turret.ecef_y = (N + turret.alt) * turret.cos_lat * turret.sin_lon;
    turret.ecef_z = (N * (1.0 - e2) + turret.alt) * turret.sin_lat;
}

void RocketCalcs(GPS_dbl& rocket_gps, float rocket_alt) {
    double rocket_sin_lat = std::sin(rocket_gps.lat);
    double rocket_cos_lat = std::cos(rocket_gps.lat);
    double rocket_sin_lon = std::sin(rocket_gps.lon);
    double rocket_cos_lon = std::cos(rocket_gps.lon);

    double N = a / std::sqrt(1.0 - e2 * rocket_sin_lat * rocket_sin_lat);
    double rocket_ecef_x = (N + rocket_alt) * rocket_cos_lat * rocket_cos_lon;
    double rocket_ecef_y = (N + rocket_alt) * rocket_cos_lat * rocket_sin_lon;
    double rocket_ecef_z = (N * (1.0 - e2) + rocket_alt) * rocket_sin_lat;

    double dx = rocket_ecef_x - turret.ecef_x;
    double dy = rocket_ecef_y - turret.ecef_y;
    double dz = rocket_ecef_z - turret.ecef_z;

    double east =   - turret.sin_lon * dx 
                    + turret.cos_lon * dy;
    double north =  - turret.sin_lat * turret.cos_lon * dx 
                    - turret.sin_lat * turret.sin_lon * dy 
                    + turret.cos_lat * dz;
    double up =       turret.cos_lat * turret.cos_lon * dx
                    + turret.cos_lat * turret.sin_lon * dy
                    + turret.sin_lat * dz;

    angles.azimuth = std::atan2(east, north);
    angles.elevation = std::atan2(up, std::sqrt(east * east + north * north));
    if (angles.elevation < 0.0) angles.elevation = 0.0;
}

DECLARE_THREAD(sammy, RocketSystems *arg)
{
    uint8_t broadcastAddress[] = {0xdc, 0x54, 0x75, 0xca, 0xa0, 0x10};  //DC:54:75:CA:A0:10
    TrackingPhase phase = TrackingPhase::SWEEP;
    int sweep_angle = 0.1;
    int mode = 0;   // 0 = manual, 1 = automatic
    
    while (true)
    {
        if (Serial.available()) {
            int v = Serial.read();
            if (v == '0') {
                Serial.println("Manual Mode");
                mode = 0;
            } else if (v == '1') {
                Serial.println("Auto Mode");
                mode = 1;
            }
            if (mode == 0) {
                if (v == 'w') {
                    angles.elevation += 0.1;
                    if (angles.elevation > M_PI / 2)
                        angles.elevation = M_PI / 2;
                } else if (v == 's') {
                    angles.elevation -= 0.1;
                    if (angles.elevation < 0)
                        angles.elevation = 0;
                } else if (v == 'd') {
                    angles.azimuth -= 0.1;
                } else if (v == 'a') {
                    angles.azimuth += 0.1;
                }
            }
        }

        GPS_dbl sam_gps = GPS_dbl::fromFixed(arg->rocket_data.sam_gps.getRecent());
        GPS_dbl rocket_gps = GPS_dbl::fromFixed(arg->rocket_data.rocket_gps.getRecent());
        KalmanData rocket_kalman = arg->rocket_data.rocket_kalman.getRecent();

        if (mode == 1) {
            switch (phase) {
                case TrackingPhase::SWEEP:
                    if (sam_gps.isValid() && rocket_gps.isValid()) {
                        phase = TrackingPhase::LOCK;
                        SamCalcs(sam_gps);
                        RocketCalcs(rocket_gps, rocket_kalman.position.px);
                    } else {
                        angles.azimuth += sweep_angle;
                        if (angles.azimuth > M_PI / 2) {
                            angles.azimuth = M_PI / 2;
                            sweep_angle *= -1;
                        } else if (angles.azimuth < -M_PI / 2) {
                            angles.azimuth = -M_PI / 2;
                            sweep_angle *= -1;
                        }
                    }
                    break;
                case TrackingPhase::LOCK:
                    if (sam_gps.isValid() && rocket_gps.isValid() && mode == 1) {
                        if (std::abs(sam_gps.lat - turret.lat) > GPS_TOL ||
                            std::abs(sam_gps.lon - turret.lon) > GPS_TOL)
                            SamCalcs(sam_gps);

                        RocketCalcs(rocket_gps, rocket_kalman.position.px);
                    }
                    break;
            }
        }

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&angles, sizeof(Angles));
        if (result != ESP_OK)
            Serial.println("Error sending the data");
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