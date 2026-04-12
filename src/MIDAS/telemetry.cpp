#include <cmath>

#include "telemetry.h"

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief This function maps an input value onto within a particular range into a fixed point value of a certin binary
 * size
 *
 * @param val: number to map into target range, values outside of the range will be clamped
 *
 * @param range: range to map number into. For unsigned output, [0, range). For signed output [-range/2, range)
 *
 * @return fixed point value represing val mapped onto the target range
 */
template <typename T>
T inv_convert_range(float val, float range) {
    size_t numeric_range = (int64_t)std::numeric_limits<T>::max() - (int64_t)std::numeric_limits<T>::min() + 1;
    float converted = val * (float)numeric_range / range;
    return std::max(std::min((float)std::numeric_limits<T>::max(), converted), (float)std::numeric_limits<T>::min());
}

/**
 * @brief packs highg and tilt infomation into 3, 2 byte integers
 * 
 * @param highg highg data to store
 * @param tilt tilt information to store
 * 
 * @return tuple with packed data
*/
std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> pack_highg_tilt(HighGData const& highg, uint16_t tilt) {
    uint16_t ax = (uint16_t)inv_convert_range<int16_t>(highg.ax, 32);
    uint16_t ay = (uint16_t)inv_convert_range<int16_t>(highg.ay, 32);
    uint16_t az = (uint16_t)inv_convert_range<int16_t>(highg.az, 32);

    uint16_t x = (ax & 0xfffc) | ((tilt >> 0) & 0x3);
    uint16_t y = (ay & 0xfffc) | ((tilt >> 2) & 0x3);
    uint16_t z = (az & 0xfffc) | ((tilt >> 4) & 0x3);
    uint16_t q = (tilt >> 6) & 15;

    return {x,y,z,q};
}

/**
 * @brief move constructor for the telemetry backend
*/
Telemetry::Telemetry(TelemetryBackend&& backend) : backend(std::move(backend)) { }

bool Telemetry::receive(TelemetryPacket* packet, int wait_milliseconds) {
    return backend.read(packet, wait_milliseconds);
}

void Telemetry::acknowledgeReceived() {
    received_count ++;
}

/**
 * @brief initializes the Telemetry system
 * 
 * @return Error Code
*/
ErrorCode __attribute__((warn_unused_result)) Telemetry::init() {
    return backend.init();
}
