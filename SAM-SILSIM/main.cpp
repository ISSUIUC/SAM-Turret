#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <iomanip>
#include "sensor_data.h"
#include "sammy.h"

constexpr float PI = 3.14159265358979323846f;
// g++ -std=c++17 -I .. main.cpp sammy.cpp -o main


Vec3 anglesToNormalVector(float azimuth, float elevation) {
    Vec3 n;
    n.x = -std::cos(elevation) * std::sin(azimuth); 
    n.y =  std::cos(elevation) * std::cos(azimuth); 
    n.z =  std::sin(elevation);                   
    return n;
}

struct FlightDataPoint {
    uint32_t timestamp_ms;
    double latitude;
    double longitude;
    double altitude;
    double kalman_alt;
};

struct TotalTelemetry {
    GPS sam_gps{};
    GPS rocket_gps{};
    KalmanData rocket_kalman{};
    bool running{true};
};


std::vector<FlightDataPoint> LoadData(const std::string& path) {
    std::vector<FlightDataPoint> points;
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << path << "\n";
        return points;
    }

    std::string line;
    std::getline(file, line); // Skip header row

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> row;

        while (std::getline(ss, val, ',')) {
            row.push_back(val);
        }

        // Ensure row has enough columns and only process GPS readings
        if (row.size() > 31 && row[2] == "gps") {
            try {
                FlightDataPoint pt{};
                pt.timestamp_ms = std::stoul(row[1]);
                // MIDAS stores lat/lon multiplied by 1e7; divide to get true degrees
                pt.latitude     = std::stod(row[21]) * 1e-7; 
                pt.longitude    = std::stod(row[22]) * 1e-7;
                pt.altitude     = std::stod(row[23]);
                // Column 31 is kalman.position.px (rocket altitude)
                pt.kalman_alt   = !row[31].empty() ? std::stod(row[31]) : pt.altitude;
                points.push_back(pt);
            } catch (const std::exception& e) {
                continue;
            }
        }
    }

    std::cout << "[SILSIM] Successfully loaded " << points.size() << " GPS data points from CSV.\n";
    return points;
}

void sammy_thread_silsim(Sammy::SamState* sam, TotalTelemetry* telem, const std::string& output_csv_path) {
    std::ofstream out_file(output_csv_path);
    out_file << "timestamp_ms,azimuth_rad,elevation_rad,azimuth_deg,elevation_deg,norm_east,norm_north,norm_up\n";
    std::cout << "\n starting silsim \n";
    std::cout << std::setw(10) << "Time (ms)" 
              << std::setw(14) << "Azimuth(°)" 
              << std::setw(14) << "Elevation(°)" 
              << std::setw(12) << "Norm X(E)" 
              << std::setw(12) << "Norm Y(N)" 
              << std::setw(12) << "Norm Z(U)" << "\n";
    std::cout << std::string(74, '-') << "\n";

    while (telem->running) {
        GPS cur_sam, cur_rocket;
        KalmanData cur_kalman;

        cur_sam = telem->sam_gps;
        cur_rocket = telem->rocket_gps;
        cur_kalman = telem->rocket_kalman;

        // sammy GPS 
        Sammy::GPS sam_gps_conv = Sammy::GPS::fromMIDAS(cur_sam);
        Sammy::GPS rocket_gps_conv = Sammy::GPS::fromMIDAS(cur_rocket);
        // feed into tracking 
        sam->track(sam_gps_conv, rocket_gps_conv, cur_kalman);

        Sammy::Angles* angles = sam->currAngles();
        Vec3 norm = anglesToNormalVector(angles->azimuth, angles->elevation);

        float az_deg = angles->azimuth * (180.0f / PI);
        float el_deg = angles->elevation * (180.0f / PI);

        // Write every packet to the output CSV
        out_file << cur_sam.time << ","
                 << angles->azimuth << "," << angles->elevation << ","
                 << az_deg << "," << el_deg << ","
                 << norm.x << "," << norm.y << "," << norm.z << "\n";

        std::cout << std::setw(10) << cur_sam.time
                  << std::setw(14) << az_deg
                  << std::setw(14) << el_deg
                  << std::setw(12) << norm.x
                  << std::setw(12) << norm.y
                  << std::setw(12) << norm.z << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz 
    }

    out_file.close();
    std::cout << "Results saved to: " << output_csv_path << "\n";
}

int main(int argc, char* argv[]) {
    std::string input_csv = (argc > 1) ? argv[1] : "./data/midas_sustainer_trimmed.csv";
    std::string output_csv = (argc > 2) ? argv[2] : "./output/turret_silsim_output.csv";

    std::vector<FlightDataPoint> flight_data = LoadData(input_csv);
    if (flight_data.empty()) {
        std::cerr << "No flight data to simulate. Exiting.\n";
        return 1;
    }

    TotalTelemetry telem;
    Sammy::SamState sam;



    GPS sam_base;
    sam_base.latitude  = static_cast<int32_t>(41.486532 * 1e7); // reference
    sam_base.longitude = static_cast<int32_t>(-89.500903 * 1e7); // reference 
    sam_base.altitude  = static_cast<float>(flight_data[0].kalman_alt); // launch site 

    //Initial Rocket Position 
    GPS initial_rocket;
    initial_rocket.latitude  = static_cast<int32_t>(flight_data[0].latitude * 1e7);
    initial_rocket.longitude = static_cast<int32_t>(flight_data[0].longitude * 1e7);
    initial_rocket.altitude  = static_cast<float>(flight_data[0].altitude);
    initial_rocket.time      = flight_data[0].timestamp_ms;

    KalmanData initial_kalman;
    initial_kalman.position.px = static_cast<float>(flight_data[0].kalman_alt);


    telem.sam_gps = sam_base;
    telem.rocket_gps = initial_rocket;
    telem.rocket_kalman = initial_kalman;


    // Launch sammy_thread
    std::thread worker(sammy_thread_silsim, &sam, &telem, output_csv);

    // Feeder loop: feed historical CSV points
    for (size_t i = 0; i < flight_data.size(); ++i) {
        const auto& pt = flight_data[i];

        GPS rocket_gps;
        rocket_gps.latitude  = static_cast<int32_t>(pt.latitude * 1e7);
        rocket_gps.longitude = static_cast<int32_t>(pt.longitude * 1e7);
        rocket_gps.altitude  = static_cast<float>(pt.altitude);
        rocket_gps.time      = pt.timestamp_ms;

        KalmanData kalman_data;
        kalman_data.position.px = static_cast<float>(pt.kalman_alt);

    
        telem.rocket_gps = rocket_gps;
        telem.rocket_kalman = kalman_data;
        telem.sam_gps.time = pt.timestamp_ms;
    
        //100Hz telemetry 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    telem.running = false;

    if (worker.joinable()) {
        worker.join();
    }

    return 0;
}