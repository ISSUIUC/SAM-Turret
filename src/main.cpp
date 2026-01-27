#include <Arduino.h>
// #include <TMCStepper.h>
#include <TMCDriver.h>
#include <SPI.h>
#include <cmath>
#include <array>
#include <Wifi.h>
#include <esp_now.h>
#include "include/conversion.h"

#define SPI_SCK 3
#define SPI_MOSI 2
#define SPI_MISO 1
#define EN_0 12
#define CS_0 6
#define STEP_0 4
#define DIR_0 5
#define SG_0 7
#define EN_1 42
#define CS_1 41
#define STEP_1 43
#define DIR_1 44
#define SG_1 45
#define LED0 9
#define LED1 8
#define BOOT 0

struct GpsData { 
    float sammy_lat;
    float sammy_lon;
    float sammy_alt;
    float sammy_pitch;
    float sammy_yaw;
    float rocket_lat;
    float rocket_lon; 
    float rocket_alt;
    int control_mode;
};

struct {
    std::array<int,2> real_position{};
    std::array<float, 2> ideal_position{};
    std::array<int,2> target_position{};
    uint64_t last_update_time = 0;
    std::array<float,2> speed{0, 0};
    std::array<float,2> max_speed{50000,50000};
    std::array<float,2> max_accel{100000,100000};
    std::array<float,2> max_decel{50000,50000}; 
    std::array<int,2> step_per_rev{int(4.125*200),int(4.667*200)};
    int microsteps = 32;
} motor_config;

GpsData gps;

TMC2660 driver(CS_0, SG_0);

bool sdoff = false;
uint8_t log_counter = 0;

void update_position(
    double goal_lat, double goal_lon, double goal_alt,
    double curr_lat, double curr_lon, double curr_alt, 
    double curr_pitch, double curr_yaw, 
    int mode
) {
    // current is turret, goal is rockect
    double curr_x, curr_y, curr_z;
    double goal_x, goal_y, goal_z;

    gps_to_ecef(curr_lat, curr_lon, curr_alt, curr_x, curr_y, curr_z);
    gps_to_ecef(goal_lat, goal_lon, goal_alt, goal_x, goal_y, goal_z);

    double east, north, up;
    ecef_to_enu(
            goal_x, goal_y, goal_z, 
            curr_lat, curr_lon, 
            curr_x, curr_y, curr_z, 
            east, north, up
    );

    double goal_pitch, goal_yaw;
    calculate_pitch_yaw(east, north, up, goal_pitch, goal_yaw);

    if (mode == 0) {
        goal_pitch = curr_pitch;
        goal_yaw = curr_yaw;
    }

    int steps_pitch = motor_config.step_per_rev[0] * motor_config.microsteps * goal_pitch / (2 * M_PI);
    int steps_yaw  = motor_config.step_per_rev[1] * motor_config.microsteps * goal_yaw / (2 * M_PI);

    if(goal_pitch < 0) steps_pitch = 0;
    if(goal_pitch > (M_PI / 2)) steps_pitch = motor_config.step_per_rev[0] * motor_config.microsteps * 0.25;

    motor_config.target_position[0] = steps_pitch;
    motor_config.target_position[1] = steps_yaw;
}

void on_data_recv(const uint8_t * mac, const uint8_t * incomingData, int len) {
    if (len != sizeof(GpsData)) {
        Serial.println("Received data of incorrect length");
        return;
    }

    memcpy(&gps, incomingData, len);
    if ((gps.rocket_lat == 0 || gps.rocket_lon == 0) && gps.control_mode == 1) {
        return;
    }
    
    update_position(
        gps.rocket_lat, gps.rocket_lon, gps.rocket_alt,
        gps.sammy_lat, gps.sammy_lon, gps.sammy_alt, 
        gps.sammy_pitch, gps.sammy_yaw, 
        gps.control_mode
    );
}

void step(int channel, bool dir) {
    if (channel == 0) {
        digitalWrite(DIR_0, dir);
        digitalWrite(STEP_0, HIGH);
        digitalWrite(STEP_0, LOW);
    }
    if (channel == 1) {
        digitalWrite(DIR_1, dir);
        digitalWrite(STEP_1, HIGH);
        digitalWrite(STEP_1, LOW);
    }   
}

void move_to_position() {
    uint64_t now = micros();
    if (motor_config.last_update_time == 0) {
        motor_config.last_update_time = now;
    }
    float dt = (now - motor_config.last_update_time) / 1000000.0;
    motor_config.last_update_time = now;

    for (int motor = 0; motor < 2; motor++) {
        // integration phase
        float& pos = motor_config.ideal_position[motor];
        float target = motor_config.target_position[motor];
        float& speed = motor_config.speed[motor];
        float accel = motor_config.max_accel[motor];
        float decel = motor_config.max_decel[motor];
        pos += speed * dt;

        // step phase
        if (pos > motor_config.real_position[motor] + 1) {
            step(motor, true);
            motor_config.real_position[motor] += 1;
            if (log_counter % 5 == 0) {
                Serial.print(pos);
                Serial.print(' ');
                Serial.print(speed);
                Serial.print(' ');
                Serial.println(dt * 1000000); // s to μs
            }
        }
        if (pos < motor_config.real_position[motor] - 1) {
            step(motor, false);
            motor_config.real_position[motor] -= 1;
            if (log_counter % 5 == 0) {
                Serial.print(pos);
                Serial.print(' ');
                Serial.print(speed);
                Serial.print(' ');
                Serial.println(dt * 1000000); // s to μs
            }
        }

        // control phase
        float dir = speed > 0 ? 1 : -1;
        float decel_pos = pos + speed * speed / decel / 2.0 * dir;
        if ((decel_pos - target) * dir >= 0) {
            speed -= decel * dt * dir;
        } else if (abs(speed) < motor_config.max_speed[motor]) {
            speed += accel * dt * dir;
        }
    }
    log_counter++;
    if(log_counter % 10 == 0) {
        Serial.print("Real Position: ");
        Serial.print(motor_config.real_position[0]);
        Serial.print(' ');
        Serial.println(motor_config.real_position[1]);
        log_counter = 0;
    }
}

//Below is saved for the communication between midas and the motor board.

void setup() {
    Serial.begin(9600);
    SPI.begin(SPI_SCK,SPI_MISO,SPI_MOSI);
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(EN_0, OUTPUT);
    pinMode(EN_1, OUTPUT);
    pinMode(STEP_0, OUTPUT);
    pinMode(STEP_1, OUTPUT);
    pinMode(DIR_0, OUTPUT);
    pinMode(DIR_1, OUTPUT);
    pinMode(CS_0, OUTPUT);
    pinMode(CS_1, OUTPUT);
    pinMode(BOOT, INPUT_PULLUP);

    driver.init();
    //CHOPCONF settings
    driver.blankTime(24);
    driver.chopperMode(1);
    driver.hystEnd(5);
    driver.hystStart(6);
    driver.hystDecrement(32);
    driver.slowDecayTime(5);

    // DRVCONF settings
    driver.readMode(1);
    driver.senseResScale(0);
    driver.stepMode(0);
    driver.motorShortTimer(2);
    driver.enableDetectGND(0);
    driver.slopeControlLow(2);
    driver.slopeControlHigh(2);

    //SMARTEN settings
    driver.coilLowerThreshold(8);
    driver.coilIncrementSize(8);
    driver.coilUpperThreshold(8);
    driver.coilDecrementSpd(8);
    driver.minCoilCurrent(0);

    //SGCSCONF settings
    driver.currentScale(15); //do not increase, if you do, the damn chip will get hot and de solder itself.
    //lol, turn fan on if you're going to do max current
    driver.stallGrdThresh(0);
    driver.filterMode(0);
    driver.setMicroStep(motor_config.microsteps);

    //Write the constructed bitfields to the driver
    driver.pushCommands();

    motor_config.real_position[0] = 0;
    motor_config.real_position[1] = 0;

    motor_config.target_position[0] = 0;
    motor_config.target_position[1] = 0;

        // setup for ESPNow
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");
    esp_now_register_recv_cb(on_data_recv);

    // get MAC Address
    // delay(3000);
    // Serial.println(WiFi.macAddress());
    // delay(3000);
}

bool validate(const std::string& number) {
    if(number.size() > 8 || number.size() == 0) return false;
    if(number[0] != '-' && (number[0] < '0' || number[0] > '9')) {
        return false;
    }
    for(int i = 1; i < number.size(); i++){
        if(number[i] < '0' || number[i] > '9') {
            return false;
        }
    }
    return true;
}

int i = 0;
std::string input_buff = "";
uint64_t last_time = 0;
void loop() {
    if(Serial.available() && i < 2) {
        char v = Serial.read();
        Serial.println(v);
        if ((v >= '0' && v <= '9') || v == '-') {
            input_buff += v;
        } else {
            if(!validate(input_buff)){
                Serial.print("Bad number ");
                Serial.println(input_buff.c_str());
            } else if (v == 'n') {
                i = (i+1)%2;
                Serial.print("Channel ");
                Serial.print(i);
                Serial.println(" Selected");
            } else if(v == 'p') {
                motor_config.target_position[i] = std::stoi(input_buff);
                Serial.print("Position Set For Channel ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(motor_config.target_position[i]);
            } else if(v == 'a') {
                motor_config.max_accel[i] = std::stoi(input_buff);
                Serial.print("Accel Set For Channel ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(motor_config.max_accel[i]);
            } else if(v == 's') {
                motor_config.max_speed[i] = std::stoi(input_buff);
                Serial.print("Speed Set For Channel ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(motor_config.max_speed[i]);
            }
            input_buff = "";
        }
    }
    move_to_position();
}
