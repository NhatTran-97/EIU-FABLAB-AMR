#ifndef ZLAC_SDK_HPP
#define ZLAC_SDK_HPP

#include <modbus/modbus.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <array>
#include "nhatbot_firmware/modbus_register.hpp"

#include <memory>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>  // cho M_PI
#include <limits> // cho std::numeric_limits


namespace zlac_modbus
{
    enum class ControlMode : uint8_t 
    {
        RELATIVE_POSITION = 1,
        ABSOLUTE_POSITION = 2,
        SPEED_RPM         = 3

    };

    struct MotorStatus {
        bool connected;
        bool enabled;
        ControlMode mode;
        
        std::pair<float, float> rpm;             // Left and Right RPM
        std::pair<float, float> linear_velocity; // Left and Right Linear Velocity (m/s)
        std::pair<float, float> position;       // Wheel position (radians)
        std::pair<int, int> faults;             // Left and Right Motor Faults
        float motor_temperature;                // Motor temperature (°C)
        float driver_temperature;               // Driver temperature (°C)
        std::string brake_state;                // Brake state ("Open" or "Closed")
        float battery_voltage;                  // Battery voltage (V)
        };



    class ZLAC8015D_SDK
    {
    private:
        using ModbusPtr = std::unique_ptr<modbus_t, decltype(&modbus_free)>;
        ModbusPtr ctx_{nullptr, &modbus_free};
        std::string port_;
        int baudrate_{115200};
        int slave_id_{1};
        int max_rpm_ = 100;
        float cpr_ = 4096;
        float travel_in_one_rev_ = 0.336;
        float R_Wheel_ = 0.0535;


        


    public:


        bool openDriver(const std::string& port, int baudrate = 115200, int slave = 1);
        void closeDriver();
        bool is_motor_enabled();
        bool isOpen() const;
        bool isDataAvailable(int timeout_ms) const;
        bool autoReconnect(int retry_interval_ms = 1000);
        bool healthCheck();
        bool readRegisters(int addr, int num, std::vector<uint16_t>& dest);
        bool writeRegister(int addr, uint16_t value);
        bool writeRegisters(int addr, const std::vector<uint16_t>& values);
        bool enableMotor();
        bool disableMotor();
        bool setMode(ControlMode mode);
        bool setDecelTime(int left_ms, int right_ms);
        bool setAccelTime(int left_ms, int right_ms);
        bool setRPM(int left_rpm, int right_rpm);
        std::pair<float, float> get_wheels_travelled();
        int32_t join_u16_to_s32(uint16_t hi, uint16_t lo);
        std::vector<uint16_t> modbus_fail_read_handler(uint16_t addr, int num_regs, int max_retries = 3, double delay=0.1);
        std::pair<float, float> get_rpm();
        std::pair<int32_t, int32_t> get_wheels_tick(); 
        float get_battery_voltage();
        ControlMode getMode();
        std::pair<int, int> get_motor_faults();
        float get_motor_temperature();
        
        float get_driver_temperature();
        std::string get_brake_state();
        bool stop_motor_emergency();
        bool clear_alarm();
        bool clear_position(int pos);
        MotorStatus get_motor_status();
        std::pair<float, float> get_linear_velocities();
        float rpm_to_linear(float rpm);



    };

}


#endif // ZLAC_SDK
