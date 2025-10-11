// #pragma once
// #include "nhatbot_firmware/zlac_sdk.hpp"
// #include <memory>
// #include <mutex>

// namespace nhatbot_interface {

// class DriverManager {
// public:
//     static DriverManager& instance() 
//     {
//         static DriverManager inst;
//         return inst;
//     }

//     zlac_modbus::ZLAC8015D_SDK& driver() 
//     {   
//         return zlac_driver_;
//     }

//     bool init(const std::string& port, int baudrate, int slave_id) 
//     {
//         std::lock_guard<std::mutex> lock(mutex_);
//         if (!initialized_) 
//         {
//             if (!zlac_driver_.openDriver(port, baudrate, slave_id)) 
//             {
//                 return false;
//             }
//             zlac_driver_.enableMotor();
//             zlac_driver_.setMode(zlac_modbus::ControlMode::SPEED_RPM);
//             zlac_driver_.setAccelTime(500, 500);
//             zlac_driver_.setDecelTime(600, 600);
//             initialized_ = true;
//         }
//         return true;
//     }

//     void shutdown() 
//     {
//         std::lock_guard<std::mutex> lock(mutex_);
//         if (initialized_) {
//             zlac_driver_.setRPM(0, 0);
//             zlac_driver_.disableMotor();
//             zlac_driver_.closeDriver();
//             initialized_ = false;
//         }
//     }

//     bool isInit() const { return initialized_; }

// private:
//     DriverManager() = default;
//     ~DriverManager() = default;
//     DriverManager(const DriverManager&) = delete;
//     DriverManager& operator=(const DriverManager&) = delete;

//     zlac_modbus::ZLAC8015D_SDK zlac_driver_; 
//     bool initialized_ = false;
//     mutable std::mutex mutex_;
// };

// } // namespace nhatbot_interface




#pragma once
#include "nhatbot_firmware/zlac_sdk.hpp"
#include <memory>
#include <mutex>
#include <string>

namespace nhatbot_interface {

class DriverManager {
public:
    static DriverManager& instance() 
    {
        static DriverManager inst;
        return inst;
    }

    // ==========================
    //   Khởi tạo / shutdown
    // ==========================
    bool init(const std::string& port, int baudrate, int slave_id) 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) 
        {
            if (!zlac_driver_.openDriver(port, baudrate, slave_id)) 
            {
                return false;
            }
            zlac_driver_.enableMotor();
            zlac_driver_.setMode(zlac_modbus::ControlMode::SPEED_RPM);
            zlac_driver_.setAccelTime(500, 500);
            zlac_driver_.setDecelTime(600, 600);
            initialized_ = true;
        }
        return true;
    }

    void shutdown() 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            zlac_driver_.setRPM(0, 0);
            zlac_driver_.disableMotor();
            zlac_driver_.closeDriver();
            initialized_ = false;
        }
    }

    bool isInit() const { return initialized_; }

    // ==========================
    //   Wrapper an toàn
    // ==========================
    double getBatteryVoltage() {
        std::lock_guard<std::mutex> lock(mutex_);
        return zlac_driver_.get_battery_voltage();
    }

    double getDriverTemperature() {
        std::lock_guard<std::mutex> lock(mutex_);
        return zlac_driver_.get_driver_temperature();
    }

    double getMotorTemperature() {
        std::lock_guard<std::mutex> lock(mutex_);
        return zlac_driver_.get_motor_temperature();
    }

    std::pair<int, int> getMotorFaults() {
        std::lock_guard<std::mutex> lock(mutex_);
        return zlac_driver_.get_motor_faults();
    }

    bool clearPosition(zlac_modbus::ClearPos pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        return zlac_driver_.clear_position(pos);
    }

    void setRPM(double left, double right) {
        std::lock_guard<std::mutex> lock(mutex_);
        zlac_driver_.setRPM(left, right);
    }

    std::pair<double,double> getWheelsTravelled() {
    std::lock_guard<std::mutex> lock(mutex_);
    return zlac_driver_.get_wheels_travelled();
}

std::pair<double,double> getWheelVelocities() {
    std::lock_guard<std::mutex> lock(mutex_);
    return zlac_driver_.get_wheel_angular_velocities();
}


    // ==========================
    //   Giữ lại API cũ (tương thích ngược)
    // ==========================
    zlac_modbus::ZLAC8015D_SDK& driver() {
        return zlac_driver_;
    }

private:
    DriverManager() = default;
    ~DriverManager() = default;
    DriverManager(const DriverManager&) = delete;
    DriverManager& operator=(const DriverManager&) = delete;

    zlac_modbus::ZLAC8015D_SDK zlac_driver_; 
    bool initialized_ = false;
    mutable std::mutex mutex_;
};

} // namespace nhatbot_interface
