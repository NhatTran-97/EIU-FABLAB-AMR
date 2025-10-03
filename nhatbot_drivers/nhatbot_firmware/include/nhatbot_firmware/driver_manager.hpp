#pragma once
#include "nhatbot_firmware/zlac_sdk.hpp"
#include <memory>
#include <mutex>

namespace nhatbot_interface {

class DriverManager {
public:
    static DriverManager& instance() 
    {
        static DriverManager inst;
        return inst;
    }

    zlac_modbus::ZLAC8015D_SDK& driver() 
    {   
        return zlac_driver_;
    }

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
