#include "nhatbot_firmware/zlac_sdk.hpp"
#include "nhatbot_firmware/modbus_register.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

zlac_modbus::ZLAC8015D_SDK zlac_driver;  // global object

void signalHandler(int signum)
{
    std::cout << "\n⚠️ Caught signal " << signum << ", stopping motor...\n";
    zlac_driver.setRPM(0, 0);       
    zlac_driver.disableMotor();    
    zlac_driver.closeDriver();      
    exit(signum);
}

int main()
{
    // Gắn handler cho Ctrl+C
    std::signal(SIGINT, signalHandler);

    if (!zlac_driver.openDriver("/dev/ttyUSB0", 115200, 1)) 
    {
        std::cerr << "❌ Không thể kết nối driver\n";
        return -1;
    }

    zlac_driver.enableMotor();
    zlac_driver.setAccelTime(1000, 1000);
    zlac_driver.setDecelTime(1000, 1000);
    zlac_driver.setMode(zlac_modbus::ControlMode::SPEED_RPM);
    zlac_driver.clear_position(zlac_modbus::ClearPos::BOTH);
    bool check = true;

    while (true) {


        // std::this_thread::sleep_for(std::chrono::seconds(2));

        // zlac_driver.setRPM(70, 70);

        auto travelled = zlac_driver.get_wheels_travelled();
        if(!std::isnan(travelled.first) && !std::isnan(travelled.second))
        {
            std::cout << "Left wheel traveled: " << travelled.first << " rad, Right wheel travelled: " << travelled.second << " rad" << std::endl;
        }
        else
        {
            std::cerr << "Failed to calculate wheel travelled distance" << std::endl; 
        }
        auto rpm = zlac_driver.get_rpm();
        std::cout << "FB RPM -> Left: " << rpm.first << ", right: "<< rpm.second << std::endl;


        auto [wL, wR]    = zlac_driver.get_wheel_angular_velocities();
        std::cout << "Angular Vel (rad/s): L=" << wL << ", R=" << wR << std::endl;

        // auto ticks = zlac_driver.get_wheels_tick();
        // std::cout << "Encoder ticks -> Left: " << ticks.first << ", Right: " << ticks.second << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));



 /*

        float voltage = zlac_driver.get_battery_voltage();
        std::cout << "Battery Voltage: " << voltage << " V" << std::endl;



        std::string brake_state = zlac_driver.get_brake_state();
        std::cout << "Brake state: " << brake_state << std::endl;

        int motor_temp = zlac_driver.get_motor_temperature();
        float driver_temp = zlac_driver.get_driver_temperature();

        std::cout << "Motor Temp: " << motor_temp << " °C" << std::endl;
        std::cout << "Driver Temp: " << driver_temp << " °C" << std::endl;


        auto status = zlac_driver.get_motor_status();

        std::cout << "Connected: " << status.connected << std::endl;
        std::cout << "Motor Enabled: " << status.enabled << std::endl;
        std::cout << "Mode: " << static_cast<int>(status.mode) << std::endl;
        std::cout << "Left RPM: " << status.rpm.first << " , Right RPM: " << status.rpm.second << std::endl;
        std::cout << "Left Position (rad): " << status.position.first << " , Right Position (rad): " << status.position.second << std::endl;
        std::cout << "Motor Temperature: " << status.motor_temperature << " °C" << std::endl;
        std::cout << "Driver Temperature: " << status.driver_temperature << " °C" << std::endl;
        std::cout << "Brake State: " << status.brake_state << std::endl;
        std::cout << "Battery Voltage: " << status.battery_voltage << " V" << std::endl;
        std::cout << "Faults -> Left: " << status.faults.first << " , Right: " << status.faults.second << std::endl;



        // std::this_thread::sleep_for(std::chrono::seconds(1));

        

        // zlac_driver.setRPM(0, 0);
        // std::this_thread::sleep_for(std::chrono::seconds(1));*/
    }

    return 0;
}
