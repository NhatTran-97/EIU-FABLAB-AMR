#include "nhatbot_firmware/zlac_sdk.hpp"
#include "nhatbot_firmware/nhatbot_hw_interface.hpp"
#include "nhatbot_firmware/driver_manager.hpp"



namespace nhatbot_interface
{
    // NhatbotInterface::NhatbotInterface()
    // {

    // }

    // NhatbotInterface::~NhatbotInterface()
    // {
    //         // Destructor can also call cleanup logic
    //     // if (zlac_driver.isOpen()) {
    //     //     zlac_driver.setRPM(0, 0);    // Dừng motor nếu đang chạy
    //     //     zlac_driver.disableMotor();   // Tắt motor
    //     //     zlac_driver.closeDriver();    // Đóng kết nối Modbus
    //     //     RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "✅ Driver closed during cleanup.");
    //     // }

    // }


    CallbackReturn NhatbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info_)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info_);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
            baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
            slave_id_ = std::stoi(info_.hardware_parameters.at("slave_id"));
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "No Serial Port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

   

        velocity_commands_.resize(info_.joints.size(), 0.0);
        position_states_.resize(info_.joints.size(), 0.0);
        velocity_states_.resize(info_.joints.size(), 0.0);
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }



    std::vector<hardware_interface::StateInterface> NhatbotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

            // Provide only a position Interafce
            for (size_t i = 0; i < info_.joints.size(); i++)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));


                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
            }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> NhatbotInterface::export_command_interfaces()
    {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Provide only a velocity Interafce
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
    return command_interfaces;
    }

    CallbackReturn NhatbotInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Starting robot hardware ...");

        // Reset commands and states
        velocity_commands_.assign(2, 0.0);
        position_states_.assign(2, 0.0);
        velocity_states_.assign(2, 0.0);

        /*For direcly control*/

        // if (!zlac_driver.openDriver(port_, baudrate_, slave_id_)) 
        // {
        //     RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to connect Modbus driver");
        //     modbus_ready_ = false;
        //     return CallbackReturn::FAILURE;
        // }
        // zlac_driver.enableMotor();
        // zlac_driver.setAccelTime(200, 200);
        // zlac_driver.setDecelTime(200, 200);
        // zlac_driver.setMode(zlac_modbus::ControlMode::SPEED_RPM);
        

        /*For Driver Manager*/

        auto& drv_mgr = DriverManager::instance();

        if (!drv_mgr.init(port_, baudrate_, slave_id_)) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to init Modbus driver");
            return CallbackReturn::FAILURE;
        }

        modbus_ready_ = true;


        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }




    /*For direcly control*/

    CallbackReturn NhatbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Stopping robot hardware ...");

        try
        {
            /*For directly control*/
            // if(zlac_driver.isOpen())
            // {
            //     zlac_driver.setRPM(0, 0);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
            //     zlac_driver.disableMotor();
            //     zlac_driver.closeDriver();
            // }

            /*For Driver manager*/

            auto& drv_mgr = DriverManager::instance();
            if (drv_mgr.isInit()) {
                drv_mgr.shutdown();
                RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "✅ Driver shutdown completed.");
            }

            modbus_ready_ = false;  
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "Exception while deactivating: " << e.what());
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
        

    }





hardware_interface::return_type NhatbotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    try {

        // if (!modbus_ready_ || !zlac_driver.isOpen()) 
        // {
        //     RCLCPP_WARN(rclcpp::get_logger("NhatbotInterface"),"⚠️ Modbus not ready, skip read/write");

        //     return hardware_interface::return_type::OK;
        // }




        /*For Driver manager*/
        auto [wL, wR] = DriverManager::instance().getWheelVelocities();
        auto [posL, posR] = DriverManager::instance().getWheelsTravelled();



     /*For directly control*/
        // auto [posL, posR] = zlac_driver.get_wheels_travelled();
        // auto [wL, wR] = zlac_driver.get_wheel_angular_velocities();
 

        position_states_[0] = posL;
        position_states_[1] = posR;

        velocity_states_[0] = wL;
        velocity_states_[1] = wR;


        

        // RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "velocity L=%.2f  R=%.2f", wL, wR);
        // RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "position L=%.2f  R=%.2f", posL, posR);

        // RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"),
        // "Δpos: L=%.3f R=%.3f  |  Δvel: L=%.3f R=%.3f",
        // position_states_[0], position_states_[1],
        // velocity_states_[0], velocity_states_[1]);



        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) 
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "❌ Exception in read(): " << e.what());
        return hardware_interface::return_type::ERROR;
    }
}


hardware_interface::return_type NhatbotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // if (!modbus_ready_ || !zlac_driver.isOpen()) 
    // {
   
    //     RCLCPP_WARN(rclcpp::get_logger("NhatbotInterface"),"⚠️ Modbus not ready, skip read/write");
    //     return hardware_interface::return_type::OK;
    // }

    try {
        int left_rpm  = static_cast<int>(velocity_commands_[0] * 60.0 / (2.0 * M_PI));
        int right_rpm = static_cast<int>(velocity_commands_[1] * 60.0 / (2.0 * M_PI));
         RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"),"CMD L=%d  R=%d", left_rpm, right_rpm);

        /*For Driver manager*/
        DriverManager::instance().setRPM(left_rpm, right_rpm);

        /*For directly control*/

        // if (std::abs(left_rpm) < 2 && std::abs(right_rpm) < 2)
        // {
        //     zlac_driver.setRPM(0, 0);
        // }
        // else
        // {
          
        //     // zlac_driver.setRPM(left_rpm, right_rpm);
        //     zlac_driver.setRPM(left_rpm, right_rpm);
        //  //   RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"),"CMD L=%d  R=%d", left_rpm, right_rpm);
        // }
        return hardware_interface::return_type::OK;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"),"❌ Exception in write(): " << e.what());
        return hardware_interface::return_type::ERROR;
    }
}



// hardware_interface::return_type NhatbotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//     try {
//         int left_rpm  = static_cast<int>(velocity_commands_[0] * 60.0 / (2.0 * M_PI));
//         int right_rpm = static_cast<int>(velocity_commands_[1] * 60.0 / (2.0 * M_PI));

//         if (std::abs(left_rpm) < 2 && std::abs(right_rpm) < 2) {
//             // Khi tốc độ quá nhỏ → phanh chủ động
//             DriverManager::instance().setRPM(0, 0);
//             DriverManager::instance().driver().disableMotor();   // <--- thêm dòng này
//         } else {
//             DriverManager::instance().driver().enableMotor();    // đảm bảo motor bật khi có lệnh di chuyển
//             DriverManager::instance().setRPM(left_rpm, right_rpm);
//         }

//         RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"),
//                     "CMD L=%d  R=%d", left_rpm, right_rpm);

//         return hardware_interface::return_type::OK;
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"),
//                             "❌ Exception in write(): " << e.what());
//         return hardware_interface::return_type::ERROR;
//     }
// }







} // namespace: nhatbot_firmware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_interface::NhatbotInterface, hardware_interface::SystemInterface)
