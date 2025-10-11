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

        // if (!zlac_driver.openDriver(port_, baudrate_, 1)) 
        // {
        //     RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to connect Modbus driver");
        //     return CallbackReturn::FAILURE;
        // }
        // zlac_driver.enableMotor();
        // zlac_driver.setAccelTime(500, 500);
        // zlac_driver.setDecelTime(600, 600);
        // zlac_driver.setMode(zlac_modbus::ControlMode::SPEED_RPM);

        auto& drv_mgr = DriverManager::instance();
        if (!drv_mgr.init(port_, baudrate_, slave_id_)) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to init Modbus driver");
            return CallbackReturn::FAILURE;
        }


        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }





    // CallbackReturn NhatbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Stopping robot hardware ...");

    //     try
    //     {
    //         if(zlac_driver.isOpen())
    //         {
    //             zlac_driver.setRPM(0, 0);
    //             zlac_driver.disableMotor();
    //             zlac_driver.closeDriver();
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "Exception while deactivating: " << e.what());
    //         return CallbackReturn::FAILURE;
    //     }

    //     return CallbackReturn::SUCCESS;
        

    // }



    CallbackReturn NhatbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Stopping robot hardware ...");

    try
    {
        auto& drv_mgr = DriverManager::instance();
        if (drv_mgr.isInit()) {
            drv_mgr.shutdown();
            RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "✅ Driver shutdown completed.");
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"),
                            "Exception while deactivating: " << e.what());
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type NhatbotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    try {
        // Đọc qua DriverManager
        auto [posL, posR] = DriverManager::instance().getWheelsTravelled();
        position_states_[0] = posL;
        position_states_[1] = posR;

        auto [wL, wR] = DriverManager::instance().getWheelVelocities();
        velocity_states_[0] = wL;
        velocity_states_[1] = wR;

        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "❌ Exception in read(): " << e.what());
        return hardware_interface::return_type::ERROR;
    }
}

hardware_interface::return_type NhatbotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    try {
        int left_rpm  = static_cast<int>(velocity_commands_[0] * 60.0 / (2.0 * M_PI));
        int right_rpm = static_cast<int>(velocity_commands_[1] * 60.0 / (2.0 * M_PI));
        //std::cout << "left RPM: "<< left_rpm << "right_rpm: " << right_rpm;



        DriverManager::instance().setRPM(left_rpm, right_rpm);

        return hardware_interface::return_type::OK;
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "❌ Exception in write(): " << e.what());
        return hardware_interface::return_type::ERROR;
    }
}



} // namespace: nhatbot_firmware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_interface::NhatbotInterface, hardware_interface::SystemInterface)
