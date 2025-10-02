#include "nhatbot_firmware/zlac_sdk.hpp"
#include "nhatbot_firmware/nhatbot_hw_interface.hpp"



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

        if (!zlac_driver.openDriver(port_, baudrate_, 1)) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to connect Modbus driver");
            return CallbackReturn::FAILURE;
        }
        zlac_driver.enableMotor();
        zlac_driver.setMode(zlac_modbus::ControlMode::SPEED_RPM);


        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }





    CallbackReturn NhatbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Stopping robot hardware ...");

        try
        {
            if(zlac_driver.isOpen())
            {
                zlac_driver.setRPM(0, 0);
                zlac_driver.disableMotor();
                zlac_driver.closeDriver();
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotInterface"), "Exception while deactivating: " << e.what());
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
        

    }



    // CallbackReturn NhatbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    // {
    //     if (modbus_) 
    //     {
    //         modbus_close(modbus_.get());
    //         modbus_.reset();
    //         RCLCPP_INFO(rclcpp::get_logger("NhatbotInterface"), "Modbus disconnected");
    //     }
    //     return CallbackReturn::SUCCESS;
    // }




    hardware_interface::return_type NhatbotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
    // Interpret the string
    // if(arduino_.IsDataAvailable())
    // {
    //     auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    //     std::string message;
    //     arduino_.ReadLine(message);
    //     std::stringstream ss(message);
    //     std::string res;
    //     int multiplier = 1;
    //     while(std::getline(ss, res, ','))
    //     {
    //     multiplier = res.at(1) == 'p' ? 1 : -1;

    //     if(res.at(0) == 'r')
    //     {
    //         velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
    //         position_states_.at(0) += velocity_states_.at(0) * dt;
    //     }
    //     else if(res.at(0) == 'l')
    //     {
    //         velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
    //         position_states_.at(1) += velocity_states_.at(1) * dt;
    //     }
    //     }
    //     last_run_ = rclcpp::Clock().now();
    // }


    if(!zlac_driver.isOpen())
    {
        return hardware_interface::return_type::ERROR;
    }

    if (!zlac_driver.isDataAvailable(100))
    { 
        return hardware_interface::return_type::OK; 
    }
    // auto [rpmL, rpmR] = zlac_driver.get_rpm();
    // if(rpmL == 0.0f || rpmR == 0.0f)
    // {
    //     RCLCPP_WARN(rclcpp::get_logger("NhatbotInterface"), "❌ Unable to get RPM from motor.");
    //     return hardware_interface::return_type::ERROR;

    // }
    auto [posL, posR] = zlac_driver.get_wheels_travelled();
    position_states_[0] = posL;
    position_states_[1] = posR;
    return hardware_interface::return_type::OK;
    return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NhatbotInterface::write(const rclcpp::Time &,
                                                            const rclcpp::Duration &)
    {
        if(!zlac_driver.isOpen())
        {
            RCLCPP_ERROR(rclcpp::get_logger("NhatbotInterface"), "❌ Modbus is not connected!");
            return hardware_interface::return_type::ERROR;

        }
        int left_rpm = static_cast<int>(velocity_commands_[0]);  // Convert m/s to RPM
        int right_rpm = static_cast<int>(velocity_commands_[1]);

        if (!zlac_driver.setRPM(left_rpm, right_rpm))
        {
            RCLCPP_ERROR(rclcpp::get_logger("NhatbotInterface"), "❌ Failed to set RPM for motors");
            return hardware_interface::return_type::ERROR;

        }


    return hardware_interface::return_type::OK;
    }




} // namespace: nhatbot_firmware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_interface::NhatbotInterface, hardware_interface::SystemInterface)
