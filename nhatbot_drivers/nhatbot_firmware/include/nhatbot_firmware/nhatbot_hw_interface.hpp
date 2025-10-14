#ifndef NHATBOT_INTERFACE_HPP
#define NHATBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <vector>
#include <string>


namespace nhatbot_interface 
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class NhatbotInterface : public hardware_interface::SystemInterface
    {
        public:
            // NhatbotInterface();
            // virtual ~NhatbotInterface();

            NhatbotInterface() = default;
            ~NhatbotInterface() override = default;

            CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
            CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
            hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
        
        private:
            bool modbus_ready_ = false; 
            std::string port_;   
            int baudrate_{115200};
            int slave_id_{1};   

            rclcpp::Time last_run_;

            std::vector<double> velocity_commands_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;

            // zlac_modbus::ZLAC8015D_SDK zlac_driver;

            

    };

} // namespace nhatbot_firmware


#endif 