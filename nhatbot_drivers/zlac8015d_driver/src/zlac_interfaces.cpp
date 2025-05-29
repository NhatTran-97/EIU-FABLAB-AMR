#include "zlac8015d_driver/zlac_interfaces.hpp"
#include "zlac8015d_driver/zlac8015d_driver.hpp"
#include <chrono>
#include <cmath>


using namespace std::chrono_literals;

Zlac_Interfaces::Zlac_Interfaces() : Node("zlac_driver_node")
{

    params = std::make_unique<NodeParameters>(this); 
    bldcMotor = std::make_shared<ZLAC8015D_API>(*params);

    this->init();
    

    // if (bldcMotor->isConnected())
    // {
    //     bldcMotor->disableMotor();
    //     bldcMotor->setAccelTime(1000, 1000);
    //     bldcMotor->setDecelTime(1000, 1000);
    //     bldcMotor->enableMotor();
    //     bldcMotor->setRPM(0, 0);
    //     RCLCPP_INFO(this->get_logger(), "✅ Động cơ đã khởi tạo thành công!");
    // }
    // else
    // {
    //     RCLCPP_ERROR(this->get_logger(), "❌ Không thể kết nối với động cơ!");
    //     bldcMotor.reset();
    //     return;
    // }



    wheel_JointState_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(params->joint_state_topic, 10);

    wheelVelocities_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                                            params->wheel_rotation_topic, 10, std::bind(&Zlac_Interfaces::sub_Vel_Callback, this, std::placeholders::_1));

    motor_srv_ = this->create_service<std_srvs::srv::Trigger>(params->reset_encoder_service,
                                            std::bind(&Zlac_Interfaces::ResetPos_Callback, this, std::placeholders::_1, std::placeholders::_2));

    timer_JointState_ = this->create_wall_timer(100ms, std::bind(&Zlac_Interfaces::pub_JointState_Callback, this));
}



void Zlac_Interfaces::init()
{
    // params = std::make_unique<NodeParameters>(shared_from_this());
    
    
    // RCLCPP_INFO(this->get_logger(), "🔌 Using port: %s", params->modbus_port.c_str());

    if (bldcMotor->isConnected()) 
    {
        RCLCPP_INFO(this->get_logger(), "✅ Đã kết nối với động cơ ZLAC");

        if(bldcMotor->setMode(params->set_operation_mode))
        {
            RCLCPP_INFO(this->get_logger(), "✅ Thiết lập mode : %d ", params->set_operation_mode);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "❌ Không thể thiết lập mode.");
        }

        if (bldcMotor->setAccelTime(params->set_accel_time, params->set_accel_time)) 
        {
            RCLCPP_INFO(this->get_logger(), "✅ Thiết lập thời gian tăng tốc: %d ms", params->set_accel_time);
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "❌ Không thể thiết lập thời gian tăng tốc.");
        }

        if (bldcMotor->setDecelTime(params->set_decel_time, params->set_decel_time)) {
            RCLCPP_INFO(this->get_logger(), "✅ Thiết lập thời gian giảm tốc: %d ms", params->set_decel_time);
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "❌ Không thể thiết lập thời gian giảm tốc.");
        }

        if (bldcMotor->enableMotor()) {
            RCLCPP_INFO(this->get_logger(), "✅ Đã kích hoạt động cơ thành công.");
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "❌ Kích hoạt động cơ thất bại.");
        }

        // bldcMotor->setRPM(params->set_stop_rpm, params->set_stop_rpm);

        RCLCPP_INFO(this->get_logger(), "✅ Động cơ đã khởi tạo thành công!");
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "❌ Mất kết nối với động cơ ZLAC");
        bldcMotor.reset();
        return;
    }
}



Zlac_Interfaces::~Zlac_Interfaces()
{
    // exitBLDCMotor();
}



void Zlac_Interfaces::pub_JointState_Callback()
{
    if (!bldcMotor || !bldcMotor->isConnected())
    {
        return;
    }

    try
    {
        sensor_msgs::msg::JointState msg;

        // // Gọi getAngularVelocity() để lấy vận tốc góc của bánh trái và phải
        // std::pair<double, double> velocities = bldcMotor->getAngularVelocity();

        // double left_vel = velocities.first;
        // double right_vel = velocities.second;

        // // Lấy vị trí bánh xe
        // auto [left_pos, right_pos] = bldcMotor->getWheelsTravelled();

        // msg.velocity = {left_vel, right_vel};
        // msg.position = {left_pos, right_pos};
        // msg.header.stamp = this->now();

        // wheel_JointState_pub_->publish(msg);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Lỗi khi xuất JointState: %s", e.what());
    }
}

void Zlac_Interfaces::sub_Vel_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!bldcMotor || !bldcMotor->isConnected())
    {
        RCLCPP_WARN(this->get_logger(), "⚠ Mất kết nối Modbus! Đang thử kết nối lại...");
        bldcMotor->reconnect();

        if (!bldcMotor->isConnected())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Thử kết nối lại Modbus thất bại! Không gửi lệnh.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ Kết nối lại Modbus thành công! Tiếp tục điều khiển động cơ.");
    }

    if (msg->data.size() != 2)
    {
        return;
    }

    double leftWheel = msg->data[0] * (60.0 / (2 * M_PI));
    double rightWheel = msg->data[1] * (60.0 / (2 * M_PI));

    // double battery_voltage = bldcMotor->getBatteryVoltage();
    // RCLCPP_INFO(this->get_logger(), "🔋 Điện áp pin: %.2fV", battery_voltage);

    // leftWheel = std::abs(leftWheel) < 0.5 ? 0.0 : leftWheel;
    // rightWheel = std::abs(rightWheel) < 0.5 ? 0.0 : rightWheel;

    try
    {
        bldcMotor->setRPM(static_cast<int>(-leftWheel), static_cast<int>(rightWheel));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Lỗi khi điều khiển động cơ: %s", e.what());
    }
}

void Zlac_Interfaces::ResetPos_Callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
{
    // (void)request; // Tránh cảnh báo unused parameter
    // response->success = true;
    // response->message = "Encoder position reset!";
}


void Zlac_Interfaces::exitBLDCMotor()
{
    // if (bldcMotor)
    // {
    //     try
    //     {
    //         RCLCPP_INFO(this->get_logger(), "🛑 Dừng động cơ ngay lập tức!");
    //         bldcMotor->setRPM(0, 0);
    //         bldcMotor->close_connect();
    //     }
    //     catch (const std::exception &e)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "❌ Không thể dừng động cơ: %s", e.what());
    //     }
    // }
}






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zlac_Interfaces>();
    // node->init(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    // rclcpp::spin(std::make_shared<Zlac_Interfaces>());
    // rclcpp::shutdown();
    return 0;
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<Zlac_Interfaces>();

//     try
//     {
//         rclcpp::spin(node);
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR(node->get_logger(), "Lỗi trong main(): %s", e.what());
//     }

//     // 🚀 Đảm bảo động cơ được dừng khi tắt ROS node
//     if (rclcpp::ok())
//     {
//         RCLCPP_INFO(node->get_logger(), "✅ Zlac_Interfaces Node đã dừng hoàn toàn!");
//     }

//     // node->exitBLDCMotor();    // 🛑 Dừng động cơ trước khi thoát
//     // node->~Zlac_Interfaces(); // Giải phóng bộ nhớ

//     rclcpp::shutdown();
//     return 0;
// }
