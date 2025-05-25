#include "zlac8015d_driver/zlac8015d_driver.hpp"

#include <modbus/modbus.h>
#include <unistd.h>
#include <ctime>
#include <vector>
#define RETURN_IF_NULL(x) if (!(x)) return {};


// Địa chỉ thanh ghi Modbus của ZLAC8015D
#define REG_CONTROL 0x200E
#define REG_CMD_RPM_LEFT 0x2088
#define REG_CMD_RPM_RIGHT 0x2089
#define REG_FB_RPM_LEFT 0x20AB
#define REG_FB_RPM_RIGHT 0x20AC
#define REG_FAULT_LEFT 0x20A5
#define REG_FAULT_RIGHT 0x20A6
#define REG_WHEEL_POS 0x20A7
#define REG_BATTERY_VOLTAGE 0x20A1
#define REG_MOTOR_TEMP 0x20A4
#define REG_DRIVER_TEMP 0x20B0

#define L_ACL_TIME 0x2080
#define R_ACL_TIME 0x2081
#define CLR_FB_POS 0x2005

// Giá trị điều khiển
#define CMD_ENABLE 0x08
#define CMD_DISABLE 0x07
#define CMD_EMERGENCY_STOP 0x05

#define OPR_MODE  0x200D

ZLAC8015D_API::ZLAC8015D_API(const NodeParameters& params) : slave_id(1), 
                                                            was_connected(false),
                                                            reconnect_interval(5),
                                                            last_attempt_time(std::time(nullptr)),
                                                            ctx(nullptr, &modbus_free),
                                                             params_(params)
{

    modbus_t* raw_ctx = modbus_new_rtu(params_.modbus_port.c_str(), params_.modbus_baudrate, 'N', 8, 1);

    if (!raw_ctx)
    {
        std::cerr << "❌ Không thể khởi tạo Modbus RTU!\n";
        return;
    }
    ctx.reset(raw_ctx);
    modbus_set_slave(ctx.get(), slave_id);
    modbus_set_response_timeout(ctx.get(), 1, 0); // 1 giây timeout

    if (modbus_connect(ctx.get()) == -1)
    {
        std::cerr << "⚠️ Kết nối Modbus thất bại!\n";
        ctx.reset(); 
    }
    else
    {
        std::cout << "✅ Kết nối Modbus thành công!\n";
        was_connected = true;
    }


}


ZLAC8015D_API::~ZLAC8015D_API() 
{
    // this->disable_motor();
    if (ctx) {
        modbus_close(ctx.get());  // ✅ Đóng kết nối (file descriptor)
        ctx.reset();   // ✅ Giải phóng bộ nhớ, gọi modbus_free   
        // modbus_free sẽ được gọi tự động bởi unique_ptr
    }
}


bool ZLAC8015D_API::isConnected() const
{
    if (!ctx) return false;

    int fd = modbus_get_socket(ctx.get());
    return was_connected && fd >= 0;
}




bool ZLAC8015D_API::checkConnection()
{
    if (!ctx)
        return false;
    std::vector<uint16_t> result = readRegisters(REG_FAULT_LEFT, 1);
    return !result.empty();
}
bool ZLAC8015D_API::reconnect()
{
    //Đóng kết nối cũ an toàn
    if (ctx) {
        modbus_close(ctx.get());
        ctx.reset();  // cleanup/ // ✅ giải phóng bộ nhớ
    }

    std::cout << "🔄 Thử kết nối lại thiết bị ZLAC..." << std::endl;

    //Tạo context mới

    // Thiết lập lại cấu hình
    // modbus_t* raw_ctx = modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1);
    modbus_t* raw_ctx = modbus_new_rtu(params_.modbus_port.c_str(), params_.modbus_baudrate, 'N', 8, 1);
    if(params_.modbus_port.empty())
    {
        std::cerr << "❌ Cổng serial chưa được cấu hình.\n";
        return false;
    }
    if (!raw_ctx) {
        std::cerr << "❌ Không tạo được context mới!\n";
        return false;
    }
    //Kết nối lại
    ctx.reset(raw_ctx);
    modbus_set_slave(ctx.get(), slave_id);
    modbus_set_response_timeout(ctx.get(), 1, 0);

    if (modbus_connect(ctx.get()) == -1) {
        std::cerr << "❌ Kết nối lại thất bại\n";
        ctx.reset();
        was_connected = false;
        return false;
    }

    std::cout << "✅ Đã kết nối lại thành công!\n";
    was_connected = true;
    return true;
}


void ZLAC8015D_API::tryReconnectEvery(int interval_ms)
{
    static std::chrono::steady_clock::time_point last_attempt_time = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt_time).count();

    if (isConnected()) {
        return; // Đã kết nối OK, không cần reconnect
    }

    if (elapsed_ms >= interval_ms) {
        std::cout << "🔁 Mất kết nối, thử reconnect...\n";
        reconnect();
        last_attempt_time = now;
    }
}



/////////////////////////Modbus/////////////////////////

std::vector<uint16_t> ZLAC8015D_API::readRegisters(int addr, int num) const
{
    /*
    🔍 “Nếu ctx chưa được khởi tạo (null), thì không thể đọc thanh ghi → trả về vector rỗng để báo lỗi nhẹ nhàng.”
    */
   RETURN_IF_NULL(ctx);
   if (num <= 0) return {};

    
    std::vector<uint16_t> values(num);
    if (modbus_read_registers(ctx.get(), addr, num, values.data()) == num)
    {
        return values;
    }

    return {};
}

bool ZLAC8015D_API::writeRegister(int addr, uint16_t value)
{
    RETURN_IF_NULL(ctx);
    return modbus_write_register(ctx.get(), addr, value) != -1;
}


bool ZLAC8015D_API::writeRegisters(int addr, const std::vector<uint16_t> &values)
{
    RETURN_IF_NULL(ctx);
    if (values.empty())
    return false;

    return modbus_write_registers(ctx.get(), addr, values.size(), values.data()) != -1;
}




bool ZLAC8015D_API::enableMotor()
{
    return writeRegister(REG_CONTROL, CMD_ENABLE);
}

bool ZLAC8015D_API::disableMotor()
{
    return writeRegister(REG_CONTROL, CMD_DISABLE);
}

bool  ZLAC8015D_API::setDecelTime(int left_ms, int right_ms)
{

    std::vector<uint16_t> values = {static_cast<uint16_t>(left_ms),static_cast<uint16_t>(right_ms)};
    return writeRegisters(R_ACL_TIME, values);
}

bool ZLAC8015D_API::setAccelTime(int left_ms, int right_ms)
{
    std::vector<uint16_t> values = {static_cast<uint16_t>(left_ms),static_cast<uint16_t>(right_ms)};
    return writeRegisters(L_ACL_TIME, values);
}

bool ZLAC8015D_API::setMode(uint8_t mode)
{
    if(mode == 1) 
    {
        std::cout << "set relative position control" << std::endl;
    }
    else if(mode == 2) 
    {
        std::cout << "Set absolute position control" << std::endl;
    }
    else if(mode == 3)
    {
        std::cout << "Set speed rpm control" << std::endl;
    }
    else 
    {
        std::cout << "set only 1,2,3" << std::endl;return 0;
    }
    return writeRegister(OPR_MODE,mode);

}





bool ZLAC8015D_API::emergencyStop()
{
    return writeRegister(REG_CONTROL, CMD_EMERGENCY_STOP);
}

bool ZLAC8015D_API::setRPM(int left_rpm, int right_rpm)
{
   std::cout << "left: " << left_rpm << " right: " << right_rpm << std::endl;


    int16_t left = std::clamp(left_rpm, -params_.max_rpm, params_.max_rpm);
    int16_t right = std::clamp(right_rpm, -params_.max_rpm, params_.max_rpm);

    return writeRegisters(REG_CMD_RPM_LEFT, {static_cast<uint16_t>(left), 
                                            static_cast<uint16_t>(right)});
}



std::pair<int, int> ZLAC8015D_API::getRPM() const
{
    std::vector<uint16_t> regs = readRegisters(REG_FB_RPM_LEFT, 2);
    if (regs.size() < 2)
        return {0, 0};

    int16_t left = static_cast<int16_t>(regs[0]);
    int16_t right = static_cast<int16_t>(regs[1]);

    return {left / 10, right / 10};
}




std::pair<uint16_t, uint16_t> ZLAC8015D_API::getMotorFaults()
{
    std::vector<uint16_t> regs = readRegisters(REG_FAULT_LEFT, 2);
    return {regs.size() >= 2 ? regs[0] : 0, regs.size() >= 2 ? regs[1] : 0};
}

std::pair<double, double> ZLAC8015D_API::getWheelsTravelled()
{
    std::vector<uint16_t> regs = readRegisters(REG_WHEEL_POS, 4);
    if (regs.size() < 4)
        return std::make_pair(0.0, 0.0);

    return std::make_pair(
        static_cast<double>((regs[0] << 16) | regs[1]),
        static_cast<double>((regs[2] << 16) | regs[3]));
}


// std::array<double, 2> ZLAC8015D_API::getWheelsTravelled() const
// {
//     std::vector<uint16_t> regs = readRegisters(REG_WHEEL_POS, 4);
//     if (regs.size() < 4)
//         return {0.0, 0.0};

//     return {static_cast<double>((regs[0] << 16) | regs[1]),
//             static_cast<double>((regs[2] << 16) | regs[3])};
// }


std::pair<int32_t, int32_t> ZLAC8015D_API::getWheelsTick()
{
    auto travelled = getWheelsTravelled();
    return {static_cast<int32_t>(travelled.first), static_cast<int32_t>(travelled.second)};
}

int ZLAC8015D_API::getMotorTemperature()
{
    std::vector<uint16_t> regs = readRegisters(REG_MOTOR_TEMP, 1);
    return regs.empty() ? 0 : regs[0];
}

int ZLAC8015D_API::getDriverTemperature()
{
    std::vector<uint16_t> regs = readRegisters(REG_DRIVER_TEMP, 1);
    return regs.empty() ? 0 : regs[0] * 0.1;
}

double ZLAC8015D_API::getBatteryVoltage()
{
    std::vector<uint16_t> regs = readRegisters(REG_BATTERY_VOLTAGE, 1);
    return regs.empty() ? 0.0 : regs[0] * 0.01;
}

void ZLAC8015D_API::clear_position(int pos)
{
    modbus_write_register(ctx.get(), CLR_FB_POS, pos);
}
void ZLAC8015D_API::close_connect()
{
    if (ctx)
    {
        modbus_close(ctx.get()); // ✅ Lấy con trỏ "raw" từ unique_ptr
        modbus_free(ctx.get()); //  ✅ Giải phóng (gọi modbus_free tự động)
    }
}

std::pair<double, double> ZLAC8015D_API::getAngularVelocity() const
{
    // Gọi hàm getRPM() để lấy tốc độ RPM của bánh trái và bánh phải
    std::pair<int, int> rpm = getRPM();

    // Chuyển đổi RPM thành vận tốc góc (rad/s)
    double left_rpm = rpm.first * 2 * M_PI / 60.0;
    double right_rpm = rpm.second * 2 * M_PI / 60.0;

    return std::make_pair(left_rpm, right_rpm);
}