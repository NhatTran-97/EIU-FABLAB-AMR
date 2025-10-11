#include "nhatbot_firmware/zlac_sdk.hpp"
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <chrono>
#include <thread>

namespace zlac_modbus
{
    bool ZLAC8015D_SDK::openDriver(const std::string& port, int baudrate, int slave)
    {
        if (ctx_) closeDriver();

        modbus_t* raw = modbus_new_rtu(port.c_str(), baudrate, 'N', 8, 1);
        if (!raw) {
            std::cerr << "❌ Cannot create Modbus context\n";
            return false;
        }

        if (modbus_set_slave(raw, slave) == -1) {
            std::cerr << "❌ Invalid slave ID\n";
            modbus_free(raw);
            return false;
        }

        modbus_set_response_timeout(raw, 1, 0);

        if (modbus_connect(raw) == -1) {
            std::cerr << "⚠️ Cannot connect to Modbus device: "
                      << modbus_strerror(errno) << "\n";
            modbus_free(raw);
            return false;
        }

        ctx_.reset(raw);
        port_ = port;
        baudrate_ = baudrate;
        slave_id_ = slave;

        std::cout << "✅ Connected to ZLAC8015D at "
                  << port << " @ " << baudrate << "\n";
        return true;
    }

    void ZLAC8015D_SDK::closeDriver()
    {
        if (ctx_) {
            modbus_close(ctx_.get());
            ctx_.reset();
            std::cout << "🔌 Modbus connection closed\n";
        }
    }

    bool ZLAC8015D_SDK::isOpen() const
    {
        return ctx_ != nullptr;
    }
    bool ZLAC8015D_SDK::is_motor_enabled()
    {
        if(!isOpen()) return false;
        std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::CONTROL_REG, 1);
        if(regs.empty()) return false;

        return (regs[0] == zlac::ENABLE);
    }

    bool ZLAC8015D_SDK::isDataAvailable(int timeout_ms) const
    {
        if (!ctx_) return false;

        int fd = modbus_get_socket(ctx_.get());
        if (fd < 0) return false;

        fd_set set;
        struct timeval timeout;

        FD_ZERO(&set);
        FD_SET(fd, &set);

        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int rv = select(fd + 1, &set, NULL, NULL, &timeout);
        return (rv > 0 && FD_ISSET(fd, &set));
    }



bool ZLAC8015D_SDK::autoReconnect(int retry_interval_ms)
{
    if (ctx_) {
        uint16_t test_reg[2];
        int rc = modbus_read_registers(ctx_.get(), 0x2103, 2, test_reg);
        if (rc == 2) {
            return true; // vẫn kết nối tốt
        }

        if (errno == EBADF || errno == EIO) {
            std::cerr << "⚠️ Lost connection, trying to reconnect...\n";
            closeDriver();
        } else {
            // lỗi do register không hợp lệ, không phải lỗi I/O
            return true;
        }
    }

    while (true) {
        if (openDriver(port_, baudrate_, slave_id_)) {
            std::cout << "🔄 Reconnected to " << port_ << "\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }
}

// bool ZLAC8015D_SDK::healthCheck()
// {
//     if (!ctx_) return false;

//     uint16_t reg;
//     int rc = modbus_read_registers(ctx_.get(), 0x0000, 1, &reg); // thử đọc 1 thanh ghi
//     if (rc == -1) {
//         std::cerr << "⚠️ Lost connection: " << modbus_strerror(errno) << "\n";
//         closeDriver();
//         return false;
//     }
//     return true;
// }

bool ZLAC8015D_SDK::healthCheck()
{
    if (!ctx_) return false;

    uint16_t reg;
    for (int i = 0; i < 3; i++) {
        int rc = modbus_read_registers(ctx_.get(), 0x0000, 1, &reg);
        if (rc != -1) return true;  // thành công
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << "⚠️ Lost connection: " << modbus_strerror(errno) << "\n";
    closeDriver();
    return false;
}

bool ZLAC8015D_SDK::readRegisters(int addr, int num, std::vector<uint16_t>& dest)
{
    if (!ctx_) return false;

    dest.resize(num);
    int rc = modbus_read_registers(ctx_.get(), addr, num, dest.data());
    if (rc == -1) {
        std::cerr << "❌ Failed to read registers at 0x" 
                  << std::hex << addr << ": " << modbus_strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool ZLAC8015D_SDK::writeRegister(int addr, uint16_t value)
{
    if (!ctx_) return false;

    int rc = modbus_write_register(ctx_.get(), addr, value);
    if (rc == -1) {
        std::cerr << "❌ Failed to write register at 0x" 
                  << std::hex << addr << ": " << modbus_strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool ZLAC8015D_SDK::writeRegisters(int addr, const std::vector<uint16_t>& values)
{
    if (!ctx_) return false;

    int rc = modbus_write_registers(ctx_.get(), addr, values.size(), values.data());
    if (rc == -1) {
        std::cerr << "❌ Failed to write registers at 0x"
                  << std::hex << addr << ": " << modbus_strerror(errno) << "\n";
        return false;
    }
    return true;
}


bool ZLAC8015D_SDK::enableMotor()
{
    return writeRegister(zlac::CONTROL_REG, zlac::ENABLE);
}

bool ZLAC8015D_SDK::disableMotor()
{
    return writeRegister(zlac::CONTROL_REG, zlac::STOP);
}

bool  ZLAC8015D_SDK::setDecelTime(int left_ms, int right_ms)
{

    std::vector<uint16_t> values = {static_cast<uint16_t>(left_ms),static_cast<uint16_t>(right_ms)};
    return writeRegisters(zlac::R_ACL_TIME, values);
}

bool ZLAC8015D_SDK::setAccelTime(int left_ms, int right_ms)
{
    std::vector<uint16_t> values = {static_cast<uint16_t>(left_ms),static_cast<uint16_t>(right_ms)};
    return writeRegisters(zlac::L_ACL_TIME, values);
}

bool ZLAC8015D_SDK::setMode(ControlMode mode)
{
    if (!ctx_) {
        std::cerr << "❌ Modbus context not open\n";
        return false;
    }

    switch (mode) {
        case ControlMode::RELATIVE_POSITION:
            std::cout << "⚙️ Set relative position control\n";
            return writeRegister(zlac::OPR_MODE, zlac::POS_REL_CONTROL);

        case ControlMode::ABSOLUTE_POSITION:
            std::cout << "⚙️ Set absolute position control\n";
            return writeRegister(zlac::OPR_MODE, zlac::POS_ABS_CONTROL);

        case ControlMode::SPEED_RPM:
            std::cout << "⚙️ Set speed RPM control\n";
            return writeRegister(zlac::OPR_MODE, zlac::VEL_CONTROL);

        default:
            std::cerr << "❌ Invalid control mode\n";
            return false;
    }
}

bool ZLAC8015D_SDK::setRPM(int left_rpm, int right_rpm)
{
    // Giới hạn RPM
    int16_t left  = std::clamp(left_rpm,  -max_rpm_, max_rpm_);
    int16_t right = std::clamp(right_rpm, -max_rpm_, max_rpm_);

    // // Đảo bên (swap)
    // int16_t tmp = left;
    // left  = right;
    // right = tmp;


    // left = -left;


    std::vector<uint16_t> values = {static_cast<uint16_t>(left),  static_cast<uint16_t>(-right)};

    return writeRegisters(zlac::L_CMD_RPM, values);
}


int32_t ZLAC8015D_SDK::join_u16_to_s32(uint16_t hi, uint16_t lo)
{
 return (static_cast<int32_t>(hi) << 16) | static_cast<int32_t>(lo);
}

std::vector<uint16_t> ZLAC8015D_SDK::modbus_fail_read_handler(uint16_t addr, int num_regs, int max_retries , double delay)
{
    std::vector<uint16_t> regs(num_regs);  // Array to store the read data
    int attempt = 0;

    while (attempt < max_retries) {
        try {
            int rc = modbus_read_registers(ctx_.get(), addr, num_regs, regs.data());

            if (rc != -1) 
            {
                return regs;  // Thành công, trả về kết quả
            }

            std::cerr << "⚠️ [Attempt " << attempt + 1 << "/" << max_retries 
                        << "] Error when reading Modbus address " << addr 
                        << ": " << modbus_strerror(errno) << std::endl;

        } catch (const std::exception& e) 
        {
            std::cerr << "❌ [Attempt " << attempt + 1 << "/" << max_retries 
                        << "] Unidentified error: " << e.what() << std::endl;
        }

        // Chờ một khoảng thời gian trước khi thử lại
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
        attempt++;
    }

    std::cerr << "❌ Unable to read Modbus address " << addr 
                << " after " << max_retries << " attempts!" << std::endl;

    return {};  // Trả về vector rỗng nếu thất bại sau max_retries lần thử
}


std::pair<float, float> ZLAC8015D_SDK::get_wheels_travelled()
{
    std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::L_FB_POS_HI, 4);
    if (regs.empty() || regs.size() < 4)
    {
        std::cerr << "Error reading wheel position data from Modbus" << std::endl; 
        return {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    }

    try
    {
        int32_t l_pulse = join_u16_to_s32(regs[0], regs[1]);
        int32_t r_pulse = join_u16_to_s32(regs[2], regs[3]);
        //std::cout << "L: " << l_pulse << "  R: " << r_pulse << std::endl;

        float l_traveled = (static_cast<float>(l_pulse) / static_cast<float>(cpr_)) * (2.0f * M_PI);  // R_Wheel_
        float r_traveled = (static_cast<float>(r_pulse) / static_cast<float>(cpr_)) * (2.0f * M_PI );

        float left  = l_traveled;
        float right = -r_traveled;
 
        return {left, right};
    }
    catch(const std::exception& e)
    {
        std::cerr << "❌ Overflow/compute error while calculating wheel position: " << e.what() << std::endl;
        return {std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()};
    }
}


std::pair<float, float> ZLAC8015D_SDK::get_wheel_angular_velocities()
{
    auto [rpmL_raw, rpmR_raw] = get_rpm();

    if (std::isnan(rpmL_raw) || std::isnan(rpmR_raw)) {
        std::cerr << "❌ Unable to get RPM from the motor." << std::endl;
        return {0.0f, 0.0f};
    }

    constexpr float RPM_TO_RAD_S = 2.0f * static_cast<float>(M_PI) / 60.0f;

    // Nếu chỉ muốn đảo chiều (ví dụ right quay ngược so với left)
    float wL = rpmR_raw * RPM_TO_RAD_S;      // rad/s
    float wR = rpmL_raw * RPM_TO_RAD_S;      // rad/s  (hoặc -rpmR_raw nếu thực tế ngược chiều)

    return {wL, -wR};
}




std::pair<float, float> ZLAC8015D_SDK::get_rpm()
{
    if (!isOpen())
    {
        std::cerr << "Unable to get RPM speed: Modbus connection lost!" << std::endl;
        return {0.0f, 0.0f};
    }

    std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::L_FB_RPM, 2);

    if (regs.empty() || regs.size() < 2)
    {
        std::cerr << "Error: unable to read RPM from the motor!" << std::endl;
        return {0.0f, 0.0f};
    }

    try 
    {
        int16_t r_raw = static_cast<int16_t>(regs[0]);
        int16_t l_raw = static_cast<int16_t>(regs[1]);

        float fb_L_rpm = static_cast<float>(l_raw) / 10.0f;
        float fb_R_rpm = static_cast<float>(r_raw) / 10.0f;

        return {fb_L_rpm, fb_R_rpm};
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error processing RPM data: " << e.what() << std::endl;
        return {0.0f, 0.0f};
    }
}



std::pair<int32_t, int32_t> ZLAC8015D_SDK::get_wheels_tick()
{
    if(!isOpen())
    {
        std::cerr << "Unable to get wheels tick: Modbus connection lost!" << std::endl;
        return {0, 0};
    }

    std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::L_FB_POS_HI, 4);
    if(regs.empty() || regs.size() < 4)
    {
        std::cerr << "Error reading encoder data from Modbus" << std::endl;
        return {0, 0};
    }

    try
    {
        // Đúng theo datasheet: 0-1 là Right, 2-3 là Left
        int32_t r_tick = join_u16_to_s32(regs[0], regs[1]);
        int32_t l_tick = join_u16_to_s32(regs[2], regs[3]);

        return {l_tick, -r_tick};   // trả về theo thứ tự (Left, Right)
    }
    catch(const std::exception& e)
    {
        std::cerr << "❌ Overflow/compute error while reading encoder tick: " << e.what() << std::endl;
        return {0, 0};
    }
}



float ZLAC8015D_SDK::get_battery_voltage()
{
    if(!isOpen())
    {
        std::cerr << "Modbus not connected, can not read voltage! "<< std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }
    std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::VOLTAGE, 1);
    if(regs.empty())
    {
        std::cerr << "Error reading bus voltage!" << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }
    try{
        float voltage = regs[0] * 0.01f;
        return voltage;
    }
    catch(const std::exception& e) {
        std::cerr << "Error reading bus voltage " << e.what() << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
        
    }
}

ControlMode ZLAC8015D_SDK::getMode()
{
    if (!isOpen()) {
        std::cerr << "❌ Modbus not connected!" << std::endl;
        return ControlMode::SPEED_RPM; // fallback mặc định
    }
    std::vector<uint16_t> regs = modbus_fail_read_handler(zlac::OPR_MODE, 1);
    if(regs.empty()) return ControlMode::SPEED_RPM;

    switch ((regs[0]))
    {
    case zlac::POS_REL_CONTROL:
        return ControlMode::RELATIVE_POSITION;
    case zlac::POS_ABS_CONTROL:
        return ControlMode::ABSOLUTE_POSITION;
    case zlac::VEL_CONTROL:
        return ControlMode::SPEED_RPM;
    default:
        std::cerr << "⚠️ Unknown mode value: " << regs[0] << std::endl;
        return ControlMode::SPEED_RPM; 
    }

}
std::pair<int, int> ZLAC8015D_SDK::get_motor_faults()
{
    if (!isOpen()) {
        std::cerr << "❌ Modbus not connected!" << std::endl;
        return {-1, -1};
    }

    // Đọc 2 thanh ghi từ địa chỉ 0x20A5 (L_fault, R_fault)
    std::vector<uint16_t> regs = modbus_fail_read_handler(0x20A5, 2);
    if (regs.empty() || regs.size() < 2) {
        std::cerr << "❌ Error reading motor fault status!" << std::endl;
        return {-1, -1};
    }

    try {
        int L_fault = static_cast<int>(regs[0]);
        int R_fault = static_cast<int>(regs[1]);
        return {L_fault, R_fault};
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception while reading motor faults: " << e.what() << std::endl;
        return {-1, -1};
    }
}
float ZLAC8015D_SDK::get_motor_temperature()
{
    if (!isOpen()) {
        std::cerr << "❌ Modbus not connected!" << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }

    std::vector<uint16_t> regs = modbus_fail_read_handler(0x20A4, 1);
    if (regs.empty()) {
        std::cerr << "❌ Error reading motor temperature!" << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }

    try {
        // Theo thực tế: raw * 0.01 °C
        return static_cast<float>(regs[0]) * 0.01f;
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception while reading motor temperature: " << e.what() << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }
}

// ====================== Driver Temperature =====================
float ZLAC8015D_SDK::get_driver_temperature()
{
    if (!isOpen()) {
        std::cerr << "❌ Modbus not connected!" << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }

    std::vector<uint16_t> regs = modbus_fail_read_handler(0x20B0, 1);
    if (regs.empty()) {
        std::cerr << "❌ Error reading driver temperature!" << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }

    try {
        // Đơn vị: 0.1°C
        return static_cast<float>(regs[0]) * 0.1f;
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception while reading driver temperature: " << e.what() << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
    }
}

std::string ZLAC8015D_SDK::get_brake_state()
{
    if (!isOpen()) {
        std::cerr << "❌ Modbus not connected!" << std::endl;
        return "Unknown";
    }

    std::vector<uint16_t> regs = modbus_fail_read_handler(0x201A, 1);
    if (regs.empty()) {
        std::cerr << "❌ Error reading brake status!" << std::endl;
        return "Unknown";
    }

    try {
        return (regs[0] == 1) ? "Closed" : "Open";
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception while reading brake status: " << e.what() << std::endl;
        return "Unknown";
    }
}

// ====================== Emergency Stop =====================
bool ZLAC8015D_SDK::stop_motor_emergency()
{
    if (!isOpen()) {
        std::cerr << "⚠ Modbus connection lost! Attempting to reconnect..." << std::endl;
        // thử reconnect tối đa 5 lần
        for (int i = 0; i < 5; i++) {
            if (openDriver(port_, baudrate_, slave_id_)) {
                std::cout << "✅ Modbus reconnection successful!" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    if (isOpen()) {
        if (writeRegister(zlac::CONTROL_REG, zlac::EMERGENCY_STOP)) {
            std::cout << "🛑 Emergency stop command sent to motor!" << std::endl;
            return true;
        } else {
            std::cerr << "❌ Failed to send Emergency Stop command!" << std::endl;
            return false;
        }
    } else {
        std::cerr << "❌ Unable to reconnect Modbus. Please check wiring or power." << std::endl;
        return false;
    }
}

// ====================== Clear Alarm =====================
bool ZLAC8015D_SDK::clear_alarm()
{
    if (!isOpen()) {
        std::cerr << "No connection! Cannot send clear_alarm() command." << std::endl;
        return false;
    }

    if (writeRegister(zlac::CONTROL_REG, zlac::CLEAR_ALARM)) {
        std::cout << "✅ Error cleared on the controller." << std::endl;
        return true;
    } else {
        std::cerr << "Unable to clear error! Check the connection and retry." << std::endl;
        return false;
    }
}

bool ZLAC8015D_SDK::clear_position(zlac_modbus::ClearPos pos)
{
    uint8_t val = static_cast<uint16_t>(pos);
    if (val > 3) 
    {
        std::cerr << "❌ clear_position: only 0, 1, 2, or 3 allowed" << std::endl;
        return false;
    }

    if (!isOpen()) 
    {
        std::cerr << "❌ No Modbus connection, cannot reset encoder position!" << std::endl;
        return false;
    }


    switch (pos) {
        case zlac_modbus::ClearPos::LEFT:
            std::cout << "✅ Reset encoder position (Left)" << std::endl; break;
        case zlac_modbus::ClearPos::RIGHT: 
            std::cout << "✅ Reset encoder position (Right)" << std::endl; break;
        case zlac_modbus::ClearPos::BOTH: 
            std::cout << "✅ Reset encoder position (Both sides)" << std::endl; break;
        default: break; // 0 = không reset
    }

    try {
        if (writeRegister(zlac::CLR_FB_POS, static_cast<uint16_t>(pos))) 
        {
            return true;
        } 
        else 
        {
            std::cerr << "❌ Error writing to CLR_FB_POS register" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception while resetting encoder position: " << e.what() << std::endl;
        return false;
    }
}

// static_cast<uint16_t>(pos))



zlac_modbus::MotorStatus ZLAC8015D_SDK::get_motor_status()
{
    MotorStatus status{};
    status.connected = isOpen();

    if(!status.connected)
    {
        std::cerr << "Modbus connection not open!" << std::endl;
        return status;
    }

    status.enabled = is_motor_enabled();
    status.mode = getMode();
    status.rpm = get_rpm();
    status.position = get_wheels_travelled();
    status.linear_velocity = {0.0f, 0.0f}; 
    status.faults = get_motor_faults();
    status.motor_temperature = get_motor_temperature();
    status.driver_temperature = get_driver_temperature();
    status.brake_state = get_brake_state();
    status.battery_voltage = get_battery_voltage();



    return status;

}

float ZLAC8015D_SDK::rpm_to_linear(float rpm)
{
    // v = (2π * RPM / 60) * R
    return (2.0f * static_cast<float>(M_PI) * rpm / 60.0f) * R_Wheel_;
}


std::pair<float, float> ZLAC8015D_SDK::get_linear_velocities()
{
    auto [rpmL, rpmR] = get_rpm();

    // Nếu không đọc được RPM (NaN), thì trả về NaN
    if (std::isnan(rpmL) || std::isnan(rpmR)) {
        std::cerr << "❌ Unable to get RPM from the motor." << std::endl;
        return {std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()};
    }

    float vL = rpm_to_linear(rpmL);
    float vR = rpm_to_linear(-rpmR);

    return {vL, vR};
}


// std::pair<float, float> ZLAC8015D_SDK::get_wheel_angular_velocities()
// {
//     auto [rpmL_raw, rpmR_raw] = get_rpm();

//     float rpmL = static_cast<float>(rpmL_raw);
//     float rpmR = static_cast<float>(rpmR_raw);

//     if (std::isnan(rpmL) || std::isnan(rpmR)) {
//         std::cerr << "❌ Unable to get RPM from the motor." << std::endl;
//         return {0.0f, 0.0f}; // an toàn
//     }

//     constexpr float RPM_TO_RAD_S = 2.0f * static_cast<float>(M_PI) / 60.0f;

//     float wL = rpmL * RPM_TO_RAD_S;     // rad/s
//     float wR = -rpmR * RPM_TO_RAD_S;    // đảo chiều cho đồng bộ

//     return {wL, wR};
// }







}
