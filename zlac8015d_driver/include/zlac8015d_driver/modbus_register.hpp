#pragma once

#include <cstdint>
#include <array>

namespace zlac {


        // 🧩 Modbus Register Addresses

        constexpr uint16_t CONTROL_REG        = 0x200E;
        constexpr uint16_t OPR_MODE           = 0x200D;
        constexpr uint16_t L_ACL_TIME         = 0x2080;
        constexpr uint16_t R_ACL_TIME         = 0x2081;
        constexpr uint16_t L_DCL_TIME         = 0x2082;
        constexpr uint16_t R_DCL_TIME         = 0x2083;
        constexpr uint16_t CLR_FB_POS         = 0x2005;
        constexpr uint16_t L_CMD_RPM          = 0x2088;
        constexpr uint16_t R_CMD_RPM          = 0x2089;
        constexpr uint16_t L_FB_POS_HI        = 0x20A7;
        constexpr uint16_t POS_CONTROL_TYPE   = 0x200F;
        constexpr uint16_t L_MAX_RPM_POS      = 0x208E;
        constexpr uint16_t L_CMD_REL_POS_HI   = 0x208A;
        constexpr uint16_t L_FB_RPM           = 0x20AB;
        constexpr uint16_t R_FB_RPM           = 0x20AC;

        //
        // 🧩 Control Commands
        //
        constexpr uint16_t EMERGENCY_STOP     = 0x05;
        constexpr uint16_t CLEAR_ALARM        = 0x06;
        constexpr uint16_t STOP               = 0x07;
        constexpr uint16_t ENABLE             = 0x08;
        constexpr uint16_t POS_SYNC           = 0x10;
        constexpr uint16_t POS_L_START        = 0x11;
        constexpr uint16_t POS_R_START        = 0x12;

        //
        // 🧩 Operation Modes
        //
        constexpr uint16_t POS_REL_CONTROL    = 1;
        constexpr uint16_t POS_ABS_CONTROL    = 2;
        constexpr uint16_t VEL_CONTROL        = 3;
        constexpr uint16_t ASYNC              = 0;
        constexpr uint16_t SYNC               = 1;

        //
        // 🧩 Troubleshooting Registers
        //
        constexpr uint16_t L_FAULT            = 0x20A5;
        constexpr uint16_t R_FAULT            = 0x20A6;

        //
        // 🧩 Fault Codes
        //
        constexpr uint16_t NO_FAULT           = 0x0000;
        constexpr uint16_t OVER_VOLT          = 0x0001;
        constexpr uint16_t UNDER_VOLT         = 0x0002;
        constexpr uint16_t OVER_CURR          = 0x0004;
        constexpr uint16_t OVER_LOAD          = 0x0008;
        constexpr uint16_t CURR_OUT_TOL       = 0x0010;
        constexpr uint16_t ENCOD_OUT_TOL      = 0x0020;
        constexpr uint16_t MOTOR_BAD          = 0x0040;
        constexpr uint16_t REF_VOLT_ERROR     = 0x0080;
        constexpr uint16_t EEPROM_ERROR       = 0x0100;
        constexpr uint16_t WALL_ERROR         = 0x0200;
        constexpr uint16_t HIGH_TEMP          = 0x0400;

        //
        // 🧩 Fault Code List
        //
        constexpr std::array<uint16_t, 11> FAULT_LIST = {
            OVER_VOLT, UNDER_VOLT, OVER_CURR, OVER_LOAD,
            CURR_OUT_TOL, ENCOD_OUT_TOL, MOTOR_BAD,
            REF_VOLT_ERROR, EEPROM_ERROR, WALL_ERROR, HIGH_TEMP
};

}  // namespace zlac
