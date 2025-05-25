## Common
CONTROL_REG                     = 0x200E # {0x05: emergency stop, 0x06: clear stop, 0x07: stop, 0x08: enable}
OPR_MODE                        = 0x200D     # control mode
L_ACL_TIME                      = 0x2080
R_ACL_TIME                      = 0x2081
L_DCL_TIME                      = 0x2082
R_DCL_TIME                      = 0x2083

# clear feedback position
CLR_FB_POS                      = 0x2005

## Velocity control (Speed RPM Control)
# Target velocity Range: --450~450/min

L_CMD_RPM                       = 0x2088
R_CMD_RPM                       = 0x2089



# Position control
L_FB_POS_HI                     = 0x20A7
POS_CONTROL_TYPE                = 0x200F  # Synchronous/asynchronous control status

L_MAX_RPM_POS                   = 0x208E # Not in use
L_CMD_REL_POS_HI                = 0x208A # Not in use


# Actual velocity  unit: 0.1r/min
L_FB_RPM                        = 0x20AB
R_FB_RPM                        = 0x20AC



########################
## Control CMDs (REG) ##
########################

EMER_STOP                       = 0x05 # emergency stop
ALRM_CLR                        = 0x06 # clear default 
DOWN_TIME                       = 0x07 # stop
ENABLE                          = 0x08    # enable

POS_SYNC                        = 0x10
POS_L_START                     = 0x11
POS_R_START                     = 0x12


####################
## Operation Mode ##
####################

POS_REL_CONTROL                 = 1
POS_ABS_CONTROL = 2
VEL_CONTROL                     = 3

ASYNC                           = 0
SYNC                            = 1


## Troubleshooting
L_FAULT                         = 0x20A5
R_FAULT                         = 0x20A6

#################
## Fault codes ##
#################

NO_FAULT                        = 0x0000       # No error
OVER_VOLT                       = 0x0001      # Over voltage
UNDER_VOLT                      = 0x0002     # Under voltage
OVER_CURR                       = 0x0004      # Over current
OVER_LOAD                       = 0x0008      # Over load
CURR_OUT_TOL                    = 0x0010   # Current out of tolerance
ENCOD_OUT_TOL                   = 0x0020  # Encoder out of tolerance
MOTOR_BAD                       = 0x0040      # Velocity out of tolerance
REF_VOLT_ERROR                  = 0x0080 # Reference voltage error
EEPROM_ERROR                    = 0x0100   # EEPROM error
WALL_ERROR                      = 0x0200     # Hall error
HIGH_TEMP                       = 0x0400      # Motor temperature over temperature

FAULT_LIST                      = [OVER_VOLT, UNDER_VOLT, OVER_CURR, OVER_LOAD, CURR_OUT_TOL, ENCOD_OUT_TOL, \
                                    MOTOR_BAD, REF_VOLT_ERROR, EEPROM_ERROR, WALL_ERROR, HIGH_TEMP]