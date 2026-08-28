uint8 VEHICLE_STATE_NORMAL = 0
uint8 VEHICLE_STATE_ESTOP = 1
uint8 VEHICLE_STATE_EXCEPTION = 2

uint8 CONTROL_MODE_STANDBY = 0
uint8 CONTROL_MODE_CAN = 1
uint8 CONTROL_MODE_UART = 2
uint8 CONTROL_MODE_RC = 3

uint8 MOTION_MODE_DUAL_ACKERMAN = 0
uint8 MOTION_MODE_PARALLEL = 1
uint8 MOTION_MODE_SPINNING = 2
uint8 MOTION_MODE_SIDE_SLIP = 3

########################################

std_msgs/Header header

# system state
uint8 vehicle_state
uint8 control_mode
# Legacy Protocol V2 error field. Ranger Mini 3.0 consumers must use error_code_v3.
uint16 error_code
# Ranger Mini 3.0 big-endian error bytes [4..7]; zero on other variants.
uint32 error_code_v3
float64 battery_voltage

# motion state
uint8 motion_mode
