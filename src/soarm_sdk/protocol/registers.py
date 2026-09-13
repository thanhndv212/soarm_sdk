"""Core protocol constants for STServo actuators."""

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
STS_END = 0

# Instruction for STServo protocol
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_SYNC_WRITE = 0x83
INST_SYNC_READ = 0x82

# Communication result codes
COMM_SUCCESS = 0
COMM_PORT_BUSY = -1
COMM_TX_FAIL = -2
COMM_RX_FAIL = -3
COMM_TX_ERROR = -4
COMM_RX_WAITING = -5
COMM_RX_TIMEOUT = -6
COMM_RX_CORRUPT = -7
COMM_NOT_AVAILABLE = -9

# SCSCL control table addresses
SCSCL_GOAL_POSITION_L = 42
SCSCL_GOAL_TIME_L = 44
SCSCL_GOAL_SPEED_L = 46
SCSCL_LOCK = 48
SCSCL_MIN_ANGLE_LIMIT_L = 9
SCSCL_PRESENT_POSITION_L = 56
SCSCL_PRESENT_SPEED_L = 58
SCSCL_MOVING = 66

# STS control table addresses
STS_ID = 5
STS_BAUD_RATE = 6
STS_MIN_ANGLE_LIMIT_L = 9
STS_MIN_ANGLE_LIMIT_H = 10
STS_MAX_ANGLE_LIMIT_L = 11
STS_MAX_ANGLE_LIMIT_H = 12
STS_TORQUE_ENABLE = 40
STS_ACC = 41
STS_GOAL_POSITION_L = 42
STS_GOAL_TIME_L = 44
STS_GOAL_SPEED_L = 46
STS_MODE = 33
STS_MOVING = 66
STS_PRESENT_POSITION_L = 56
STS_PRESENT_SPEED_L = 58
STS_LOCK = 55

# 波特率定义
STS_1M = 0
STS_0_5M = 1
STS_250K = 2
STS_128K = 3
STS_115200 = 4
STS_76800 = 5
STS_57600 = 6
STS_38400 = 7

# 内存表定义
# -------EPROM(只读)--------
STS_MODEL_L = 3
STS_MODEL_H = 4

# -------EPROM(读写)--------
STS_ID = 5
STS_BAUD_RATE = 6
STS_MIN_ANGLE_LIMIT_L = 9
STS_MIN_ANGLE_LIMIT_H = 10
STS_MAX_ANGLE_LIMIT_L = 11
STS_MAX_ANGLE_LIMIT_H = 12
STS_CW_DEAD = 26
STS_CCW_DEAD = 27
STS_P_COEF = 21  # Proportional gain (EEPROM, 1 byte, default ~32)
STS_D_COEF = 22  # Derivative gain   (EEPROM, 1 byte, default ~32)
STS_I_COEF = 23  # Integral gain     (EEPROM, 1 byte, default 0)
STS_OFS_L = 31
STS_OFS_H = 32
STS_MODE = 33

# -------SRAM(读写)--------
STS_TORQUE_ENABLE = 40
STS_ACC = 41
STS_GOAL_POSITION_L = 42
STS_GOAL_POSITION_H = 43
STS_GOAL_TIME_L = 44
STS_GOAL_TIME_H = 45
STS_GOAL_SPEED_L = 46
STS_GOAL_SPEED_H = 47
STS_LOCK = 55

# -------SRAM(只读)--------
STS_PRESENT_POSITION_L = 56
STS_PRESENT_POSITION_H = 57
STS_PRESENT_SPEED_L = 58
STS_PRESENT_SPEED_H = 59
STS_PRESENT_LOAD_L = 60
STS_PRESENT_LOAD_H = 61
STS_PRESENT_VOLTAGE = 62
STS_PRESENT_TEMPERATURE = 63
STS_STATUS = 65
STS_MOVING = 66
STS_PRESENT_CURRENT_L = 69
STS_PRESENT_CURRENT_H = 70

# -------Telemetry block (SRAM read-only, 56..70)--------
#: First address of the contiguous read-only telemetry block.
STS_TELEMETRY_START = STS_PRESENT_POSITION_L  # 56
#: Length in bytes of that block, position through current inclusive.
#: Addresses 64, 67 and 68 are gaps — read as part of the block, discarded.
STS_TELEMETRY_LENGTH = STS_PRESENT_CURRENT_H - STS_PRESENT_POSITION_L + 1  # 15

# Sign-bit positions for the sign-magnitude registers. PRESENT_LOAD holds a
# magnitude of 0..1000 (0.1% PWM duty) in bits 0-9 with direction in bit 10;
# position, speed and current are sign-magnitude in bit 15. This follows the
# Feetech SMS/STS convention (cf. ReadCorrection, whose offset register signs
# at bit 11).
STS_POSITION_SIGN_BIT = 15
STS_SPEED_SIGN_BIT = 15
STS_LOAD_SIGN_BIT = 10
STS_CURRENT_SIGN_BIT = 15

# Raw LSB -> unit scale factors.
STS_LOAD_PERCENT_PER_LSB = 0.1
STS_CURRENT_MA_PER_LSB = 6.5
STS_VOLTAGE_V_PER_LSB = 0.1
