# The Jogging joint message contains control information for jogging
# a specific joint
# Set velocity_scale to 0 to stop jogging

# The jogged joint index
# Negative joint index value means stop jogging
int8 joint_index

# The jogging direction
int8 direction

# The jogging mode
int8 mode

# The velocity scale for move_group interface
float32 velocity_scale

int8 DIR_PLUS = 0
int8 DIR_MINUS = 1

int8 MOD_CONTINUOUS            = 0
int8 MOD_INCREMENT_10_DEGREE   = 1
int8 MOD_INCREMENT_1_DEGREE    = 2
int8 MOD_INCREMENT_01_DEGREE   = 3