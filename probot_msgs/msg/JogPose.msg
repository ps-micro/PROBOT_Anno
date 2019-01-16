# The Jogging pose message contains control information for jogging
# end-effector to a target pose
# It means stopping jogging if mask is JOGGING_STOP

# The bit map mask to indicate which pose data is jogged
int8 mask

# The jogging direction
int8 direction

# The jogging mode
int8 mode

# The velocity scale for move_group interface
float32 velocity_scale

# The jogging frame id
int8 frame_id

int8 DIR_PLUS = 0
int8 DIR_MINUS = 1

# bit map of jogging
int8 JOGGING_STOP = 0
int8 JOGGING_X = 1
int8 JOGGING_Y = 2
int8 JOGGING_Z = 4
int8 JOGGING_ROLL = 8
int8 JOGGING_PITCH = 16
int8 JOGGING_YAW = 32

int8 MOD_CONTINUOUS            = 0
int8 MOD_INCREMENT_10_MM       = 1
int8 MOD_INCREMENT_1_MM        = 2
int8 MOD_INCREMENT_01_MM       = 3