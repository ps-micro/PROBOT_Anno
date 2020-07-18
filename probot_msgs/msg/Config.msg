# The Config message contains system configuration data

# The output log level for controller
int8 controller_log_level
bool init_with_all_zero_pos

# The zero calibration parameters
bool        enable_joint_calibration
float32[]   seneor_position
float32[]   speed_to_sensor
float32[]   speed_to_zero

# The tool's relative pose to end-effector(tool0)
# This is used to jog tool instead of end-effector
# If it is to jog end-effector, set pose_to_eef to 0, that is (0,0,0)(0,0,0,1)
geometry_msgs/Pose[] pose_to_eef

# The bit-map mask of valid fields which this message contains
int32 mask

int32 MASK_LOGLEVEL          = 1
int32 MASK_ZERO_CALIBRATION  = 2
int32 MASK_TOOL_POSE         = 4
