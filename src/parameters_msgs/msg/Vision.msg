################### Camera 1 ##################
# Camera 1 position
geometry_msgs/Vector3 rok1_F_F

# Camera 1 orientation
geometry_msgs/Vector3 camera1_euler

# Camera 1 stream status
bool camera1_stream_status

# Camera 1 corresponding points status
bool camera1_cp_status

# Camera 1 base point status
bool camera1_bp_status

# Camera 1 image type (0: GrayScale, 1: Binary, 2: Binary ROI)
int8 camera1_type

# Camera 1 binary threshold number
float32 camera1_binary_threshold_number

################### Camera 2 ##################
# Camera 2 position
geometry_msgs/Vector3 rok2_F_F

# Camera 2 orientation
geometry_msgs/Vector3 camera2_euler

# Camera 2 stream status
bool camera2_stream_status

# Camera 2 corresponding points status
bool camera2_cp_status

# Camera 2 base point status
bool camera2_bp_status

# Camera 2 image type (0: GrayScale, 1: Binary, 2: Binary ROI)
int8 camera2_type

# Camera 2 binary threshold number
float32 camera2_binary_threshold_number