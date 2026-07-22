# Free-run observation snapshot, published by sim_node every control window.
uint64 obs_seq
uint64 window                        # sim window index within the episode
builtin_interfaces/Time stamp
sensor_msgs/JointState joint_state
float64[] qpos_full
geometry_msgs/Pose flange_pose
geometry_msgs/Pose object_pose
sensor_msgs/Image[] images
bool done
