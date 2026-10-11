std_msgs/Header header                 # 最后一次真实观测时刻；frame_id=world
string target_id                       # 阶段 A 为 intruder_uav
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
float32 confidence                     # 来源内已校准的观测质量
string source                          # gt_adapter / air_fusion
