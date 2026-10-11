std_msgs/Header header                 # 观测时刻（传感器时间）；frame_id=world
string obstacle_id                     # 如 pole_1 / pole_2
string sensor_id                       # 观测通道
geometry_msgs/PoseWithCovariance pose  # world 系障碍中心位姿
geometry_msgs/Vector3 size             # 障碍尺寸 (m)
float32 confidence                     # 该源已标定的观测质量
uint32 obs_seq                         # 该通道观测序号
