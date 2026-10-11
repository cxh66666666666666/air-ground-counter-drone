std_msgs/Header header                 # 观测时刻（传感器时间）；frame_id=world
string target_id                       # 阶段 A 为 intruder_uav
string sensor_id                       # 观测通道，如 air_eo_front / air_ir_top
geometry_msgs/PoseWithCovariance pose  # world 系；协方差对角=各轴 sigma^2
float32 confidence                     # 该源已标定的观测质量（经验覆盖 P(|err|<=tol)）
uint32 obs_seq                         # 该通道观测序号（可据此核查丢包）
