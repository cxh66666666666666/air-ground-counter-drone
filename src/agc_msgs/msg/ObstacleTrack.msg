std_msgs/Header header                 # 最后一次真实观测时刻；frame_id=world
string obstacle_id                     # 障碍标识
geometry_msgs/Pose pose                # world 系障碍中心位姿
geometry_msgs/Vector3 size             # 障碍尺寸 (m)，融合后为保守外扩值
float32 confidence                     # 融合置信度
string source                          # air_fusion
