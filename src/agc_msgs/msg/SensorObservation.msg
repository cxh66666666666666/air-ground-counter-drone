std_msgs/Header header                 # 该通道数组发布时刻；frame_id=world
string sensor_id                       # 观测通道标识
TargetObservation[] targets            # 本拍该通道的目标观测（无则空）
ObstacleObservation[] obstacles        # 本拍该通道的障碍观测（无则空）
