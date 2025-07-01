from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PerceptionAgent: 采集并发布原始图像和骨架数据
        Node(
            package='jupiterobot2_vision',
            executable='perception_agent',
            name='perception_agent',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
        ),
        # DecisionAgent: YOLOv5 + 骨架规则评估，发布跌倒事件
        Node(
            package='jupiterobot2_vision',
            executable='decision_agent',
            name='decision_agent',
            output='screen',
            parameters=[
                {'yolo_model_path': '~/catkin_ws/src/jupiterobot2/jupiterobot2_vision/models/yolov5_fall.pt'},
                {'use_sim_time': False},
            ],
        ),
        # ExecutionAgent: 根据跌倒事件执行报警播报
        Node(
            package='jupiterobot2_vision',
            executable='execution_agent',
            name='execution_agent',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
        ),
    ])
