from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import datetime
import yaml
import os

def generate_launch_description():
    
    nodes = []
    topics = []

    package_share = get_package_share_directory('sphero_stage')
    config_path = os.path.join(package_share, 'launch', 'launch_params.yaml')
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)

    basic_reynolds = config['basic_reynolds']
    n_boids = config['num_of_robots']
    rand_seed = config['rand_seed']
    dt = config['dt']

    distribution = config['distribution']
    distribution_params = config['distribution_list'][distribution]
    swarm_target_x = distribution_params['center_x']
    swarm_target_y = distribution_params['center_y']

    for i in range(config['num_of_robots']):
        nodes.append(
            Node(
                package='sphero_stage',
                executable='boid_brain',
                name=f'boid_brain_{i}',
                parameters=[
                    {'basic_reynolds': basic_reynolds},
                    {'robot_id': i},
                    {'n_boids': n_boids},
                    {'rand_seed': rand_seed},
                    {'dt': dt},
                    {'swarm_target_x': swarm_target_x},
                    {'swarm_target_y': swarm_target_y},
                    ],
                output='screen'
            )
        )
        topics.append(f'/robot_{i}/odom')
        topics.append(f'/robot_{i}/cmd_vel')
        topics.append(f'/robot_{i}/ac')
        
    # Rosbag recording, change path if needed
#    time = datetime.datetime.now()
       
#    bag_record = ExecuteProcess(
#        cmd=['ros2', 'bag', 'record', '-o',
#        '/root/ros2_ws/src/sphero_simulation/sphero_stage/bags/' + 
#        f'bag_{time}',
#        *topics],
#        output='screen'
#    )
#    nodes.append(bag_record)

    return LaunchDescription(nodes)