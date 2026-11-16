from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from math import pi
import datetime

def generate_launch_description():
    
    nodes = []
    topics = []

### Change simulation parameters here ###
    n_boids = 10
    rand_seed = 17
    fov_r = 3
    fov_ang = (2*pi)*(1)
    dt = 0.1
###-----------------------------------###

    for i in range(n_boids):
        nodes.append(
            Node(
                package='sphero_stage',
                executable='boid_brain',
                name=f'boid_brain_{i}',
                parameters=[
                    {'robot_id': i},
                    {'n_boids': n_boids},
                    {'rand_seed': rand_seed},
                    {'fov_r': fov_r},
                    {'fov_ang': fov_ang},
                    {'dt': dt}],
                output='screen'
            )
        )
        topics.append(f'/robot_{i}/odom')
        topics.append(f'/robot_{i}/cmd_vel')
        topics.append(f'/robot_{i}/ac')
        
    # Rosbag recording, change path if needed
#    fov_ang = round(fov_ang, 2)
#    c_ali = round(c_ali, 2)
#    c_coh = round(c_coh, 2)
#    c_sep = round(c_sep, 2)
#    time = datetime.datetime.now()
       
#    bag_record = ExecuteProcess(
#        cmd=['ros2', 'bag', 'record', '-o',
#        '/root/ros2_ws/src/sphero_simulation/sphero_stage/bags/' + 
#        f'nb{n_boids}r{fov_r}fov{fov_ang}ca{c_ali}cc{c_coh}cs{c_sep}_{time}',
#        *topics],
#        output='screen'
#    )
#    nodes.append(bag_record)

    return LaunchDescription(nodes)