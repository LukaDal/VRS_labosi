# Node control for each boid
import random
import math
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from sphero_interfaces.msg import Accelerations
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class BoidBrain(Node):
    def __init__(self):
        super().__init__('boid_brain')

        #---VARIABLES----------------------------------------------------------------------------------------

        # Simulation parameters (unchangeable during runtime)
        self.declare_parameter('basic_reynolds', False)
        self.basic_reynolds = self.get_parameter('basic_reynolds').value
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').value
        self.declare_parameter('n_boids', 10)
        self.n_boids = self.get_parameter('n_boids').value
        self.declare_parameter('rand_seed', 1)
        self.rand_seed = self.get_parameter('rand_seed').value
        self.declare_parameter('dt', 0.1)
        self.dt = self.get_parameter('dt').value

        # Boid GENREAL parameters - defaults (updated from Hivemind node and rqt)
        self.fov_r = 1.0
        self.fov_ang = 2.0 * math.pi
        self.max_vel = 2.0
        self.break_ac = 0.3

        # Boid REYNOLDS parameters - defaults (updated from Hivemind node and rqt)
        self.c_ali = 1.0
        self.c_coh = 1.0
        self.c_sep = 1.0
        self.c_tar = 1.0
        self.c_wal = 1.0

        # Boid swarm target
        self.declare_parameter('swarm_target_x', 0.0)
        self.swarm_target_x = self.get_parameter('swarm_target_x').value
        self.declare_parameter('swarm_target_y', 0.0)
        self.swarm_target_y = self.get_parameter('swarm_target_y').value

        # Boid's current heading
        self.head_x = None
        self.head_y = None

        # Full odometry tracking
        self.odom_data = {}
        self.odom_subs = []

        # Map
        self.map = None
        self.map_res = None
        self.map_width = None
        self.map_height = None
        self.map_ox = None
        self.map_oy = None

        #---COMMUNICATION------------------------------------------------------------------------------------

        # Create subscribers to other boid odometries
        for i in range(self.n_boids):
            topic_odom = f'robot_{i}/odom'
            sub = self.create_subscription(
                Odometry,
                topic_odom,
                lambda msg, rid=i: self.odom_callback(msg, rid),
                10
            )
            self.odom_subs.append(sub)

        # Create subscribers for boid parameters and swarm target
        self.param_sub_gen = self.create_subscription(Float64MultiArray, '/boid_params_gen', self.param_gen_callback, 10)
        self.param_sub_rey = self.create_subscription(Float64MultiArray, '/boid_params_rey', self.param_rey_callback, 10)
        self.target_sub = self.create_subscription(Float64MultiArray, '/boid_swarm_target', self.target_callback, 10)

        # Create subscriber for the level map
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # Create publishers
        topic_cmd_vel = f'robot_{self.robot_id}/cmd_vel'
        topic_ac = f'robot_{self.robot_id}/ac'
        self.publisher_vel = self.create_publisher(Twist, topic_cmd_vel, 10)
        self.publisher_ac = self.create_publisher(Accelerations, topic_ac, 10)

        #---INITIALIZATION---------------------------------------------------------------------------------------------

        # Initialize timer
        self.last_timestamp = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.controller)

        # Launch check
        self.get_logger().info(f'Boid {self.robot_id} online!')

        # Send random initial velocity
#        rng = random.Random(self.rand_seed + self.robot_id)
#        rand_vx_cmd = rng.uniform(-0.5, 0.5)
#        rand_vy_cmd = rng.uniform(-0.5, 0.5)
#        rand_vel_cmd = Twist()
#        rand_vel_cmd.linear.x = rand_vx_cmd
#        rand_vel_cmd.linear.y = rand_vy_cmd
#        self.publisher_vel.publish(rand_vel_cmd)



    def odom_callback(self, msg, robot_id):
        self.odom_data[robot_id] = msg

    def param_gen_callback(self, msg):
        self.fov_r = msg.data[0]
        self.fov_ang = msg.data[1]
        self.max_vel = msg.data[2]
        self.break_ac = msg.data[3]

    def param_rey_callback(self, msg):
        self.c_ali = msg.data[0]
        self.c_coh = msg.data[1]
        self.c_sep = msg.data[2]
        self.c_tar = msg.data[3]
        self.c_wal = msg.data[4]

    def target_callback(self, msg):
        self.swarm_target_x = msg.data[0]
        self.swarm_target_y = msg.data[1]

    def map_callback(self, msg):
        self.map_res = msg.info.resolution
        self.map_width =  msg.info.width
        self.map_height = msg.info.height
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        self.map = np.array(msg.data).reshape(self.map_height, self.map_width)
        self.destroy_subscription(self.map_sub)
        self.map_sub = None



    def controller(self):

        # Calculate elapsed time
        timestamp = self.get_clock().now()
        elapsed = (timestamp - self.last_timestamp).nanoseconds * (1e-9)
        self.last_timestamp = timestamp

        # Wait until we have all odometries and a map
        if (len(self.odom_data) < self.n_boids) or (self.map is None):
            return
        
        # Calculate heading (h vector)
        self.calculate_heading()

        # Find all visible boids
        visible_boids = self.find_visible()

        # Calculate Reynolds accelerations
        ax, ay = self.reynolds(visible_boids)

        # Calculate and publish velocity command
        vx = self.odom_data[self.robot_id].twist.twist.linear.x
        vy = self.odom_data[self.robot_id].twist.twist.linear.y
        vx_cmd = vx + ax*elapsed
        vy_cmd = vy + ay*elapsed
        
        # Limit velocities
        vx_cmd = max(-self.max_vel, min(self.max_vel, vx_cmd))
        vy_cmd = max(-self.max_vel, min(self.max_vel, vy_cmd))

        # Publish velocities
        vel_cmd = Twist()
        vel_cmd.linear.x = vx_cmd
        vel_cmd.linear.y = vy_cmd
        self.publisher_vel.publish(vel_cmd)



    def calculate_heading(self):

        qx = self.odom_data[self.robot_id].pose.pose.orientation.x
        qy = self.odom_data[self.robot_id].pose.pose.orientation.y
        qz = self.odom_data[self.robot_id].pose.pose.orientation.z
        qw = self.odom_data[self.robot_id].pose.pose.orientation.w

        yaw = math.atan2( (2*(qw*qz+qx*qy)) , (1 - 2*(qy*qy+qz*qz)))
        self.head_x = math.cos(yaw)
        self.head_y = math.sin(yaw)



    def find_visible(self):

        x = self.odom_data[self.robot_id].pose.pose.position.x
        y = self.odom_data[self.robot_id].pose.pose.position.y
        fov_cos = math.cos(self.fov_ang / 2)

        visible_boids = []
        for robot_id, odom in self.odom_data.items():

            # Skip ourselves
            if robot_id == self.robot_id:
                continue
            
            # (d vector)
            x_other = odom.pose.pose.position.x
            y_other = odom.pose.pose.position.y
            dx = x_other - x
            dy = y_other - y
            d = math.hypot(dx, dy)

            # Fov check
            if (d < 1e-6) or (d > self.fov_r):
                continue
            h_dot_d = self.head_x * dx + self.head_y * dy
            if h_dot_d / d < fov_cos:
                continue

            visible_boids.append({
                'id': robot_id,
                'odom': odom,
                'dx': dx,
                'dy': dy,
                'd': d
            })

        return visible_boids



    def reynolds(self, visible_boids):

        x = self.odom_data[self.robot_id].pose.pose.position.x
        y = self.odom_data[self.robot_id].pose.pose.position.y
        vx = self.odom_data[self.robot_id].twist.twist.linear.x
        vy = self.odom_data[self.robot_id].twist.twist.linear.y
        n_vis = len(visible_boids)

        # BASIC REYNOLDS CONTROL ------------------------------------------------------------------

        sum_ali_x = 0.0
        sum_ali_y = 0.0
        sum_sep_x = 0.0
        sum_sep_y = 0.0
        cent_x = 0.0
        cent_y = 0.0
        for boid in visible_boids:
            sum_ali_x += boid['odom'].twist.twist.linear.x - vx
            sum_ali_y += boid['odom'].twist.twist.linear.y - vy
            sum_sep_x += (boid['dx'])/(boid['d']**2)
            sum_sep_y += (boid['dy'])/(boid['d']**2)
            cent_x += boid['odom'].pose.pose.position.x
            cent_y += boid['odom'].pose.pose.position.y
        if n_vis > 0:
            cent_x /= n_vis
            cent_y /= n_vis
        else:
            cent_x = x
            cent_y = y

        ac_x_ali = self.c_ali * sum_ali_x
        ac_y_ali = self.c_ali * sum_ali_y
        ac_x_coh = self.c_coh * (cent_x - x)
        ac_y_coh = self.c_coh * (cent_y - y)
        ac_x_sep = -self.c_sep * sum_sep_x
        ac_y_sep = -self.c_sep * sum_sep_y

        # ADVANCED CONTROL

        if not self.basic_reynolds:

            # TARGET CONTROL --------------------------------------------------------------------------

            tar_dx = self.swarm_target_x - cent_x
            tar_dy = self.swarm_target_y - cent_y
            tar_d = math.hypot(tar_dx, tar_dy)

            if tar_d > 1e-6:

                tar_dir_x = tar_dx / tar_d
                tar_dir_y = tar_dy / tar_d

                tar_safe_vel = math.sqrt(2.0 * self.break_ac * tar_d)
                tar_safe_vel = min(self.max_vel, tar_safe_vel)

                tar_safe_vx = tar_safe_vel * tar_dir_x
                tar_safe_vy = tar_safe_vel * tar_dir_y

                ac_x_tar = self.c_tar * (tar_safe_vx - vx)
                ac_y_tar = self.c_tar * (tar_safe_vy - vy)

            else:
                ac_x_tar = 0.0
                ac_y_tar = 0.0

            # WALL AVOIDANCE CONTROL ------------------------------------------------------------------

            boid_col = math.floor((x - self.map_ox) / self.map_res)
            boid_row = math.floor((y - self.map_oy) / self.map_res)
            cell_lim = math.ceil(self.fov_r / self.map_res)

            sum_x_wal = 0.0
            sum_y_wal = 0.0
            n_wall_cells = 0

            fov_cos = math.cos(self.fov_ang / 2)

            for i in range(-cell_lim, cell_lim + 1):
                for j in range(-cell_lim, cell_lim + 1):

                    cell_row = boid_row + i
                    cell_col = boid_col + j

                    if (not (0 <= cell_row < self.map_height)):
                        continue
                    if (not (0 <= cell_col < self.map_width)):
                        continue
                    if (self.map[cell_row, cell_col] < 80):
                        continue
                    
                    cell_x = (cell_col + 1/2)*self.map_res + self.map_ox
                    cell_y = (cell_row + 1/2)*self.map_res + self.map_oy
                    dx = cell_x - x
                    dy = cell_y - y
                    d = math.hypot(dx, dy)
                    h_dot_d = self.head_x * dx + self.head_y * dy

                    if (d < 1e-6) or (d > self.fov_r):
                        continue
                    if h_dot_d / d < fov_cos:
                        continue

                    sum_x_wal += (dx)/(d**3)
                    sum_y_wal += (dy)/(d**3)
                    n_wall_cells += 1

            if n_wall_cells > 0:
                ac_x_wal = -self.c_wal * sum_x_wal / n_wall_cells
                ac_y_wal = -self.c_wal * sum_y_wal / n_wall_cells
            else:
                ac_x_wal = 0.0
                ac_y_wal = 0.0

        else:
            ac_x_tar = 0.0
            ac_y_tar = 0.0
            ac_x_wal = 0.0
            ac_y_wal = 0.0

        # TOTAL ACCELERATION ----------------------------------------------------------------------

        ac_x = ac_x_ali + ac_x_coh + ac_x_sep + ac_x_tar + ac_x_wal
        ac_y = ac_y_ali + ac_y_coh + ac_y_sep + ac_y_tar + ac_y_wal
        ac = math.sqrt(ac_x**2 + ac_y**2)

        ac_data = Accelerations()
        ac_data.ac_x_ali = ac_x_ali
        ac_data.ac_y_ali = ac_y_ali
        ac_data.ac_x_coh = ac_x_coh
        ac_data.ac_y_coh = ac_y_coh
        ac_data.ac_x_sep = ac_x_sep
        ac_data.ac_y_sep = ac_y_sep
        ac_data.ac_x_tar = ac_x_tar
        ac_data.ac_y_tar = ac_y_tar
        ac_data.ac_x_wal = ac_x_wal
        ac_data.ac_y_wal = ac_y_wal
        ac_data.ac_x = ac_x
        ac_data.ac_y = ac_y
        ac_data.ac = ac

        self.publisher_ac.publish(ac_data)

        return ac_x, ac_y



def main(args=None):
    rclpy.init(args=args)
    node = BoidBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()