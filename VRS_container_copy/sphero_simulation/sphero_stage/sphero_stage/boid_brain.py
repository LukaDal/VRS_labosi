# Node control for each boid
import random
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sphero_interfaces.msg import Accelerations
from std_msgs.msg import Float64MultiArray

class BoidBrain(Node):
    def __init__(self):
        super().__init__('boid_brain')

        # Get ROS parameters
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').value
        self.declare_parameter('n_boids', 10)
        self.n_boids = self.get_parameter('n_boids').value
        self.declare_parameter('rand_seed', 1)
        self.rand_seed = self.get_parameter('rand_seed').value
        self.declare_parameter('fov_r', 3.0)
        self.fov_r = self.get_parameter('fov_r').value
        self.declare_parameter('fov_ang', math.pi*2)
        self.fov_ang = self.get_parameter('fov_ang').value
        self.declare_parameter('dt', 0.1)
        self.dt = self.get_parameter('dt').value

        # Reynolds coefficients and speed defaults (updated from Hivemind node and rqt)
        self.c_ali = 1.0
        self.c_coh = 1.0
        self.c_sep = 1.0
        self.c_tar = 1.0
        self.max_vel = 2.0
        self.break_ac = 0.4

        # Swarm target default (updated from Hivemind node and rqt)
        self.swarm_target_x = 0.0
        self.swarm_target_y = 0.0

        # Boid's current heading
        self.head_x = None
        self.head_y = None

        # Full odometry tracking
        self.odom_data = {}
        self.odom_subs = []

        # Create subscribers to other boid positions
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
        param_subscriber = self.create_subscription(Float64MultiArray, '/boid_parameters', self.param_callback, 10)
        target_subscriber = self.create_subscription(Point, '/swarm_target', self.target_callback, 10)

        # Create publishers
        topic_cmd_vel = f'robot_{self.robot_id}/cmd_vel'
        topic_ac = f'robot_{self.robot_id}/ac'
        self.publisher_vel = self.create_publisher(Twist, topic_cmd_vel, 10)
        self.publisher_ac = self.create_publisher(Accelerations, topic_ac, 10)

        # Initialize timer
        self.last_timestamp = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.controller)

        # Launch check
        self.get_logger().info(f'Boid {self.robot_id} online!')

        # Send random initial velocity
        rng = random.Random(self.rand_seed + self.robot_id)
        rand_vx_cmd = rng.uniform(-0.2, 0.2)
        rand_vy_cmd = rng.uniform(-0.2, 0.2)
        rand_vel_cmd = Twist()
        rand_vel_cmd.linear.x = rand_vx_cmd
        rand_vel_cmd.linear.y = rand_vy_cmd
        self.publisher_vel.publish(rand_vel_cmd)



    def odom_callback(self, msg, robot_id):
        self.odom_data[robot_id] = msg

    def param_callback(self, msg):
        self.c_ali = msg.data[0]
        self.c_coh = msg.data[1]
        self.c_sep = msg.data[2]
        self.c_tar = msg.data[3]
        self.max_vel = msg.data[4]
        self.break_ac = msg.data[5]

    def target_callback(self, msg):
        self.swarm_target_x = msg.x
        self.swarm_target_y = msg.y
        self.received_new_tar = True



    def controller(self):

        # Wait until we have all odometries
        if len(self.odom_data) < self.n_boids:
            return

        # Calculate elapsed time
        timestamp = self.get_clock().now()
        elapsed = (timestamp - self.last_timestamp).nanoseconds * (1e-9)
        self.last_timestamp = timestamp
        
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
            distance = math.hypot(dx, dy)

            # Fov check
            if (distance > self.fov_r) or (distance == 0):
                continue
            h_dot_d = self.head_x * dx + self.head_y * dy
            if h_dot_d / distance < math.cos(self.fov_ang / 2):
                continue

            visible_boids.append({
                'id': robot_id,
                'odom': odom,
                'dx': dx,
                'dy': dy,
                'distance': distance
            })

        return visible_boids



    def reynolds(self, visible_boids):

        x = self.odom_data[self.robot_id].pose.pose.position.x
        y = self.odom_data[self.robot_id].pose.pose.position.y
        vx = self.odom_data[self.robot_id].twist.twist.linear.x
        vy = self.odom_data[self.robot_id].twist.twist.linear.y
        n_vis = len(visible_boids)

        # Calculate needed sums and centroid
        sum_ali_x = 0.0
        sum_ali_y = 0.0
        sum_sep_x = 0.0
        sum_sep_y = 0.0
        cent_x = 0.0
        cent_y = 0.0
        for boid in visible_boids:
            sum_ali_x += boid['odom'].twist.twist.linear.x - vx
            sum_ali_y += boid['odom'].twist.twist.linear.y - vy
            sum_sep_x += (boid['dx'])/(boid['distance']**2)
            sum_sep_y += (boid['dy'])/(boid['distance']**2)
            cent_x += boid['odom'].pose.pose.position.x
            cent_y += boid['odom'].pose.pose.position.y
        if n_vis > 0:
            cent_x /= n_vis
            cent_y /= n_vis
        else:
            cent_x = x
            cent_y = y

        # Calculate acceleration components (basic Reynolds)
        ac_x_ali = self.c_ali * sum_ali_x
        ac_y_ali = self.c_ali * sum_ali_y
        ac_x_coh = self.c_coh * (cent_x - x)
        ac_y_coh = self.c_coh * (cent_y - y)
        ac_x_sep = -self.c_sep * sum_sep_x
        ac_y_sep = -self.c_sep * sum_sep_y

        # Calculate swarm target vector
        tar_dx = self.swarm_target_x - cent_x
        tar_dy = self.swarm_target_y - cent_y
        tar_d = math.hypot(tar_dx, tar_dy)

        # Target control
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

        # Calculate total accelerations
        ac_x = ac_x_ali + ac_x_coh + ac_x_sep + ac_x_tar
        ac_y = ac_y_ali + ac_y_coh + ac_y_sep + ac_y_tar
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