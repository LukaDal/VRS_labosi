import rclpy
import time
import math
import matplotlib.pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sphero_interfaces.msg import Accelerations

class Grapher(Node):
    def __init__(self):
        super().__init__('grapher')

        self.declare_parameter('n_boids', 6)
        self.n_boids = self.get_parameter('n_boids').value
        self.declare_parameter('ac_boid_id', 1)
        self.ac_boid_id = self.get_parameter('ac_boid_id').value

        self.declare_parameter('fov_r', 3)
        self.fov_r = self.get_parameter('fov_r').value
        self.declare_parameter('fov_ang', (math.pi*2)*1)
        self.fov_ang = self.get_parameter('fov_ang').value
        self.declare_parameter('c_ali', 1/3)
        self.c_ali = self.get_parameter('c_ali').value
        self.declare_parameter('c_coh', 1/3)
        self.c_coh = self.get_parameter('c_coh').value
        self.declare_parameter('c_sep', 1/3)
        self.c_sep = self.get_parameter('c_sep').value

        # Start time
        self.start_time = time.monotonic()

        # Data
        self.odom_data = {}
        self.cmd_data = {}
        self.ac_data = {}

        for i in range(self.n_boids):
            self.odom_data[i] = {
                'time': [],
                'vx': [],
                'vy': []
            }

            self.cmd_data[i] = {
                'time': [],
                'cmd_vel_x': [],
                'cmd_vel_y': []
            }

            self.ac_data[i] = {
                'time': [],
                'ac_ali_x': [],
                'ac_ali_y': [],
                'ac_coh_x': [],
                'ac_coh_y': [],
                'ac_sep_x': [],
                'ac_sep_y': [],
                'ac_x': [],
                'ac_y': []
            }
    
        self.odom_subs = []
        self.cmd_subs = []
        self.ac_subs = []

        # Create subscribers
        for i in range(self.n_boids):

            topic_odom = f'robot_{i}/odom'
            sub_odom = self.create_subscription(
                Odometry,
                topic_odom,
                lambda msg, rid=i: self.odom_callback(msg, rid),
                10
            )
            self.odom_subs.append(sub_odom)

            topic_cmd = f'robot_{i}/cmd_vel'
            sub_cmd = self.create_subscription(
                Twist,
                topic_cmd,
                lambda msg, rid=i: self.cmd_callback(msg, rid),
                10
            )
            self.cmd_subs.append(sub_cmd)

            topic_ac = f'robot_{i}/ac'
            sub_ac = self.create_subscription(
                Accelerations,
                topic_ac,
                lambda msg, rid=i: self.ac_callback(msg, rid),
                10
            )
            self.ac_subs.append(sub_ac)

        # Kill all subs and begin drawing
        self.timer = self.create_timer(45, self.make_plots)


    def get_time(self):
        return time.monotonic() - self.start_time

    def odom_callback(self, msg, robot_id):
        data = self.odom_data[robot_id]
        data['time'].append(self.get_time())
        data['vx'].append(msg.twist.twist.linear.x)
        data['vy'].append(msg.twist.twist.linear.y)

    def cmd_callback(self, msg, robot_id):
        data = self.cmd_data[robot_id]
        data['time'].append(self.get_time())
        data['cmd_vel_x'].append(msg.linear.x)
        data['cmd_vel_y'].append(msg.linear.y)

    def ac_callback(self, msg, robot_id):
        data = self.ac_data[robot_id]
        data['time'].append(self.get_time())
        data['ac_ali_x'].append(msg.acc_x_ali)
        data['ac_ali_y'].append(msg.acc_y_ali)
        data['ac_coh_x'].append(msg.acc_x_coh)
        data['ac_coh_y'].append(msg.acc_y_coh)
        data['ac_sep_x'].append(msg.acc_x_sep)
        data['ac_sep_y'].append(msg.acc_y_sep)
        data['ac_x'].append(msg.acc_x)
        data['ac_y'].append(msg.acc_y)

    def stop_subscriptions(self):
        for sub in self.odom_subs:
            self.destroy_subscription(sub)
        for sub in self.cmd_subs:
            self.destroy_subscription(sub)
        for sub in self.ac_subs:
            self.destroy_subscription(sub)

    def make_plots(self):
        self.get_logger().info('Process Generating plots...')
        self.stop_subscriptions()
        self.destroy_timer(self.timer)

        # Select acceleration data only for the chosen boid
        ac = self.ac_data[self.ac_boid_id]

        # Create six plots
        fig, axes = plt.subplots(3, 2, figsize=(16, 12))

        fov_r = round(self.fov_r, 2)
        fov_ang = round(self.fov_ang, 2)
        c_ali = round(self.c_ali, 2)
        c_coh = round(self.c_coh, 2)
        c_sep = round(self.c_sep, 2)

        fig.suptitle(f'r = {fov_r}, FOV = {fov_ang}, c_ali = {c_ali}, c_coh = {c_coh}, c_sep = {c_sep}', fontsize=16, fontweight='bold')

        # 1. Acceleration X
        ax = axes[0, 0]
        ax.plot(ac['time'], ac['ac_sep_x'], label='Separation')
        ax.plot(ac['time'], ac['ac_ali_x'], label='Alignment')
        ax.plot(ac['time'], ac['ac_coh_x'], label='Cohesion')
        ax.plot(ac['time'], ac['ac_x'], label='Combined')
        ax.set_title(f'Boid {self.ac_boid_id} - Acceleration X')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration')
        ax.legend()
        ax.grid()

        # 2. Acceleration Y
        ax = axes[0, 1]
        ax.plot(ac['time'], ac['ac_sep_y'], label='Separation')
        ax.plot(ac['time'], ac['ac_ali_y'], label='Alignment')
        ax.plot(ac['time'], ac['ac_coh_y'], label='Cohesion')
        ax.plot(ac['time'], ac['ac_y'], label='Combined')
        ax.set_title(f'Boid {self.ac_boid_id} - Acceleration Y')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration')
        ax.legend()
        ax.grid()

        # 3. Command velocity X
        ax = axes[1, 0]
        for i in range(self.n_boids):
            data = self.cmd_data[i]
            ax.plot(
                data['time'],
                data['cmd_vel_x'],
                label=f'Boid {i}'
            )
        ax.set_title('Command Velocity X')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.legend()
        ax.grid()

        # 4. Command velocity Y
        ax = axes[1, 1]
        for i in range(self.n_boids):
            data = self.cmd_data[i]
            ax.plot(
                data['time'],
                data['cmd_vel_y'],
                label=f'Boid {i}'
            )
        ax.set_title('Command Velocity Y')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.legend()
        ax.grid()

        # 5. Actual velocity X
        ax = axes[2, 0]
        for i in range(self.n_boids):
            data = self.odom_data[i]
            ax.plot(
                data['time'],
                data['vx'],
                label=f'Boid {i}'
            )
        ax.set_title('Actual Velocity X')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.legend()
        ax.grid()

        # 6. Actual velocity Y
        ax = axes[2, 1]
        for i in range(self.n_boids):
            data = self.odom_data[i]
            ax.plot(
                data['time'],
                data['vy'],
                label=f'Boid {i}'
            )
        ax.set_title('Actual Velocity Y')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.legend()
        ax.grid()

        # Layout and display
        plt.tight_layout(rect=[0, 0, 1, 0.94])
        plt.show()
        self.get_logger().info('Plotting complete.')


def main(args=None):
    rclpy.init(args=args)
    node = Grapher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()