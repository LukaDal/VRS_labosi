import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class Hivemind(Node):
    def __init__(self):
        super().__init__('hivemind')

        # Active parameters being published
        self.fov_r = 1.0
        self.fov_ang = 2.0 # *pi, RQT displays only the coefficient
        self.max_vel = 2.0
        self.break_ac = 0.3

        self.c_ali = 1.0
        self.c_coh = 1.0
        self.c_sep = 1.0
        self.c_tar = 1.0
        self.c_wal = 1.0

        self.target_x = 0.0
        self.target_y = 0.0

        # Define a slider for FOV Radius
        slider_fov_r = ParameterDescriptor(
            description = 'Field of view range',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 5.0,
                    step = 0.01
                )])

        # Define a slider for FOV Angle
        slider_fov_ang = ParameterDescriptor(
            description = 'Field of view angle (*pi)',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 2.0,
                    step = 0.01
                )])

        # Define a slider for max velocity
        slider_max_vel = ParameterDescriptor(
            description = 'Maximum boid velocity',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 1.0,
                    to_value = 5.0,
                    step = 0.01
                )])
                
        # Define a slider for breaking acceleration
        slider_break_ac = ParameterDescriptor(
            description = 'Breaking acceleration (lower value -> greater slowdown)',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 2.0,
                    step = 0.01
                )])

        # Define a slider for Reynolds coefficients
        slider_param = ParameterDescriptor(
            description = 'Reynolds coefficient',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 5.0,
                    step = 0.01
                )])

        # Define swarm target input description
        coords_desc = ParameterDescriptor(description = 'Coordinates for swarm target')

        # RQT-side parameters
        self.declare_parameter('FOV_Radius',   self.fov_r,    slider_fov_r   )
        self.declare_parameter('FOV_Angle',    self.fov_ang,  slider_fov_ang )
        self.declare_parameter('Max_Velocity', self.max_vel,  slider_max_vel )
        self.declare_parameter('Breaking_ac',  self.break_ac, slider_break_ac)
        self.declare_parameter('CONFIRM_GENERAL_PARAMETERS', False)
        self.declare_parameter('c_Alignment',  self.c_ali, slider_param)
        self.declare_parameter('c_Cohesion',   self.c_coh, slider_param)
        self.declare_parameter('c_Separation', self.c_sep, slider_param)
        self.declare_parameter('c_TargetSeek', self.c_tar, slider_param)
        self.declare_parameter('c_WallDodge',  self.c_wal, slider_param)
        self.declare_parameter('CHANGE_REYNOLDS_PARAMETERS', False)
        self.declare_parameter('Target_X', self.target_x, coords_desc)
        self.declare_parameter('Target_Y', self.target_y, coords_desc)
        self.declare_parameter('CONFIRM_TARGET', False)

        # Publishers and timer
        self.param_pub_gen = self.create_publisher(Float64MultiArray, '/boid_params_gen', 10)
        self.param_pub_rey = self.create_publisher(Float64MultiArray, '/boid_params_rey', 10)
        self.target_pub = self.create_publisher(Float64MultiArray, '/boid_swarm_target', 10)
        self.timer = self.create_timer(0.1, self.publish_params)



    def publish_params(self):

        # WHEN the CONFIRM_GENERAL_PARAMETERS is PRESSED we update parameters once
        if self.get_parameter('CONFIRM_GENERAL_PARAMETERS').value:
            self.set_parameters([rclpy.parameter.Parameter('CONFIRM_GENERAL_PARAMETERS', rclpy.Parameter.Type.BOOL, False)])

            self.fov_r    = self.get_parameter('FOV_Radius').value
            self.fov_ang  = self.get_parameter('FOV_Angle').value * math.pi
            self.max_vel  = self.get_parameter('Max_Velocity').value
            self.break_ac = self.get_parameter('Breaking_ac').value

            msg = Float64MultiArray()
            msg.data = [self.fov_r, self.fov_ang, self.max_vel, self.break_ac]
            self.param_pub_gen.publish(msg)

        # WHILE the CHANGE_REYNOLDS_PARAMETERS is CHECKED we can change coefficients dynamically
        if self.get_parameter('CHANGE_REYNOLDS_PARAMETERS').value:

            self.c_ali = self.get_parameter('c_Alignment').value
            self.c_coh = self.get_parameter('c_Cohesion').value
            self.c_sep = self.get_parameter('c_Separation').value
            self.c_tar = self.get_parameter('c_TargetSeek').value
            self.c_wal = self.get_parameter('c_WallDodge').value

            msg = Float64MultiArray()
            msg.data = [self.c_ali, self.c_coh, self.c_sep, self.c_tar, self.c_wal]
            self.param_pub_rey.publish(msg)

        # WHEN the CONFIRM_TARGET is PRESSED we update swarm target once
        if self.get_parameter('CONFIRM_TARGET').value:
            self.set_parameters([rclpy.parameter.Parameter('CONFIRM_TARGET', rclpy.Parameter.Type.BOOL, False)])

            self.target_x = self.get_parameter('Target_X').value
            self.target_y = self.get_parameter('Target_Y').value

            msg = Float64MultiArray()
            msg.data = [self.target_x, self.target_y]
            self.target_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Hivemind()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()