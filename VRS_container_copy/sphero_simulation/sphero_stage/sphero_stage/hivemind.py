import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange
from geometry_msgs.msg import Point

class Hivemind(Node):
    def __init__(self):
        super().__init__('hivemind')

        # Define a slider for Reynolds coefficients
        slider_param = ParameterDescriptor(
            description = 'Reynolds coefficient',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 5.0,
                    step = 0.01
                )
            ]
        )

        # Define a slider for max velocity
        slider_velocity = ParameterDescriptor(
            description = 'Maximum boid velocity',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 1.0,
                    to_value = 5.0,
                    step = 0.01
                )
            ]
        )

        # Define a slider for breaking acceleration
        slider_break_ac = ParameterDescriptor(
            description = 'Breaking acceleration (lower value -> greater slowdown)',
            floating_point_range = [ 
                FloatingPointRange(
                    from_value = 0.0,
                    to_value = 2.0,
                    step = 0.01
                )
            ]
        )

        # Define swarm target input description
        coords_desc = ParameterDescriptor(description = 'Coordinate for swarm target')

        # RQT-side parameters
        self.declare_parameter('rqt_c_ali', 1.0, slider_param)
        self.declare_parameter('rqt_c_coh', 1.0, slider_param)
        self.declare_parameter('rqt_c_sep', 1.0, slider_param)
        self.declare_parameter('rqt_c_tar', 1.0, slider_param)
        self.declare_parameter('rqt_max_vel', 2.0, slider_velocity)
        self.declare_parameter('rqt_break_ac', 0.4, slider_break_ac)
        self.declare_parameter('rqt_params_engage', False)
        self.declare_parameter('rqt_target_x', 0.0, coords_desc)
        self.declare_parameter('rqt_target_y', 0.0, coords_desc)
        self.declare_parameter('rqt_confirm_target', False)

        # Active parameters that will be published
        self.c_ali = 1.0
        self.c_coh = 1.0
        self.c_sep = 1.0
        self.c_tar = 1.0
        self.max_vel = 2.0
        self.break_ac = 0.4
        self.target = Point()
        self.target.x = 0.0
        self.target.y = 0.0
        self.target.z = 0.0

        # Publishers and timer
        self.param_publisher = self.create_publisher(Float64MultiArray, '/boid_parameters', 10)
        self.target_publisher = self.create_publisher(Point, '/swarm_target', 10)
        self.timer = self.create_timer(0.1, self.publish_params)


    def publish_params(self):

        # WHILE the coefficient change checkbox IS True we can change coefficients
        if self.get_parameter('rqt_params_engage').value:

            self.c_ali = self.get_parameter('rqt_c_ali').value
            self.c_coh = self.get_parameter('rqt_c_coh').value
            self.c_sep = self.get_parameter('rqt_c_sep').value
            self.c_tar = self.get_parameter('rqt_c_tar').value
            self.max_vel = self.get_parameter('rqt_max_vel').value
            self.break_ac = self.get_parameter('rqt_break_ac').value
            msg = Float64MultiArray()
            msg.data = [self.c_ali, self.c_coh, self.c_sep, self.c_tar, self.max_vel, self.break_ac]

            self.param_publisher.publish(msg)

        # WHEN the target change checkbox is pressed ONCE the target gets updated once
        if self.get_parameter('rqt_confirm_target').value:

            # Reset checkbox
            self.set_parameters([rclpy.parameter.Parameter('rqt_confirm_target', rclpy.Parameter.Type.BOOL, False)])

            self.target.x = self.get_parameter('rqt_target_x').value
            self.target.y = self.get_parameter('rqt_target_y').value

            self.target_publisher.publish(self.target)



def main(args=None):
    rclpy.init(args=args)
    node = Hivemind()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()