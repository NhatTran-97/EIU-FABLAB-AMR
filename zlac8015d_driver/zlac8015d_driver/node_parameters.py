from rclpy.node import Node

class NodeParameters:
    def __init__(self, node: Node):
        node.get_logger().info('Initializing parameters for ZLAC8015D motor drivers')

        node.declare_parameter(name='connection_type', value='rtu')
        node.declare_parameter(name='modbus_port', value='/dev/zlac_8015d')  # zlac_8015d / ttyUSB0
        node.declare_parameter(name='modbus_baudrate', value=115200)
        node.declare_parameter(name='joint_state_topic', value='/nhatbot/JointState')
        node.declare_parameter(name='wheel_rotation_topic', value='/nhatbot/wheel_rotational_vel')
        node.declare_parameter(name='zlac_status_topic', value='/nhatbot/zlac_status')
        node.declare_parameter(name='reset_encoder_service', value='/nhatbot/reset_feedback_position')
        node.declare_parameter(name='joint_state_frequency', value=0.1)
        node.declare_parameter(name='zlac_status_frequency', value=1.0)
        node.declare_parameter(name='set_accel_time', value=1000)
        node.declare_parameter(name='set_decel_time', value=1000)
        node.declare_parameter(name='set_operation_mode', value=3)
        node.declare_parameter(name='set_stop_rpm', value=0)
        node.declare_parameter(name='set_clear_wheel_encoders', value=3)
        node.declare_parameter(name='ignore_small_speed_threshold', value=0.5)
        node.declare_parameter(name='max_rpm', value=200)
        node.declare_parameter(name='travel_in_one_rev', value=0.336)
        node.declare_parameter(name='cpr', value=4096)
        node.declare_parameter(name='wheel_radius', value=0.0535)

        try:
            self.connection_type = node.get_parameter('connection_type').get_parameter_value().string_value
            self.modbus_port = node.get_parameter('modbus_port').get_parameter_value().string_value
            self.modbus_baudrate = node.get_parameter('modbus_baudrate').get_parameter_value().integer_value
            self.joint_state_topic = node.get_parameter('joint_state_topic').get_parameter_value().string_value
            self.wheel_rotation_topic = node.get_parameter('wheel_rotation_topic').get_parameter_value().string_value
            self.zlac_status_topic = node.get_parameter('zlac_status_topic').get_parameter_value().string_value
            self.reset_encoder_service = node.get_parameter('reset_encoder_service').get_parameter_value().string_value
            self.joint_state_frequency = node.get_parameter('joint_state_frequency').get_parameter_value().double_value
            self.zlac_status_frequency = node.get_parameter('zlac_status_frequency').get_parameter_value().double_value
            self.set_accel_time = node.get_parameter('set_accel_time').get_parameter_value().integer_value
            self.set_decel_time = node.get_parameter('set_decel_time').get_parameter_value().integer_value
            self.set_operation_mode = node.get_parameter('set_operation_mode').get_parameter_value().integer_value
            self.set_stop_rpm = node.get_parameter('set_stop_rpm').get_parameter_value().integer_value
            self.set_clear_wheel_encoders = node.get_parameter('set_clear_wheel_encoders').get_parameter_value().integer_value
            self.ignore_small_speed_threshold = node.get_parameter('ignore_small_speed_threshold').get_parameter_value().double_value
            self.max_rpm = node.get_parameter('max_rpm').get_parameter_value().integer_value
            self.travel_in_one_rev = node.get_parameter('travel_in_one_rev').get_parameter_value().double_value
            self.cpr = node.get_parameter('cpr').get_parameter_value().integer_value
            self.wheel_radius = node.get_parameter('wheel_radius').get_parameter_value().double_value


        except Exception as e:
            node.get_logger().warn('Could not get parameters...setting variables to default')
            node.get_logger().warn('Error: "%s"' % e)


