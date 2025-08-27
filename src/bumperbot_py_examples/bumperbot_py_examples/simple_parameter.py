import rclpy
from rclpy.node import Node


class SimpleParameterNode(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_str_param", "bumperbot")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"simple_int_param changed to {param.value}")
                result.successful = True

            if param.name == "simple_str_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"simple_str_param changed to {param.value}")
                result.successful = True

        return result
    
def main():
    rclpy.init()
    node = SimpleParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()