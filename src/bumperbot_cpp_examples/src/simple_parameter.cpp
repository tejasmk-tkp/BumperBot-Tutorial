#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node {
    public:
        SimpleParameter() : Node("simple_parameter") {
            declare_parameter<int>("simple_int_param", 28);
            declare_parameter<std::string>("simple_str_param", "bumperbot");

            add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
        }

    private:
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & parameters) {
            
            rcl_interfaces::msg::SetParametersResult result;

            for (const auto & param : parameters) {
                if (param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    RCLCPP_INFO(get_logger(), "Changing simple_int_param from %d to %d", param.get_value<int>(), param.get_value<int>());
                    result.successful = true;
                }

                if (param.get_name() == "simple_str_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    RCLCPP_INFO(get_logger(), "Changing simple_str_param from %s to %s", param.get_value<std::string>().c_str(), param.get_value<std::string>().c_str());
                    result.successful = true;
                }
            }
            return result;
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}