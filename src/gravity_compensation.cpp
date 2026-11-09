#include "advrf_ros2_interfaces/msg/motor_tx_pdo.hpp"
#include "advrf_ros2_interfaces/msg/motor_tx_pdo_vector.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo_vector.hpp>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>
#include <xbot2_interface/xbotinterface2.h>

using JointState = sensor_msgs::msg::JointState;
using MotorCommands = advrf_ros2_interfaces::msg::MotorTxPdoVector;
using MotorCommand = advrf_ros2_interfaces::msg::MotorTxPdo;

class GravityCompensation : public rclcpp::Node {
public:
    GravityCompensation() : Node("gravity_compensation") 
    {
        const auto urdf = declare_parameter<std::string>("robot_description", "");
        const auto motor_ids = declare_parameter<std::vector<int64_t>>("motor_ids", {});
        const auto gains = declare_parameter<std::vector<double>>("gains", {0.0, 0.0, 0.0, 0.0, 0.0});
    
        if (urdf.empty())
            throw std::invalid_argument("robot_description is empty");

        model_ = XBot::ModelInterface::getModel(urdf, "pin");
        if (!model_)
            throw std::runtime_error("Could not create XBot2 Model.");

        publisher_ = create_publisher<MotorCommands>("tx/motors", rclcpp::QoS(1).reliable());
        subscription_ = create_subscription<JointState>("rx/joint_states", rclcpp::SensorDataQoS(), 
            [this](JointState::ConstSharedPtr msg) {
                update(*msg);
            }
        );    

        RCLCPP_INFO(get_logger(), "Gravity Compensation Node: ready!");
    }


private:

    void update(const JointState& msg) {}


    XBot::ModelInterface::Ptr model_;
    XBot::v2::JointNameMap q_;

    rclcpp::Publisher<MotorCommands>::SharedPtr publisher_;
    rclcpp::Subscription<JointState>::SharedPtr subscription_;

    std::vector<std::string> joints_;
    std::map<std::string, int32_t> motor_ids_;
    std::array<float, 5> gains_{};

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<GravityCompensation>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("gravity_compensation"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}