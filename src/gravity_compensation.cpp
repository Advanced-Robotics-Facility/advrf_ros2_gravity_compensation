#include <rclcpp/rclcpp.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo_vector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <xbot2_interface/xbotinterface2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using JointState = sensor_msgs::msg::JointState;
using MotorCommand = advrf_ros2_interfaces::msg::MotorTxPdo;
using MotorCommands = advrf_ros2_interfaces::msg::MotorTxPdoVector;

class GravityCompensation : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;

  GravityCompensation() : Node("gravity_compensation")
  {
    // ROS2 Params
    const auto urdf = declare_parameter<std::string>("robot_description", "");
    joints_ = declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    const auto motor_ids = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});
    const auto joint_gains = declare_parameter<std::vector<double>>("joint_gains", std::vector<double>{});
    gain_ramp_duration_ = declare_parameter("gain_ramp_duration", 3.0);

    // Sanity Checks
    if (urdf.empty())
      throw std::invalid_argument("robot_description is empty");
    if (joints_.empty() || joints_.size() != motor_ids.size())
      throw std::invalid_argument("joints and motor_ids must have equal nonzero sizes");
    if (std::set<std::string>(joints_.begin(), joints_.end()).size() != joints_.size())
      throw std::invalid_argument("joints must contain unique names");
    if (joint_gains.size() != 5 * joints_.size())
      throw std::invalid_argument("joint_gains must contain five values per joint");
    if (!std::isfinite(gain_ramp_duration_) || gain_ramp_duration_ <= 0.0)
      throw std::invalid_argument("gain_ramp_duration must be finite and positive");

    std::set<int32_t> unique_ids;
    gains_.resize(joints_.size());
    for (std::size_t joint = 0; joint < joints_.size(); ++joint)
    {
      if (motor_ids[joint] < 0 || motor_ids[joint] > std::numeric_limits<int32_t>::max())
        throw std::invalid_argument("motor_ids must be nonnegative int32 values");

      const auto id = static_cast<int32_t>(motor_ids[joint]);
      if (!unique_ids.insert(id).second)
        throw std::invalid_argument("motor_ids must be unique");
      motor_ids_[joints_[joint]] = id;

      for (std::size_t gain = 0; gain < 5; ++gain) {
        const auto value = joint_gains[5 * joint + gain];
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
          throw std::invalid_argument("joint_gains must be finite float32 values");
        gains_[joint][gain] = static_cast<float>(value);
      }
    }

    // Model Initialization (Pinocchio backend - "pin")
    model_ = XBot::ModelInterface::getModel(urdf, "pin");
    if (!model_)
      throw std::runtime_error("Could not create the XBot model");

    const auto model_joints = model_->getJointNames();
    const std::set<std::string> model_joint_set(model_joints.begin(), model_joints.end());
    for (const auto &name : joints_)
      if (!model_joint_set.count(name))
        throw std::invalid_argument("Joint is absent from robot_description: " + name);
    for (const auto &name : model_joints)
      q_[name] = 0.0;

    // Pub/Sub Initialization (rx-JointState, tx-MotorTxPdoVector)
    publisher_ = create_publisher<MotorCommands>("tx/motors", rclcpp::QoS(1).reliable().durability_volatile());
    subscription_ = create_subscription<JointState>("rx/joint_states", rclcpp::SensorDataQoS(),
        [this](JointState::ConstSharedPtr message) { update(*message); }
    );

    current_gains_ = gains_;
    ramp_start_gains_ = gains_;
    ramp_target_gains_ = gains_;

    // Srv Initialization
    service_ = create_service<SetBool>("set_gravity_compensation",
        [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) {
          begin_gain_ramp(request->data);
          response->success = true;
          response->message = request->data
              ? "Reducing position gains"
              : "Restoring impedance gains";
        }
    );

    RCLCPP_INFO(get_logger(),
                "Gravity compensation ready for %zu joints (full gravity torque, impedance gains held)",
                joints_.size());
  }

private:
  // State Machine
  // - Hold: complete gains, tor_ref=0
  // - RampingToGravity: reduces gradually gain-0 (stiffness) and gain_1 (damping)
  // - Gravity: gain_0 and gain_1 are 0 and gravity compensation is active
  // - RampingToHold: restores gradually the gains
  enum class State { RampingToGravity, Gravity, RampingToHold, Hold };

  void begin_gain_ramp(bool enable)
  {
    ramp_start_gains_ = current_gains_;
    ramp_target_gains_ = gains_;
    if (enable) {
      for (auto &gain : ramp_target_gains_) {
        gain[0] = 0.0F;
        gain[1] = 0.0F;
      }
    }
    state_ = enable ? State::RampingToGravity : State::RampingToHold;
    ramp_start_ = std::chrono::steady_clock::now();
  }

  void update_gains()
  {
    if (state_ != State::RampingToGravity && state_ != State::RampingToHold)
      return;

    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_start_).count();
    const auto tau = std::clamp(elapsed / gain_ramp_duration_, 0.0, 1.0);
    const auto alpha = ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau; // quintic poly

    for (std::size_t joint = 0; joint < current_gains_.size(); ++joint)
      for (std::size_t gain = 0; gain < 5; ++gain)
        current_gains_[joint][gain] = static_cast<float>(
            ramp_start_gains_[joint][gain] + alpha *
            (ramp_target_gains_[joint][gain] - ramp_start_gains_[joint][gain])
        );

    if (tau >= 1.0) {
      current_gains_ = ramp_target_gains_;
      state_ = state_ == State::RampingToGravity ? State::Gravity : State::Hold;
      RCLCPP_INFO(get_logger(), "%s", state_ == State::Gravity
          ? "Gravity-compensation gains active" : "Impedance gains restored");
    }
  }

  void update(const JointState &message)
  {
    if (message.name.size() != message.position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected malformed JointState");
      return;
    }

    std::unordered_map<std::string, double> measured;
    measured.reserve(message.name.size());
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      if (!std::isfinite(message.position[i])) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected non-finite joint position");
        return;
      }
      measured[message.name[i]] = message.position[i];
    }

    for (const auto &joint : joints_) {
      const auto position = measured.find(joint);
      if (position == measured.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing feedback for joint %s", joint.c_str());
        return;
      }
      q_[joint] = position->second;
    }
    for (const auto &[name, position] : measured)
      if (q_.count(name))
        q_[name] = position;

    model_->setJointPositionMinimal(q_);
    model_->update();

    Eigen::VectorXd gravity;
    XBot::v2::JointNameMap gravity_by_name;
    model_->computeGravityCompensation(gravity);
    model_->vToMap(gravity, gravity_by_name);
    update_gains();

    MotorCommands output;
    output.data.reserve(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto &joint = joints_[i];
      const auto torque = state_ == State::Hold ? 0.0 : gravity_by_name.at(joint);
      if (!std::isfinite(torque)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Non-finite gravity torque for %s; command not published",
                              joint.c_str());
        return;
      }

      MotorCommand command;
      command.ecat_id = motor_ids_.at(joint);
      command.pos_ref = static_cast<float>(q_.at(joint));
      command.vel_ref = 0.0F;
      command.tor_ref = static_cast<float>(torque);
      command.gain_0 = current_gains_[i][0];
      command.gain_1 = current_gains_[i][1];
      command.gain_2 = current_gains_[i][2];
      command.gain_3 = current_gains_[i][3];
      command.gain_4 = current_gains_[i][4];
      output.data.push_back(command);
    }
    publisher_->publish(output);
  }

  XBot::ModelInterface::Ptr model_;
  XBot::v2::JointNameMap q_;

  std::vector<std::string> joints_;
  std::map<std::string, int32_t> motor_ids_;
  std::vector<std::array<float, 5>> gains_;
  std::vector<std::array<float, 5>> current_gains_;
  std::vector<std::array<float, 5>> ramp_start_gains_;
  std::vector<std::array<float, 5>> ramp_target_gains_;
  double gain_ramp_duration_{3.0};
  State state_{State::Hold};
  std::chrono::steady_clock::time_point ramp_start_;

  rclcpp::Publisher<MotorCommands>::SharedPtr publisher_;
  rclcpp::Subscription<JointState>::SharedPtr subscription_;
  rclcpp::Service<SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
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
