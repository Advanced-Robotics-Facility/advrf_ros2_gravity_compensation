#include <rclcpp/rclcpp.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

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
using MotorCommands = advrf_ros2_interfaces::msg::MotorTxPdo;

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
    const auto gravity_damping = declare_parameter("gravity_damping", 0.0);
    hold_position_ = declare_parameter("hold_position", false);

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
    if (!std::isfinite(gravity_damping) || gravity_damping < 0.0 ||
        gravity_damping > std::numeric_limits<float>::max())
      throw std::invalid_argument("gravity_damping must be a nonnegative float32 value");
    gravity_damping_ = static_cast<float>(gravity_damping);

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

    // Model Initialization
    pinocchio::urdf::buildModelFromXML(urdf, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
    q_ = pinocchio::neutral(model_);

    for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(model_.njoints); ++id) {
      const auto &joint_model = model_.joints[id];
      if (joint_model.nq() == 1 && joint_model.nv() == 1) {
        model_joint_indices_.emplace(model_.names[id], JointIndices{joint_model.idx_q(), joint_model.idx_v()});
      }
    }

    for (const auto &name : joints_) {
      if (!model_.existJointName(name))
        throw std::invalid_argument("Joint is absent from robot_description: " + name);

      const auto id = model_.getJointId(name);
      const auto& joint_model = model_.joints[id];
      if (joint_model.nq() != 1 || joint_model.nv() != 1)
        throw std::invalid_argument("Only one-DoF joints are supported: " + name);
    }

    // Pub/Sub Initialization (rx-JointState, tx-MotorTxPdo)
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
              ? "Ramping to gravity compensation"
              : "Restoring impedance gains";
        }
    );

    RCLCPP_INFO(get_logger(),
                "Gravity compensation ready for %zu joints (full gravity torque, impedance gains held)",
                joints_.size());
  }

private:

  struct JointIndices {
    Eigen::Index idx_q;
    Eigen::Index idx_v;
  };

  // State Machine
  // - Hold: complete gains, gravity compensated with full stiffness/damping gains
  // - RampingToGravity: transitions from impedance gains to gravity-mode gains
  // - Gravity: zero stiffness with optional damping
  // - RampingToHold: restores gradually the gains
  enum class State { RampingToGravity, Gravity, RampingToHold, Hold };

  void begin_gain_ramp(bool enable)
  {
    ramp_start_gains_ = current_gains_;
    ramp_target_gains_ = gains_;
    if (enable) {
      for (auto &gain : ramp_target_gains_) {
        gain[0] = 0.0F;
        gain[1] = gravity_damping_;
      }
    }
    else if (hold_position_ && (state_ == State::Gravity || state_ == State::RampingToGravity)) {
      hold_positions_.clear();
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
    }

    for (const auto &[name, position] : measured)
      if (const auto indices = model_joint_indices_.find(name); indices != model_joint_indices_.end())
        q_[indices->second.idx_q] = position;

    if (hold_position_ && hold_positions_.empty())
      for (const auto &joint : joints_)
        hold_positions_.push_back(measured.at(joint));

    const auto &gravity = pinocchio::computeGeneralizedGravity(model_, *data_, q_);
    update_gains();

    MotorCommands output;
    output.header.stamp = now();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto &joint = joints_[i];
      const auto &indices = model_joint_indices_.at(joint);
      const auto torque = gravity[indices.idx_v];
      if (!std::isfinite(torque)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Non-finite gravity torque for %s; command not published",
                              joint.c_str());
        return;
      }

      output.ecat_id.push_back(motor_ids_.at(joint));
      const auto reference = hold_position_ ? hold_positions_[i] : q_[indices.idx_q];
      output.pos_ref.push_back(static_cast<float>(reference));
      output.vel_ref.push_back(0.0F);
      output.tor_ref.push_back(static_cast<float>(torque));
      output.gain_0.push_back(current_gains_[i][0]);
      output.gain_1.push_back(current_gains_[i][1]);
      output.gain_2.push_back(current_gains_[i][2]);
      output.gain_3.push_back(current_gains_[i][3]);
      output.gain_4.push_back(current_gains_[i][4]);
      output.cur_ref.push_back(0);
      output.fault_ack.push_back(0);
      output.ts.push_back(0);
      output.op_idx_aux.push_back(0);
      output.aux.push_back(0);
    }
    publisher_->publish(output);
  }

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_;
  std::unordered_map<std::string, JointIndices> model_joint_indices_;

  std::vector<std::string> joints_;
  std::map<std::string, int32_t> motor_ids_;
  std::vector<std::array<float, 5>> gains_;
  std::vector<std::array<float, 5>> current_gains_;
  std::vector<std::array<float, 5>> ramp_start_gains_;
  std::vector<std::array<float, 5>> ramp_target_gains_;
  double gain_ramp_duration_{3.0};
  float gravity_damping_{0.0F};
  bool hold_position_{false};
  std::vector<double> hold_positions_;
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
