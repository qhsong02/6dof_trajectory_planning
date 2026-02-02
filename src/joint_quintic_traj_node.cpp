#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

using std::placeholders::_1;

class JointQuinticTrajNode : public rclcpp::Node
{
  
private:
  std::vector<std::string> joint_names_;
  std::string joint_states_topic_;
  std::string target_topic_;
  std::string traj_pub_topic_;
  double T_{2.0};
  double dt_{0.02};
  bool include_vel_acc_{true};

  bool have_joint_state_{false};
  std::unordered_map<std::string, size_t> name_to_index_;
  std::vector<double> last_positions_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  
public:
  JointQuinticTrajNode(): Node("joint_quintic_traj_node")
  {
    // ---- params ----
    this->declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1","joint2","joint3","joint4","joint5","joint6"});
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    this->declare_parameter<std::string>("target_topic", "/target_joint_angles");
    this->declare_parameter<std::string>("traj_pub_topic", "/joint_trajectory_controller/joint_trajectory");

    this->declare_parameter<double>("duration_sec", 2.0);
    this->declare_parameter<double>("dt_sec", 0.02);
    this->declare_parameter<bool>("include_vel_acc", true);

    joint_names_ = this->get_parameter("joint_names").as_string_array();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    target_topic_ = this->get_parameter("target_topic").as_string();
    traj_pub_topic_ = this->get_parameter("traj_pub_topic").as_string();
    T_ = this->get_parameter("duration_sec").as_double();
    dt_ = this->get_parameter("dt_sec").as_double();
    include_vel_acc_ = this->get_parameter("include_vel_acc").as_bool();

    // ---- ros io ----
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JointQuinticTrajNode::on_joint_state, this, _1));

    target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      target_topic_, 10, std::bind(&JointQuinticTrajNode::on_target, this, _1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_pub_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Quintic traj planner started.");
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", traj_pub_topic_.c_str());
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    name_to_index_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      name_to_index_[msg->name[i]] = i;
    }
    last_positions_ = msg->position;
    have_joint_state_ = true;
  }

  void on_target(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!have_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "No /joint_states yet. Ignoring target.");
      return;
    }

    const size_t N = joint_names_.size();
    if (msg->data.size() != N) {
      RCLCPP_ERROR(this->get_logger(), "Target size mismatch: expected %zu, got %zu", N, msg->data.size());
      return;
    }

    std::vector<double> q0(N, 0.0), qf(N, 0.0);
    for (size_t j = 0; j < N; ++j) {
      auto it = name_to_index_.find(joint_names_[j]);
      if (it == name_to_index_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Joint '%s' not found in /joint_states name list.", joint_names_[j].c_str());
        return;
      }
      q0[j] = last_positions_[it->second];
      qf[j] = msg->data[j];
    }

    auto traj = make_quintic_trajectory(joint_names_, q0, qf, T_, dt_, include_vel_acc_);
    traj_pub_->publish(traj);

    RCLCPP_INFO(this->get_logger(), "Published trajectory: points=%zu, T=%.2f, dt=%.3f",
                traj.points.size(), T_, dt_);
  }

  static builtin_interfaces::msg::Duration to_duration(double t_sec)
  {
    builtin_interfaces::msg::Duration d;
    if (t_sec < 0.0) t_sec = 0.0;
    const int64_t sec = static_cast<int64_t>(std::floor(t_sec));
    int64_t nsec = static_cast<int64_t>(std::llround((t_sec - static_cast<double>(sec)) * 1e9));
    d.sec = static_cast<int32_t>(sec);
    d.nanosec = static_cast<uint32_t>(std::max<int64_t>(0, std::min<int64_t>(999999999, nsec)));
    return d;
  }

  //五次多项式参数求解 with qd(0)=qd(T)=0, qdd(0)=qdd(T)=0 
  static void coeff(double q0, double qf, double T,
                    double &a0, double &a1, double &a2, double &a3, double &a4, double &a5)
  {
    a0 = q0; a1 = 0.0; a2 = 0.0;
    const double dq = qf - q0;
    const double T2 = T*T;
    const double T3 = T2*T;
    const double T4 = T3*T;
    const double T5 = T4*T;
    a3 = 10.0 * dq / T3;
    a4 = -15.0 * dq / T4;
    a5 = 6.0 * dq / T5;
  }

  static void eval(double t, double a0, double a1, double a2, double a3, double a4, double a5,
                   double &q, double &qd, double &qdd)
  {
    const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    q   = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
    qd  = a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
    qdd = 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
  }

  static trajectory_msgs::msg::JointTrajectory make_quintic_trajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& q0,
    const std::vector<double>& qf,
    double T, double dt,
    bool include_vel_acc)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names;

    const size_t N = joint_names.size();
    if (T <= 0.0) T = 1.0;
    if (dt <= 0.0) dt = 0.02;

    std::vector<double> a0(N), a1(N), a2(N), a3(N), a4(N), a5(N);
    for (size_t j = 0; j < N; ++j) {
      coeff(q0[j], qf[j], T, a0[j], a1[j], a2[j], a3[j], a4[j], a5[j]);
    }

    const int steps = static_cast<int>(std::ceil(T / dt));
    traj.points.reserve(static_cast<size_t>(steps + 1));

    for (int k = 0; k <= steps; ++k) {
      const double t = std::min(T, k * dt);

      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.time_from_start = to_duration(t);
      p.positions.resize(N);

      if (include_vel_acc) {
        p.velocities.resize(N);
        p.accelerations.resize(N);
      }

      for (size_t j = 0; j < N; ++j) {
        double q, qd, qdd;
        eval(t, a0[j], a1[j], a2[j], a3[j], a4[j], a5[j], q, qd, qdd);
        p.positions[j] = q;
        if (include_vel_acc) {
          p.velocities[j] = qd;
          p.accelerations[j] = qdd;
        }
      }

      traj.points.push_back(std::move(p));
    }
    return traj;
  }


};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointQuinticTrajNode>();
  rclcpp::spin(node);
  //rclcpp::spin(std::make_shared<JointQuinticTrajNode>());
  rclcpp::shutdown();
  return 0;
}
