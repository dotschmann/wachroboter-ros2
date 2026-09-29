#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class WachroboterNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct Pose2D
  {
    double x;
    double y;
    double yaw;
  };

  WachroboterNode()
  : Node("wachroboter_node")
  {
    role_ = declare_parameter<std::string>("role", "guest");
    codeword_ = declare_parameter<std::string>("codeword", "OPEN123");
    expected_codeword_ = declare_parameter<std::string>("expected_codeword", "OPEN123");

    topic_codeword_ = declare_parameter<std::string>(
      "topic_codeword", "/wachroboter/codeword");
    topic_guard_status_ = declare_parameter<std::string>(
      "topic_guard_status", "/wachroboter/guard_status");
    topic_alarm_ = declare_parameter<std::string>(
      "topic_alarm", "/wachroboter/alarm");

    guard_position_ = read_pose("guard_position", -1.185, -0.100, -3.012);
    visitor_stop_ = read_pose("visitor_stop", -1.850, 1.442, -1.655);
    guard_clear_position_ = read_pose("guard_clear_position", -0.349, -0.731, -2.468);
    lager_ = read_pose("lager", 0.343, 1.442, -1.655);
    polizei_ = read_pose("polizei", -2.001, 6.053, 3.047);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");

    codeword_pub_ = create_publisher<std_msgs::msg::String>(topic_codeword_, 10);
    guard_status_pub_ = create_publisher<std_msgs::msg::String>(topic_guard_status_, 10);
    alarm_pub_ = create_publisher<std_msgs::msg::Bool>(topic_alarm_, 10);

    codeword_sub_ = create_subscription<std_msgs::msg::String>(
      topic_codeword_, 10,
      std::bind(&WachroboterNode::on_codeword, this, std::placeholders::_1));

    guard_status_sub_ = create_subscription<std_msgs::msg::String>(
      topic_guard_status_, 10,
      std::bind(&WachroboterNode::on_guard_status, this, std::placeholders::_1));

    startup_timer_ = create_wall_timer(
      1500ms,
      std::bind(&WachroboterNode::start_behavior, this));

    RCLCPP_INFO(get_logger(), "Wachroboter started with role='%s'", role_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Topics: codeword=%s guard_status=%s alarm=%s",
      topic_codeword_.c_str(),
      topic_guard_status_.c_str(),
      topic_alarm_.c_str());
  }

private:
  Pose2D read_pose(
    const std::string & prefix,
    double default_x,
    double default_y,
    double default_yaw)
  {
    Pose2D pose;
    pose.x = declare_parameter<double>(prefix + "_x", default_x);
    pose.y = declare_parameter<double>(prefix + "_y", default_y);
    pose.yaw = declare_parameter<double>(prefix + "_yaw", default_yaw);
    return pose;
  }

  void start_behavior()
  {
    if (startup_started_) {
      return;
    }

    startup_started_ = true;
    startup_timer_->cancel();

    if (role_ == "guard") {
      RCLCPP_INFO(get_logger(), "GUARD: moving to GUARD_POSITION");
      navigate_to(
        guard_position_,
        "GUARD_POSITION",
        [this](bool success) {
          if (!success) {
            RCLCPP_ERROR(get_logger(), "GUARD: failed to reach GUARD_POSITION");
            return;
          }

          guard_ready_ = true;
          publish_guard_status("GUARD_READY");

          guard_ready_timer_ = create_wall_timer(
            1s,
            [this]() {
              if (guard_ready_ && !code_processed_) {
                publish_guard_status("GUARD_READY");
              }
            });

          RCLCPP_INFO(get_logger(), "GUARD: ready and waiting for codeword");
        });
    } else if (role_ == "guest" || role_ == "visitor") {
      RCLCPP_INFO(get_logger(), "VISITOR: waiting for GUARD_READY");
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown role '%s'. Use guard or guest.", role_.c_str());
    }
  }

  void on_guard_status(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!(role_ == "guest" || role_ == "visitor")) {
      return;
    }

    const auto & status = msg->data;

    if (status == "GUARD_READY" && !visitor_started_) {
      visitor_started_ = true;

      RCLCPP_INFO(get_logger(), "VISITOR: GUARD_READY received");
      RCLCPP_INFO(get_logger(), "VISITOR: moving to VISITOR_STOP");

      navigate_to(
        visitor_stop_,
        "VISITOR_STOP",
        [this](bool success) {
          if (!success) {
            RCLCPP_ERROR(get_logger(), "VISITOR: failed to reach VISITOR_STOP");
            return;
          }

          std_msgs::msg::String msg;
          msg.data = codeword_;
          codeword_pub_->publish(msg);

          waiting_for_decision_ = true;
          RCLCPP_INFO(
            get_logger(),
            "VISITOR: codeword '%s' sent", codeword_.c_str());
        });

      return;
    }

    if (status == "PATH_CLEAR" && waiting_for_decision_ && !lager_started_) {
      lager_started_ = true;
      waiting_for_decision_ = false;

      RCLCPP_INFO(get_logger(), "VISITOR: PATH_CLEAR received");
      RCLCPP_INFO(get_logger(), "VISITOR: moving to LAGER");

      navigate_to(
        lager_,
        "LAGER",
        [this](bool success) {
          if (success) {
            RCLCPP_INFO(get_logger(), "VISITOR: reached LAGER");
          } else {
            RCLCPP_ERROR(get_logger(), "VISITOR: failed to reach LAGER");
          }
        });

      return;
    }

    if (status == "DENIED" && waiting_for_decision_) {
      waiting_for_decision_ = false;
      denied_ = true;

      RCLCPP_WARN(
        get_logger(),
        "VISITOR: DENIED received. Staying at VISITOR_STOP.");
    }
  }

  void on_codeword(const std_msgs::msg::String::SharedPtr msg)
  {
    if (role_ != "guard") {
      return;
    }

    if (!guard_ready_ || code_processed_) {
      return;
    }

    code_processed_ = true;
    guard_ready_ = false;

    if (guard_ready_timer_) {
      guard_ready_timer_->cancel();
    }

    RCLCPP_INFO(
      get_logger(),
      "GUARD: received codeword '%s'", msg->data.c_str());

    if (msg->data == expected_codeword_) {
      RCLCPP_INFO(get_logger(), "GUARD: codeword CORRECT");
      RCLCPP_INFO(get_logger(), "GUARD: moving to GUARD_CLEAR_POSITION");

      navigate_to(
        guard_clear_position_,
        "GUARD_CLEAR_POSITION",
        [this](bool success) {
          if (!success) {
            RCLCPP_ERROR(
              get_logger(),
              "GUARD: failed to reach GUARD_CLEAR_POSITION");
            return;
          }

          publish_guard_status("PATH_CLEAR");
          RCLCPP_INFO(get_logger(), "GUARD: PATH_CLEAR published");
        });
    } else {
      RCLCPP_WARN(get_logger(), "GUARD: codeword WRONG");

      publish_guard_status("DENIED");

      std_msgs::msg::Bool alarm;
      alarm.data = true;
      alarm_pub_->publish(alarm);

      RCLCPP_WARN(get_logger(), "GUARD: DENIED + alarm=true published");
      RCLCPP_INFO(get_logger(), "GUARD: moving to POLIZEI");

      navigate_to(
        polizei_,
        "POLIZEI",
        [this](bool success) {
          if (success) {
            RCLCPP_INFO(get_logger(), "GUARD: reached POLIZEI");
          } else {
            RCLCPP_ERROR(get_logger(), "GUARD: failed to reach POLIZEI");
          }
        });
    }
  }

  void publish_guard_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    guard_status_pub_->publish(msg);
  }

  void navigate_to(
    const Pose2D & pose,
    const std::string & label,
    std::function<void(bool)> done_cb)
  {
    if (!nav_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(
        get_logger(),
        "NavigateToPose action server unavailable for %s",
        label.c_str());
      done_cb(false);
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();

    goal.pose.pose.position.x = pose.x;
    goal.pose.pose.position.y = pose.y;
    goal.pose.pose.position.z = 0.0;

    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(pose.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(pose.yaw / 2.0);

    RCLCPP_INFO(
      get_logger(),
      "Navigate -> %s (x=%.3f y=%.3f yaw=%.3f)",
      label.c_str(),
      pose.x,
      pose.y,
      pose.yaw);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this, label](const GoalHandleNavigate::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(
            get_logger(),
            "Navigation goal rejected: %s", label.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Navigation goal accepted: %s", label.c_str());
        }
      };

    options.result_callback =
      [this, label, done_cb](
        const GoalHandleNavigate::WrappedResult & result) {
        const bool success =
          result.code == rclcpp_action::ResultCode::SUCCEEDED;

        if (success) {
          RCLCPP_INFO(
            get_logger(),
            "Navigation succeeded: %s", label.c_str());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "Navigation failed: %s (result code %d)",
            label.c_str(),
            static_cast<int>(result.code));
        }

        done_cb(success);
      };

    nav_client_->async_send_goal(goal, options);
  }

  std::string role_;
  std::string codeword_;
  std::string expected_codeword_;

  std::string topic_codeword_;
  std::string topic_guard_status_;
  std::string topic_alarm_;

  Pose2D guard_position_;
  Pose2D visitor_stop_;
  Pose2D guard_clear_position_;
  Pose2D lager_;
  Pose2D polizei_;

  bool startup_started_{false};
  bool guard_ready_{false};
  bool code_processed_{false};
  bool visitor_started_{false};
  bool waiting_for_decision_{false};
  bool lager_started_{false};
  bool denied_{false};

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr codeword_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guard_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr codeword_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_status_sub_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr guard_ready_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WachroboterNode>());
  rclcpp::shutdown();
  return 0;
}
