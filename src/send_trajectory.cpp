#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class TiagoTrajectoryPublisher : public rclcpp::Node {
public:
    TiagoTrajectoryPublisher() : Node("tiago_trajectory_publisher") {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&TiagoTrajectoryPublisher::publish_trajectory, this));
    }

private:
    void publish_trajectory() {
        auto message = trajectory_msgs::msg::JointTrajectory();

        // Define the joint names for the TIAGo arm
        message.joint_names = {
            "arm_1_joint", "arm_2_joint", "arm_3_joint", 
            "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
        };

        // Define four key points to form a square motion in joint space
        std::vector<std::vector<double>> square_positions = {
            {0.0, -0.5,  0.5, -1.5,  0.0,  1.2,  0.0},  // Start position
            {0.2, -0.3,  0.7, -1.3,  0.1,  1.0, -0.1},  // Move forward
            {0.4, -0.2,  0.6, -1.0,  0.3,  0.8, -0.2},  // Move right
            {0.2, -0.3,  0.7, -1.3,  0.1,  1.0, -0.1},  // Move back
            {0.0, -0.5,  0.5, -1.5,  0.0,  1.2,  0.0}   // Return to start
        };

        for (size_t i = 0; i < square_positions.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = square_positions[i];
            point.time_from_start = rclcpp::Duration::from_seconds((i + 1) * 3.0); // 3 sec per move
            message.points.push_back(point);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing square trajectory...");
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TiagoTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}

