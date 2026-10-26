#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class JointVelocityPublisher : public rclcpp::Node
{
public:
    JointVelocityPublisher()
        : Node("joint_velocity_pub_node")
    {
        velocity_publisher_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                "/panther_velocity_controller/commands",
                10);

        /*
         * 发布周期。
         * 这里是 10 ms，即 100 Hz。
         */
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(
                &JointVelocityPublisher::timerCallback,
                this));

        /*
         * 保存轨迹开始时间。
         */
        start_time_ = now();

        RCLCPP_INFO(
            get_logger(),
            "关节速度发布节点已经启动");
    }

private:
    static constexpr std::size_t JOINT_COUNT = 6;

    void timerCallback()
    {
        /*
         * 从启动时刻开始经过的时间，单位为秒。
         */
        const double elapsed_time =
            (now() - start_time_).seconds();

        /*
         * 根据当前时间计算六轴速度。
         */
        const std::array<double, JOINT_COUNT> velocity =
            calculateVelocity(elapsed_time);

        publishVelocity(velocity);
    }

    std::array<double, JOINT_COUNT> calculateVelocity(
        double elapsed_time)
    {
        std::array<double, JOINT_COUNT> velocity{
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        /*
         * 在这里设计轨迹。
         *
         * velocity[0] 对应 joint1
         * velocity[1] 对应 joint2
         * velocity[2] 对应 joint3
         * velocity[3] 对应 joint4
         * velocity[4] 对应 joint5
         * velocity[5] 对应 joint6
         *
         * elapsed_time 是节点启动后经过的秒数。
         */

        // 每轮前1秒正转、后1秒反转，共执行3轮（总计6秒）。
        constexpr double speed = 0.5;
        constexpr double cycle_duration = 4.0;
        constexpr double total_duration = 60.0;

        if (elapsed_time < total_duration) {
            const double time_in_cycle =
                std::fmod(elapsed_time, cycle_duration);

            if (time_in_cycle < 2.0) {
                velocity[0] = speed;   // joint1
                velocity[4] = speed;   // joint5
            } else {
                velocity[0] = -speed;  // joint1
                velocity[4] = -speed;  // joint5
            }
        }

        return velocity;
    }

    void publishVelocity(
        const std::array<double, JOINT_COUNT> &velocity)
    {
        std_msgs::msg::Float64MultiArray message;

        message.data.assign(
            velocity.begin(),
            velocity.end());

        velocity_publisher_->publish(message);
    }

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        velocity_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<JointVelocityPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
