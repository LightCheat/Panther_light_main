// 使用该节点前，记得使用该命令切换控制器ros2 control switch_controllers   --deactivate panther_arm_controller   --activate panther_velocity_controller   --strict

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kinematics_application/forward_differential_kinematics.hpp"

class DifferentialKinematicsDemo 
{
public:
    // 构造函数
    explicit DifferentialKinematicsDemo(const rclcpp::Node::SharedPtr &node): node_(node), kinematics_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
    {
        joint_names_ = {
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        };

        // 创建关节位置状态订阅
        joint_state_sub_ =
            node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &DifferentialKinematicsDemo::jointStateCallback,
                    this,
                    std::placeholders::_1));

        // 创建预测（速度雅克比矩阵计算出的末端笛卡尔速度）、测量（末端link时间微分的速度）和误差（两者差值）的发布器
        predicted_pub_ =
            node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/predicted_cartesian_velocity", 10);

        measured_pub_ =
            node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/measured_cartesian_velocity", 10);

        error_pub_ =
            node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/cartesian_velocity_error", 10);

    }

private:
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 用于接收雅克比计算出的速度
        Eigen::VectorXd q_dot(6);

        // 如果无法从 /joint_states 中取得完整关节速度，则发出警告并返回
        if (!extractJointVelocities(*msg, q_dot)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "无法从 /joint_states 中取得完整关节速度");
            return;
        }

        // 之前的一个bug，在没有添加try异常处理时，第一次执行lookupTransfrom会由于没数据出现错误
        try {
            // 1. 雅克比计算速度，predicted = [vx, vy, vz, wx, wy, wz]
            Eigen::Matrix<double, 6, 1> predicted =
                    kinematics_.forwardDifferentialKinematics(q_dot);

            // 2. 通过tf2获取当前末端位姿
            auto tf = tf_buffer_.lookupTransform(
                    base_frame_,
                    tool_frame_,
                    tf2::TimePointZero);

            // 记录当前时间戳，用于微分计算
            rclcpp::Time current_stamp(tf.header.stamp);

            // 测量末端姿态，四元数要归一化处理
            Eigen::Vector3d current_position(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);

            Eigen::Quaterniond current_orientation(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z);

            current_orientation.normalize();

            // 用于第一帧没有数据时：第一帧没有上一帧可供比较，只保存数据。
            if (!has_previous_tf_) {
                    previous_position_ = current_position;
                    previous_orientation_ = current_orientation;
                    previous_stamp_ = current_stamp;
                    has_previous_tf_ = true;
                    return;
            }

            // 计算时间间隔dt
            double dt =
                    (current_stamp - previous_stamp_).seconds();

            // 如果 TF 还没有更新，不能进行微分。
            if (dt <= 1e-6) {
                    return;
            }

            // 3. 对位置进行微分，得到实际线速度。
            Eigen::Vector3d measured_linear =
                    (current_position - previous_position_) / dt;

            // 4. 用相邻两帧姿态差计算实际角速度。
            Eigen::Quaterniond delta_orientation =
                    current_orientation *
                    previous_orientation_.conjugate();

            // 使q 和 -q 表示相同姿态。并选择较短的旋转方向，避免跳变。
            if (delta_orientation.w() < 0.0) {
                    delta_orientation.coeffs() *= -1.0;
            }

            // 将四元数差转换为欧拉角后，再计算角速度
            Eigen::AngleAxisd angle_axis(delta_orientation);

            Eigen::Vector3d measured_angular =
                angle_axis.axis() *
                angle_axis.angle() / dt;

            // 组建测量的末端速度向量
            Eigen::Matrix<double, 6, 1> measured;

            measured.head<3>() = measured_linear;
            measured.tail<3>() = measured_angular;

            // 5. 计算预测速度和测量速度的误差
            Eigen::Matrix<double, 6, 1> error =
                    predicted - measured;

            // 6. 发布预测速度、测量速度和误差
            predicted_pub_->publish(
                makeTwistMessage(predicted, current_stamp));

            measured_pub_->publish(
                makeTwistMessage(measured, current_stamp));

            error_pub_->publish(
                makeTwistMessage(error, current_stamp));

            RCLCPP_INFO(
                node_->get_logger(),
                "线速度误差: %.2f m/s, 角速度误差: %.2f rad/s",
                error.head<3>().norm(),
                error.tail<3>().norm());

            // 7. 当前帧保存为下一次的上一帧。
            previous_position_ = current_position;
            previous_orientation_ = current_orientation;
            previous_stamp_ = current_stamp;
        }
        catch (const tf2::TransformException &exception) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "等待TF %s -> %s: %s",
                base_frame_.c_str(),
                tool_frame_.c_str(),
                exception.what());

            return;
        }
    }

    bool extractJointVelocities(
        const sensor_msgs::msg::JointState &msg,
        Eigen::VectorXd &q_dot)
    {
        /*
         * 不要假设 /joint_states 中的关节排列顺序固定。
         * 应通过关节名称寻找对应速度。
         */
        if (msg.name.size() != msg.velocity.size()) {
            return false;
        }

        std::unordered_map<std::string, double> velocity_map;

        for (std::size_t i = 0; i < msg.name.size(); ++i) {
            velocity_map[msg.name[i]] = msg.velocity[i];
        }

        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            auto iter = velocity_map.find(joint_names_[i]);

            if (iter == velocity_map.end()) {
                return false;
            }

            q_dot(static_cast<Eigen::Index>(i)) =
                iter->second;
        }

        return true;
    }

    // 发布笛卡尔速度类型消息
    geometry_msgs::msg::TwistStamped makeTwistMessage(
        const Eigen::Matrix<double, 6, 1> &velocity,
        const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TwistStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = base_frame_;

        msg.twist.linear.x = velocity(0);
        msg.twist.linear.y = velocity(1);
        msg.twist.linear.z = velocity(2);

        msg.twist.angular.x = velocity(3);
        msg.twist.angular.y = velocity(4);
        msg.twist.angular.z = velocity(5);

        return msg;
    }

    rclcpp::Node::SharedPtr node_;

    ForwardDifferentialKinematics kinematics_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped>::SharedPtr predicted_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped>::SharedPtr measured_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;

    std::vector<std::string> joint_names_;

    std::string base_frame_ = "base_link";
    std::string tool_frame_ = "link6";

    bool has_previous_tf_ = false;

    Eigen::Vector3d previous_position_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond previous_orientation_ =
        Eigen::Quaterniond::Identity();

    rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
};

// 节点主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "forward_differential_kinematics_demo1");

    auto demo =
        std::make_shared<DifferentialKinematicsDemo>(node);

    rclcpp::spin(node);

    demo.reset();
    node.reset();

    rclcpp::shutdown();

    return 0;
}