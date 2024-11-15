#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <mutex>
#include <bio_ik/bio_ik.h>
#include <boost/program_options.hpp>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <yaml-cpp/yaml.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex> 
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

// Converts Eigen vectors to TF2 vectors
tf2::Vector3 toTF(const Eigen::Vector3d &v) {
    return tf2::Vector3(v.x(), v.y(), v.z());
}

// 在main函数开始处添加home位置的关节角度
std::vector<double> home_position = {0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0};
std::vector<std::string> joint_names = {
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7"
};

// 在文件开头添加Franka的速度限制常量
const double max_joint_velocity = 1.0;  // rad/s, 应小于硬件限制
const double max_joint_acceleration = 5.0;  // rad/s^2, 应小于硬件限制
const double max_cartesian_velocity = 0.5;  // m/s
const double max_cartesian_acceleration = 2.0;  // m/s^2
const double max_angular_velocity = 0.5;  // rad/s

// 添加一个函数用于回到home位置
void moveToHome(const moveit::core::RobotStatePtr& current_state,
               const ros::Publisher& joint_command_pub,
               const moveit::core::JointModelGroup* joint_model_group,
               const std::vector<double>& home_position,
               double frequency) {
    trajectory_msgs::JointTrajectory joint_command;
    joint_command.joint_names = joint_names;
    
    // 获取当前位置
    std::vector<double> current_positions;
    for (const auto& joint_name : joint_names) {
        current_positions.push_back(current_state->getVariablePosition(joint_name));
    }
    
    // 计算总移动时间(基于最大速度和加速度)
    double max_diff = 0.0;
    for (size_t i = 0; i < joint_names.size(); i++) {
        max_diff = std::max(max_diff, 
                           std::abs(home_position[i] - current_positions[i]));
    }
    double total_time = std::max(2.0, max_diff / max_joint_velocity);
    int steps = static_cast<int>(total_time * frequency);
    
    // 计算每个关节的最大允许速度
    std::vector<double> joint_velocities(7, 0.0);
    for (size_t i = 0; i < 7; i++) {
        double diff = std::abs(home_position[i] - current_positions[i]);
        joint_velocities[i] = std::min(diff * frequency, max_joint_velocity);
    }
    
    // 生成轨迹点
    for (int i = 0; i <= steps; i++) {
        trajectory_msgs::JointTrajectoryPoint waypoint;
        double t = static_cast<double>(i) / steps;
        
        // 使用五次多项式插值实现平滑轨迹
        double scale = 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
        
        for (size_t j = 0; j < joint_names.size(); j++) {
            double pos = current_positions[j] + 
                        scale * (home_position[j] - current_positions[j]);
            waypoint.positions.push_back(pos);
            
            // 添加速度加速度约束
            double vel = 0.0;
            double acc = 0.0;
            if (i > 0 && i < steps) {
                vel = (30 * std::pow(t, 2) - 60 * std::pow(t, 3) + 30 * std::pow(t, 4)) * 
                      (home_position[j] - current_positions[j]) / total_time;
                acc = (60 * t - 180 * std::pow(t, 2) + 120 * std::pow(t, 3)) * 
                      (home_position[j] - current_positions[j]) / std::pow(total_time, 2);
            }
            waypoint.velocities.push_back(vel);
            waypoint.accelerations.push_back(acc);
        }
        
        waypoint.time_from_start = ros::Duration(t * total_time);
        joint_command.points.push_back(waypoint);
    }
    
    // 检查轨迹有效性
    if (joint_command.points.empty()) {
        ROS_ERROR("生成的home轨迹无效!");
        return;
    }
    
    // 在轨迹点中设置速度限制
    for (auto& point : joint_command.points) {
        point.velocities = joint_velocities;
        // 确保加速度也在限制范围内
        std::vector<double> accelerations(7, max_joint_acceleration);
        point.accelerations = accelerations;
    }
    
    // 发布轨迹
    joint_command_pub.publish(joint_command);
    ROS_INFO("Moving to home position... (预计用时: %.2f秒)", total_time);
}

// 在文件开头添加结构体定义
struct InitialPoses {
    Eigen::Affine3d optitrack_pose;
    Eigen::Affine3d flange_pose;
    bool is_initialized = false;
} initial_poses;

// 在main函数前添加服务回调
std::mutex waiting_mutex;
std::mutex control_mutex;
bool waiting_for_initial_pose = true;

bool resetInitialPose(std_srvs::Empty::Request& req, 
                     std_srvs::Empty::Response& res) {
    std::lock_guard<std::mutex> lock(waiting_mutex);
    waiting_for_initial_pose = true;
    initial_poses.is_initialized = false;
    ROS_INFO("重置初始位姿捕捉");
    return true;
}

// 在main函数前添加可视化辅助函数
void publishVisualization(
    tf2_ros::TransformBroadcaster& broadcaster,
    const ros::Publisher& marker_pub,
    const Eigen::Affine3d& optitrack_pose,
    const Eigen::Affine3d& target_pose,
    const std::string& frame_id = "world") {
    
    // 发布OptiTrack位姿TF
    geometry_msgs::TransformStamped optitrack_tf;
    optitrack_tf.header.stamp = ros::Time::now();
    optitrack_tf.header.frame_id = frame_id;
    optitrack_tf.child_frame_id = "optitrack_marker";
    
    optitrack_tf.transform.translation.x = optitrack_pose.translation().x();
    optitrack_tf.transform.translation.y = optitrack_pose.translation().y();
    optitrack_tf.transform.translation.z = optitrack_pose.translation().z();
    
    // 修复Eigen矩阵操作
    Eigen::Matrix3d rot_matrix = optitrack_pose.rotation();
    Eigen::Quaterniond q_optitrack(rot_matrix);
    optitrack_tf.transform.rotation.w = q_optitrack.w();
    optitrack_tf.transform.rotation.x = q_optitrack.x();
    optitrack_tf.transform.rotation.y = q_optitrack.y();
    optitrack_tf.transform.rotation.z = q_optitrack.z();
    
    // 发布目标位姿TF
    geometry_msgs::TransformStamped target_tf;
    target_tf.header.stamp = ros::Time::now();
    target_tf.header.frame_id = frame_id;
    target_tf.child_frame_id = "target_flange";
    
    target_tf.transform.translation.x = target_pose.translation().x();
    target_tf.transform.translation.y = target_pose.translation().y();
    target_tf.transform.translation.z = target_pose.translation().z();
    
    Eigen::Quaterniond q_target(target_pose.rotation());
    target_tf.transform.rotation.w = q_target.w();
    target_tf.transform.rotation.x = q_target.x();
    target_tf.transform.rotation.y = q_target.y();
    target_tf.transform.rotation.z = q_target.z();
    
    broadcaster.sendTransform(optitrack_tf);
    broadcaster.sendTransform(target_tf);
    
    // 发布坐标轴标记
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "coordinate_axes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;  // 箭头长度
    marker.scale.y = 0.01; // 箭头宽度
    marker.scale.z = 0.01; // 箭头高度
    
    // 为OptiTrack和目标位置分别发布三个轴的箭头
    std::vector<Eigen::Affine3d> poses = {optitrack_pose, target_pose};
    std::vector<std::string> names = {"optitrack", "target"};
    
    for(size_t i = 0; i < poses.size(); i++) {
        for(int axis = 0; axis < 3; axis++) {
            marker.id = i * 3 + axis;
            marker.pose.position.x = poses[i].translation().x();
            marker.pose.position.y = poses[i].translation().y();
            marker.pose.position.z = poses[i].translation().z();
            
            // 设置不同轴的颜色
            marker.color.a = 1.0;
            marker.color.r = (axis == 0) ? 1.0 : 0.0;
            marker.color.g = (axis == 1) ? 1.0 : 0.0;
            marker.color.b = (axis == 2) ? 1.0 : 0.0;
            
            // 修改: 使用正确的方式获取旋转矩阵的列
            Eigen::Vector3d direction;
            if (axis == 0) {
                direction = poses[i].rotation().col(0);
            } else if (axis == 1) {
                direction = poses[i].rotation().col(1);
            } else {
                direction = poses[i].rotation().col(2);
            }
            
            // 计算方向的四元数
            Eigen::Quaterniond q;
            if(direction.norm() > 1e-6) {
                Eigen::Vector3d z = direction.normalized();
                Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitY());
                if(x.norm() < 1e-6) {
                    x = z.cross(Eigen::Vector3d::UnitZ());
                }
                x.normalize();
                Eigen::Vector3d y = z.cross(x);
                Eigen::Matrix3d rot;
                rot.col(0) = x;
                rot.col(1) = y;
                rot.col(2) = z;
                q = Eigen::Quaterniond(rot);
            } else {
                q = Eigen::Quaterniond::Identity();
            }
            
            marker.pose.orientation.w = q.w();
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            
            marker_pub.publish(marker);
        }
    }
}

// 整合所有安全检查到一个函数中
bool isMovementSafe(Eigen::Vector3d& target_position,
                   const Eigen::Vector3d& previous_position,
                   const Eigen::Vector3d& workspace_center,
                   const Eigen::Vector3d& workspace_size) {
    
    // 1. 检查目标是否有效
    if (!target_position.allFinite()) {
        ROS_WARN_THROTTLE(1.0, "接收到无效目标位置");
        return false;
    }

    // 2. 只检查位置变化
    double position_diff = (target_position - previous_position).norm();
    if (position_diff > 0.5) {  // 50cm
        ROS_WARN("检测到大幅度运动! 位置差: %.3fm", position_diff);
        return false;
    }

    // 3. 工作空间检查
    bool in_workspace = 
        target_position.x() >= workspace_center.x() - workspace_size.x() * 0.5 &&
        target_position.x() <= workspace_center.x() + workspace_size.x() * 0.5 &&
        target_position.y() >= workspace_center.y() - workspace_size.y() * 0.5 &&
        target_position.y() <= workspace_center.y() + workspace_size.y() * 0.5 &&
        target_position.z() >= workspace_center.z() - workspace_size.z() * 0.5 &&
        target_position.z() <= workspace_center.z() + workspace_size.z() * 0.5;

    if (!in_workspace) {
        ROS_WARN("目标位置超出工作空间");
        return false;
    }

    // 4. 检查是否接近工作空间边界
    double boundary_threshold = 0.05; // 5cm的边界警告区域
    bool near_boundary = false;
    
    if (std::abs(target_position.x() - (workspace_center.x() + workspace_size.x() * 0.5)) < boundary_threshold ||
        std::abs(target_position.x() - (workspace_center.x() - workspace_size.x() * 0.5)) < boundary_threshold ||
        std::abs(target_position.y() - (workspace_center.y() + workspace_size.y() * 0.5)) < boundary_threshold ||
        std::abs(target_position.y() - (workspace_center.y() - workspace_size.y() * 0.5)) < boundary_threshold ||
        std::abs(target_position.z() - (workspace_center.z() + workspace_size.z() * 0.5)) < boundary_threshold ||
        std::abs(target_position.z() - (workspace_center.z() - workspace_size.z() * 0.5)) < boundary_threshold) {
            
        ROS_WARN_THROTTLE(1.0, "TCP接近工作空间边界!");
        near_boundary = true;
    }

    // 5. 添加工作空间限制
    target_position.x() = std::min(std::max(target_position.x(), 
        workspace_center.x() - workspace_size.x() * 0.5),
        workspace_center.x() + workspace_size.x() * 0.5);
    
    target_position.y() = std::min(std::max(target_position.y(), 
        workspace_center.y() - workspace_size.y() * 0.5),
        workspace_center.y() + workspace_size.y() * 0.5);
    
    target_position.z() = std::min(std::max(target_position.z(), 
        workspace_center.z() - workspace_size.z() * 0.5),
        workspace_center.z() + workspace_size.z() * 0.5);

    return true;
}

// 在主循环外部定义一个函数来发布工作空间可视化
void publishWorkspaceVisualization(
    const ros::Publisher& workspace_viz_pub,
    const Eigen::Vector3d& workspace_center,
    const Eigen::Vector3d& workspace_size) {
    
    visualization_msgs::Marker workspace_marker;
    workspace_marker.header.frame_id = "world";
    workspace_marker.header.stamp = ros::Time::now();
    workspace_marker.ns = "workspace";
    workspace_marker.id = 0;
    workspace_marker.type = visualization_msgs::Marker::CUBE;
    workspace_marker.action = visualization_msgs::Marker::ADD;

    // 设置工作空间大小
    workspace_marker.scale.x = workspace_size.x();
    workspace_marker.scale.y = workspace_size.y();
    workspace_marker.scale.z = workspace_size.z();

    // 设置工作空间位置
    workspace_marker.pose.position.x = workspace_center.x();
    workspace_marker.pose.position.y = workspace_center.y();
    workspace_marker.pose.position.z = workspace_center.z();

    // 设置方向（默认无旋转）
    workspace_marker.pose.orientation.w = 1.0;
    workspace_marker.pose.orientation.x = 0.0;
    workspace_marker.pose.orientation.y = 0.0;
    workspace_marker.pose.orientation.z = 0.0;

    // 设置颜色和透明度
    workspace_marker.color.r = 0.1;  // 淡蓝色
    workspace_marker.color.g = 0.1;
    workspace_marker.color.b = 0.8;
    workspace_marker.color.a = 0.3;  // 70%透明

    workspace_marker.lifetime = ros::Duration(0.0);
    workspace_marker.frame_locked = true;

    // 添加调试信息
    ROS_INFO("Publishing workspace visualization at position: [%.2f, %.2f, %.2f], size: [%.2f, %.2f, %.2f]",
             workspace_center.x(), workspace_center.y(), workspace_center.z(),
             workspace_size.x(), workspace_size.y(), workspace_size.z());

    workspace_viz_pub.publish(workspace_marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "franka_tracking_control");
    ros::NodeHandle node_handle("~");  
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // Publisher for the joint command
    ros::Publisher joint_command_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

    // Publisher for flange position (for visualization purposes)
    ros::Publisher flange_position_pub = node_handle.advertise<geometry_msgs::PoseStamped>("flange_position", 1);

    // Create a TransformListener to listen to the transforms
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Initialize variables for the hand pose and mutex for thread safety
    Eigen::Affine3d hand_pose = Eigen::Affine3d::Identity();
    std::mutex hand_pose_mutex;

    // Initialize MoveIt for IK and planning
    std::string group_name = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setMaxVelocityScalingFactor(0.1);
    auto robot_model = move_group.getRobotModel();
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    auto joint_model_group = robot_model->getJointModelGroup(group_name);

    // Set workspace center and size for constraints
    Eigen::Vector3d workspace_center(0.555, 0.0, 0.614);
    Eigen::Vector3d workspace_size(0.4, 0.4, 0.3);
    node_handle.getParam("workspace_center/x", workspace_center.x());
    node_handle.getParam("workspace_center/y", workspace_center.y());
    node_handle.getParam("workspace_center/z", workspace_center.z());
    node_handle.getParam("workspace_size/x", workspace_size.x());
    node_handle.getParam("workspace_size/y", workspace_size.y());
    node_handle.getParam("workspace_size/z", workspace_size.z());

    // Set control parameters
    int frequency = 30;
    ros::Rate rate(frequency);
    double max_position_velocity = 0.1;
    double max_acceleration = 0.1;
    double max_rotation_velocity = 0.1;

    // Variables to store the previous goal state for smooth control
    Eigen::Vector3d previous_goal_position = workspace_center;
    Eigen::Quaterniond previous_goal_rotation_q(1, 0, 0, 0);
    Eigen::Vector3d previous_position_velocity = Eigen::Vector3d::Zero();

    // 在main函数开始处添加这些声明
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Subscribe to OptiTrack pose data
    ros::Subscriber sub_pose = node_handle.subscribe<geometry_msgs::PoseStamped>(
        "/natnet_ros/RigidBody001/pose", 1,
        [&tf_broadcaster, &marker_pub, &hand_pose, &hand_pose_mutex, &move_group](
            const geometry_msgs::PoseStamped::ConstPtr& rigidObjectMsg) {
            std::lock_guard<std::mutex> lock(hand_pose_mutex);
            if (!rigidObjectMsg) return;

            // 转换当前OptiTrack位姿到Eigen格式
            Eigen::Affine3d current_optitrack = Eigen::Translation3d(
                rigidObjectMsg->pose.position.x,
                rigidObjectMsg->pose.position.y,
                rigidObjectMsg->pose.position.z
            ) * Eigen::Quaterniond(
                rigidObjectMsg->pose.orientation.w,
                rigidObjectMsg->pose.orientation.x,
                rigidObjectMsg->pose.orientation.y,
                rigidObjectMsg->pose.orientation.z
            );

            if (waiting_for_initial_pose) {
                // 获取当前机器人末端位姿
                geometry_msgs::PoseStamped current_robot_pose = move_group.getCurrentPose();
                initial_poses.flange_pose = Eigen::Translation3d(
                    current_robot_pose.pose.position.x,
                    current_robot_pose.pose.position.y,
                    current_robot_pose.pose.position.z
                ) * Eigen::Quaterniond(
                    current_robot_pose.pose.orientation.w,
                    current_robot_pose.pose.orientation.x,
                    current_robot_pose.pose.orientation.y,
                    current_robot_pose.pose.orientation.z
                );
                
                initial_poses.optitrack_pose = current_optitrack;
                initial_poses.is_initialized = true;
                waiting_for_initial_pose = false;
                
                ROS_INFO("初始位姿已捕获");
                ROS_INFO_STREAM("OptiTrack初始位置: " << initial_poses.optitrack_pose.translation().transpose());
                ROS_INFO_STREAM("Flange初始位置: " << initial_poses.flange_pose.translation().transpose());
                
                // 发布初始位姿可视化
                publishVisualization(tf_broadcaster, marker_pub, 
                                   initial_poses.optitrack_pose, 
                                   initial_poses.flange_pose);
                return;
            } else {
                // 修改相对运动计算，只计算位置变化
                Eigen::Vector3d position_diff = current_optitrack.translation() - 
                                              initial_poses.optitrack_pose.translation();
                
                // 只更新位置，保持初始方向不变
                Eigen::Affine3d target_pose = initial_poses.flange_pose;
                target_pose.translation() = initial_poses.flange_pose.translation() + position_diff;
                
                // 打印调试信息
                ROS_INFO_STREAM_THROTTLE(1.0, "Position diff: " << position_diff.transpose());
                
                // 存储用于后续控制的位姿
                hand_pose = target_pose;
            }
        });

    // 添加新的变量声明
    const int MAX_FAILURES = 5;  // 大连续失败次数
    int consecutive_failures = 0;
    moveit::core::RobotStatePtr previous_robot_state;
    ros::Publisher workspace_viz_pub = node_handle.advertise<visualization_msgs::Marker>("/workspace_visualization", 10);
    ros::Publisher initial_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("initial_pose", 1);

    // 在主循环中添加定时发布
    ros::Time last_viz_time = ros::Time::now();
    double viz_publish_rate = 1.0; // 1Hz

    // Control loop
    while (ros::ok()) {
        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;

        {
            std::lock_guard<std::mutex> lock(hand_pose_mutex);
            target_position = hand_pose.translation();
            // 保持初始方向不变
            target_orientation = Eigen::Quaterniond(initial_poses.flange_pose.rotation());
        }

        // 1. 检查目标是否有效
        if (target_position.norm() == 0.0) {
            ROS_WARN_THROTTLE(1.0, "接收到无效目标位置，跳过此次控制");
            continue;
        }

        // 2. 检查大幅度运动
        double position_jump_threshold = 0.5;  // 50cm
        double angle_jump_threshold = 60.0 * M_PI / 180.0;  // 60度

        double position_diff = (target_position - previous_goal_position).norm();
        double angle_diff = previous_goal_rotation_q.angularDistance(target_orientation);

        if (position_diff > position_jump_threshold || angle_diff > angle_jump_threshold) {
            ROS_WARN("检测到大幅度运动! 位置差: %.3fm, 角度差: %.1f度", 
                     position_diff, angle_diff * 180.0 / M_PI);
            continue;
        }

        // 添加工作空间限制
        target_position.x() = std::min(std::max(target_position.x(), 
            workspace_center.x() - workspace_size.x() * 0.5),
            workspace_center.x() + workspace_size.x() * 0.5);
        
        target_position.y() = std::min(std::max(target_position.y(), 
            workspace_center.y() - workspace_size.y() * 0.5),
            workspace_center.y() + workspace_size.y() * 0.5);
        
        target_position.z() = std::min(std::max(target_position.z(), 
            workspace_center.z() - workspace_size.z() * 0.5),
            workspace_center.z() + workspace_size.z() * 0.5);
    // 检查目标是在工作空间边界附近
    bool near_boundary = false;
    double boundary_threshold = 0.05; // 5cm的边界警告区域
    
    if (std::abs(target_position.x() - (workspace_center.x() + workspace_size.x() * 0.5)) < boundary_threshold ||
        std::abs(target_position.x() - (workspace_center.x() - workspace_size.x() * 0.5)) < boundary_threshold ||
        std::abs(target_position.y() - (workspace_center.y() + workspace_size.y() * 0.5)) < boundary_threshold ||
        std::abs(target_position.y() - (workspace_center.y() - workspace_size.y() * 0.5)) < boundary_threshold ||
        std::abs(target_position.z() - (workspace_center.z() + workspace_size.z() * 0.5)) < boundary_threshold ||
        std::abs(target_position.z() - (workspace_center.z() - workspace_size.z() * 0.5)) < boundary_threshold) {
            
            ROS_WARN_THROTTLE(1.0, "TCP接近工作空间边界!");
            near_boundary = true;
        }
        // Smooth position control: limit velocity and acceleration
        Eigen::Vector3d position_velocity = (target_position - previous_goal_position) * frequency;
        if (position_velocity.norm() > max_position_velocity) {
            position_velocity = position_velocity.normalized() * max_position_velocity;
        }
        Eigen::Vector3d acceleration = (position_velocity - previous_position_velocity) * frequency;
        if (acceleration.norm() > max_acceleration) {
            acceleration = acceleration.normalized() * max_acceleration;
        }
        position_velocity = previous_position_velocity + acceleration * (1.0 / frequency);
        target_position = previous_goal_position + position_velocity * (1.0 / frequency);

        // Smooth orientation control: limit angular velocity
        double max_rotation_angle = max_rotation_velocity / frequency;
        double vel_factor = std::min(1.0, max_rotation_angle / angle_diff);
        target_orientation = previous_goal_rotation_q.slerp(vel_factor, target_orientation);

        // Perform IK computation with the updated target position and orientation
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        ik_options.replace = true;
        ik_options.return_approximate_solution = true;

        // 只添加位置目标
        ik_options.goals.emplace_back(
            new bio_ik::PositionGoal("franka_flange", toTF(target_position)));

        // 保持初始方向
        Eigen::Matrix3d rot_matrix = initial_poses.flange_pose.rotation();  // 先获取旋转矩阵
        Eigen::Quaterniond q(rot_matrix);  // 从旋转矩阵构造四元数
        ik_options.goals.emplace_back(
            new bio_ik::OrientationGoal("franka_flange", 
                tf2::Quaternion(q.x(),
                              q.y(),
                              q.z(),
                              q.w())));

        // 添加肘部约束 - 保持肘部朝下
        ik_options.goals.emplace_back(new bio_ik::SideGoal(
            "panda_link4",  // Franka的肘部链接
            tf2::Vector3(0, 0, -1),  // 期望方向
            tf2::Vector3(0, 0, 1)    // 避免方向
        ));

        // 添加腕部约束 - 限制腕部旋转
        ik_options.goals.emplace_back(new bio_ik::SideGoal(
            "panda_link7",  // Franka的腕部链接
            tf2::Vector3(0, 0, 1),   // 期望方向
            tf2::Vector3(0, 0, -1)   // 避免方向
        ));

        // 添加最小位移目标 - 使运动更平滑
        if (previous_robot_state) {
            ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal(0.1));
        }

        // 添加正则化目标 - 使姿态更自然
        ik_options.goals.emplace_back(new bio_ik::RegularizationGoal(0.1));

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

        // 在IK求解前添加碰撞检查回调
        moveit::core::GroupStateValidityCallbackFn state_validity_callback = 
            [&scene](robot_state::RobotState* state,
                    const robot_state::JointModelGroup* group,
                    const double* ik_solution) {
                state->setJointGroupPositions(group, ik_solution);
                state->update();
                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult res;
                scene->checkCollision(req, res, *state);
                if (res.collision) {
                    ROS_WARN_STREAM("检测到碰撞: " << res.contacts.begin()->first.first 
                                   << " 与 " << res.contacts.begin()->first.second);
                    return false;
                }
                return true;
            };

        // 修改setFromIK调用
        bool ik_success = current_state->setFromIK(
            joint_model_group,
            Eigen::Isometry3d(Eigen::Translation3d(target_position) * target_orientation),  // 使用正确的目标变换
            0.1,
            state_validity_callback
        );

        // If IK is successful, publish joint angles
        if (ik_success) {
            ROS_INFO_STREAM("IK solution found!");

            // Prepare the joint trajectory message
            trajectory_msgs::JointTrajectory joint_command;
            joint_command.joint_names = joint_model_group->getVariableNames();
            trajectory_msgs::JointTrajectoryPoint point;

            // 添加速度和加速度限制
            std::vector<double> velocities(joint_names.size(), 0.0);
            std::vector<double> accelerations(joint_names.size(), 0.0);
            
            for (const auto& joint_name : joint_model_group->getVariableNames()) {
                double joint_position = current_state->getVariablePosition(joint_name);
                point.positions.push_back(joint_position);
                
                // 如果一个状态，计算速度和加速度
                if (previous_robot_state) {
                    double prev_position = previous_robot_state->getVariablePosition(joint_name);
                    double velocity = (joint_position - prev_position) * frequency;
                    velocities.push_back(std::min(max_joint_velocity, std::abs(velocity)));
                    
                    // 简单的加速度估计
                    double acceleration = velocity * frequency;
                    accelerations.push_back(std::min(max_joint_acceleration, std::abs(acceleration)));
                }
            }

            point.velocities = velocities;
            point.accelerations = accelerations;
            point.time_from_start = ros::Duration(1.0 / frequency);
            joint_command.points.push_back(point);

            // 发布轨迹命令
            joint_command_pub.publish(joint_command);
            
            // 更新上一个状态
            previous_robot_state = current_state;

            ROS_INFO_STREAM("Published IK solution to /arm_controller/command.");

            // Update previous state variables
            previous_goal_position = target_position;
            previous_goal_rotation_q = target_orientation;
            previous_position_velocity = position_velocity;
        } else {
            ROS_WARN("IK求解失败");
            consecutive_failures++;
            // 2. 错误恢复
            if (consecutive_failures >= MAX_FAILURES) {
                ROS_ERROR("多次IK求解失败。回到home位置");
                moveToHome(current_state, joint_command_pub, joint_model_group, 
                          home_position, frequency);
                
                // 重置目标位置为home位置对应的TCP位置
                moveit::core::RobotState temp_state(robot_model);
                temp_state.setToDefaultValues();
                for (size_t i = 0; i < joint_names.size(); i++) {
                    temp_state.setJointPositions(joint_names[i], &home_position[i]);
                }
                temp_state.update();
                
                const Eigen::Isometry3d& home_tcp = 
                    temp_state.getGlobalLinkTransform("franka_flange");
                previous_goal_position = home_tcp.translation();
                previous_goal_rotation_q = Eigen::Quaterniond(home_tcp.rotation());
                
                consecutive_failures = 0;
                ros::Duration(2.0).sleep();  // 等待回到home位置
            }
        }

        ros::Time loop_start_time = ros::Time::now();
        rate.sleep();
        double loop_duration = (ros::Time::now() - loop_start_time).toSec();
        if (loop_duration > 1.0/frequency) {
            ROS_WARN_STREAM("控制循环延迟: " << loop_duration*1000 << "ms");
        }

        // 1. 发布目标位姿TF
        geometry_msgs::TransformStamped target_tf;
        target_tf.header.stamp = ros::Time::now();
        target_tf.header.frame_id = "world";
        target_tf.child_frame_id = "target_pose";
        target_tf.transform.translation.x = target_position.x();
        target_tf.transform.translation.y = target_position.y();
        target_tf.transform.translation.z = target_position.z();
        target_tf.transform.rotation.w = target_orientation.w();
        target_tf.transform.rotation.x = target_orientation.x();
        target_tf.transform.rotation.y = target_orientation.y();
        target_tf.transform.rotation.z = target_orientation.z();
        tf_broadcaster.sendTransform(target_tf);

        // 每秒发布一次工作空间可视化
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_viz_time).toSec() >= 1.0/viz_publish_rate) {
            publishWorkspaceVisualization(workspace_viz_pub, workspace_center, workspace_size);
            last_viz_time = current_time;
        }

        // 在控制循环末尾只需要调用：
        publishVisualization(tf_broadcaster, marker_pub, 
                            Eigen::Affine3d(Eigen::Translation3d(target_position) * target_orientation),
                            hand_pose);

        // 在控制循环中添加笛卡尔空间速度限制
        Eigen::Vector3d cartesian_velocity = (target_position - previous_goal_position) * frequency;
        double current_velocity = cartesian_velocity.norm();

        if (current_velocity > max_cartesian_velocity) {
            // 限制笛卡尔空间速度
            cartesian_velocity *= (max_cartesian_velocity / current_velocity);
            target_position = previous_goal_position + cartesian_velocity * (1.0 / frequency);
        }

        // 限制角速度
        double angular_velocity = target_orientation.angularDistance(previous_goal_rotation_q) * frequency;
        if (angular_velocity > max_angular_velocity) {
            double scale = max_angular_velocity / angular_velocity;
            target_orientation = previous_goal_rotation_q.slerp(scale, target_orientation);
        }
    }

    // 在main函数中添加服务
    ros::ServiceServer reset_service = 
        node_handle.advertiseService("reset_initial_pose", resetInitialPose);

    return 0;
}

