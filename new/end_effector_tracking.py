#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from moveit_commander import MoveGroupCommander, RobotCommander
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_multiply, quaternion_from_euler
import threading
import math
import numpy as np
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState

## 末端执行器追踪器（python版本）
## 追踪末端执行器的位置和方向（使用TF树）
class EndEffectorTracker:
    def __init__(self):
        rospy.init_node('end_effector_tracking')
        
        # 初始化互斥锁和位姿数据
        self.pose_lock = threading.Lock()
        self.wrist_pose = None
        
        # 创建MoveIt接口
        self.move_group = MoveGroupCommander("arm")
        
        # 创建订阅者和发布者
        self.wrist_pose_sub = rospy.Subscriber(
            "/natnet_ros/RigidBody001/pose", 
            PoseStamped, 
            self.wrist_pose_callback
        )
        
        self.arrival_pub = rospy.Publisher(
            "end_effector_arrival", 
            String, 
            queue_size=10
        )
        
        # 添加新的发布者
        self.actual_pose_pub = rospy.Publisher(
            "end_effector_actual_pose", 
            JointState, 
            queue_size=1
        )
        
        # 初始化TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 轨迹可视化发布器
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            queue_size=20
        )
        
        # 添加位姿发布线程
        self.pose_thread = threading.Thread(target=self.pose_publisher_thread)
        self.pose_thread.daemon = True
        self.pose_thread.start()

    def wrist_pose_callback(self, msg):
        """手腕位置的回调函数"""
        with self.pose_lock:
            self.wrist_pose = msg

    def pose_publisher_thread(self):
        """专门用于发布末端执行器位姿的线程"""
        rate = rospy.Rate(100)  # 100Hz
        
        while not rospy.is_shutdown():
            try:
                # 直接从TF树获取末端执行器位姿
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "world",
                        "franka_flange",
                        rospy.Time(0)
                    )
                    
                    # 创建PoseStamped消息
                    current_pose = PoseStamped()
                    current_pose.header.frame_id = "world"
                    current_pose.header.stamp = rospy.Time.now()
                    
                    # 从transform中获取位置和方向
                    current_pose.pose.position.x = transform.transform.translation.x
                    current_pose.pose.position.y = transform.transform.translation.y
                    current_pose.pose.position.z = transform.transform.translation.z
                    current_pose.pose.orientation = transform.transform.rotation
                    
                    # 发布位姿
                    self.publish_pose(current_pose)
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn_throttle(1, f"获取TF转换失败: {str(e)}")
                    
            except Exception as e:
                rospy.logwarn_throttle(1, f"位姿发布错误: {str(e)}")
                
            rate.sleep()

    def publish_pose(self, pose):
        """发布位姿信息"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = "world"  # 明确指定坐标系
        
        # 定义关节名称
        joint_state.name = ['ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw']
        
        # 设置位置值
        joint_state.position = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        
        # 发布消息
        self.actual_pose_pub.publish(joint_state)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            with self.pose_lock:
                current_wrist_pose = self.wrist_pose
            
            if current_wrist_pose is None:
                rospy.logwarn("No valid wrist position received yet.")
                rate.sleep()
                continue

            try:
                # 获取坐标转换
                transform = self.tf_buffer.lookup_transform(
                    "world",
                    current_wrist_pose.header.frame_id,
                    rospy.Time(0)
                )
                current_wrist_pose = tf2_geometry_msgs.do_transform_pose(
                    current_wrist_pose, 
                    transform
                )
                
                # 应用y轴偏移
                current_wrist_pose.pose.position.y -= 0.5
                
                # 应用旋转（绕z轴旋转90度）
                rotation = quaternion_from_euler(0, 0, math.pi/2)
                current_quat = [
                    current_wrist_pose.pose.orientation.x,
                    current_wrist_pose.pose.orientation.y,
                    current_wrist_pose.pose.orientation.z,
                    current_wrist_pose.pose.orientation.w
                ]
                new_quat = quaternion_multiply(rotation, current_quat)
                
                current_wrist_pose.pose.orientation.x = new_quat[0]
                current_wrist_pose.pose.orientation.y = new_quat[1]
                current_wrist_pose.pose.orientation.z = new_quat[2]
                current_wrist_pose.pose.orientation.w = new_quat[3]
                
                # 设置目标位置并执行
                self.move_group.set_pose_target(current_wrist_pose, "franka_flange")
                
                # 获取规划并打印返回值类型和内容
                plan_result = self.move_group.plan()
                # rospy.loginfo(f"Plan result type: {type(plan_result)}")
                # rospy.loginfo(f"Plan result: {plan_result}")

                # 解析返回值：(success, trajectory, planning_time)
                success = plan_result[0]
                trajectory = plan_result[1]

                if success:
                    # 打印规划的目标位置
                    try:
                        joint_values = trajectory.joint_trajectory.points[-1].positions
                        rospy.loginfo("Planned joint target positions:")
                        for i, value in enumerate(joint_values):
                            rospy.loginfo(f"Joint {i}: {value:.4f}")

                        # 执行运动
                        success = self.move_group.execute(trajectory, wait=True)
                        
                        if success:
                            # 获取实际执行后的位置
                            current_pose = self.move_group.get_current_pose("franka_flange")
                            
                            # 发布实际位置
                            self.publish_pose(current_pose)
                            
                            # 原有的日志信息
                            rospy.loginfo("Actual position after execution:")
                            rospy.loginfo(f"End-effector position: x={current_pose.pose.position.x:.4f}, "
                                         f"y={current_pose.pose.position.y:.4f}, "
                                         f"z={current_pose.pose.position.z:.4f}")
                            rospy.loginfo(f"End-effector orientation (quaternion): x={current_pose.pose.orientation.x:.4f}, "
                                         f"y={current_pose.pose.orientation.y:.4f}, "
                                         f"z={current_pose.pose.orientation.z:.4f}, "
                                         f"w={current_pose.pose.orientation.w:.4f}")
                            
                            self.arrival_pub.publish("End-effector has reached the target position.")
                    except (AttributeError, IndexError) as e:
                        rospy.logwarn(f"Failed to get joint values: {e}")
                else:
                    rospy.logwarn("Failed to plan to wrist position.")
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(str(e))
                
            rate.sleep()

if __name__ == '__main__':
    try:
        tracker = EndEffectorTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass 