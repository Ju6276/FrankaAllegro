#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf
import tf2_geometry_msgs

class EndEffectorPosePublisher:
    def __init__(self):
        rospy.init_node('end_effector_pose_publisher')
        
        # 创建位姿发布器
        self.actual_pose_pub = rospy.Publisher(
            "end_effector_actual_pose", 
            JointState, 
            queue_size=1
        )
        
        # 初始化TF监听器（同时支持tf和tf2）
        self.tf_listener = tf.TransformListener()
        
        # 等待TF树准备就绪
        rospy.sleep(1.0)  # 等待1秒确保TF树已经建立
        
        # 设置发布频率
        self.rate = rospy.Rate(100)  # 100Hz
        
        rospy.loginfo("末端执行器位姿发布器已初始化")
        
        # 开始发布位姿
        self.publish_pose_loop()

    def publish_pose_loop(self):
        """循环发布末端执行器位姿"""
        while not rospy.is_shutdown():
            try:
                # 获取变换
                (trans, rot) = self.tf_listener.lookupTransform(
                    'panda_link0',    # 基座坐标系
                    'franka_flange',  # 末端法兰坐标系
                    rospy.Time(0)
                )
                
                # 创建PoseStamped消息
                current_pose = PoseStamped()
                current_pose.header.frame_id = "panda_link0"
                current_pose.header.stamp = rospy.Time.now()
                
                # 设置位置和方向
                current_pose.pose.position.x = trans[0]
                current_pose.pose.position.y = trans[1]
                current_pose.pose.position.z = trans[2]
                current_pose.pose.orientation.x = rot[0]
                current_pose.pose.orientation.y = rot[1]
                current_pose.pose.orientation.z = rot[2]
                current_pose.pose.orientation.w = rot[3]
                
                # 转换为JointState格式并发布
                self.publish_pose(current_pose)
                
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(1, f"获取TF转换失败: {str(e)}")
                
            self.rate.sleep()

    def publish_pose(self, pose):
        """发布位姿信息"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = "panda_link0"
        
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

def main():
    try:
        publisher = EndEffectorPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
