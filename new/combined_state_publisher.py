#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import threading

## 组合状态发布器
## 发布末端执行器位姿和Allegro手关节状态
class CombinedStatePublisher:
    def __init__(self):
        rospy.init_node('combined_state_publisher')
        
        # 存储最新的状态
        self.allegro_states = None
        self.ee_pose = None
        self.state_lock = threading.Lock()
        
        # 创建映射字典
        self.mapping = {}
        for i in range(16):
            self.mapping[f'joint_{i}.0'] = f'allegro_joint_{i}'
            
        # 创建发布器 - 发布组合后的关节状态
        self.pub = rospy.Publisher('/combined_robot_state', JointState, queue_size=1)
        
        # 订阅器
        self.sub_allegro = rospy.Subscriber('/allegroHand/joint_states', JointState, self.allegro_cb)
        self.sub_ee_pose = rospy.Subscriber('end_effector_actual_pose', JointState, self.ee_pose_cb)
        
        rospy.loginfo("组合状态发布器已初始化")

    def allegro_cb(self, msg):
        """Allegro手关节状态的回调函数"""
        with self.state_lock:
            self.allegro_states = msg
            self.publish_combined_states()

    def ee_pose_cb(self, msg):
        """末端执行器位姿的回调函数"""
        with self.state_lock:
            self.ee_pose = msg
            self.publish_combined_states()

    def publish_combined_states(self):
        """发布组合后的状态"""
        if not all([self.allegro_states, self.ee_pose]):
            return

        # 创建新的关节状态消息
        combined_msg = JointState()
        combined_msg.header.stamp = rospy.Time.now()
        combined_msg.header.frame_id = "world"
        
        # 添加末端执行器位姿数据
        combined_msg.name = list(self.ee_pose.name)
        combined_msg.position = list(self.ee_pose.position)

        # 添加Allegro手关节数据
        for old_name, pos in zip(self.allegro_states.name, self.allegro_states.position):
            new_name = self.mapping[old_name]
            combined_msg.name.append(new_name)
            combined_msg.position.append(pos)

        # 发布组合后的消息
        self.pub.publish(combined_msg)

if __name__ == '__main__':
    try:
        node = CombinedStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass