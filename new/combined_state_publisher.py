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
        
        # 设置发布频率
        self.publish_rate = rospy.Rate(30)  # 设置为30Hz
        
        # 存储最新的状态
        self.allegro_states = None
        self.ee_pose = None
        self.state_lock = threading.Lock()
        
        # 创建发布器 - 发布组合后的关节状态
        self.pub = rospy.Publisher('/combined_robot_state', JointState, queue_size=10)
        
        # 订阅器
        self.sub_allegro = rospy.Subscriber(
            '/allegroHand/joint_states', 
            JointState, 
            self.allegro_cb,
            queue_size=10
        )
        self.sub_ee_pose = rospy.Subscriber(
            'end_effector_actual_pose', 
            JointState, 
            self.ee_pose_cb,
            queue_size=10
        )
        
        # 启动发布线程
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo("组合状态发布器已初始化")

    def allegro_cb(self, msg):
        """Allegro手关节状态的回调函数"""
        with self.state_lock:
            self.allegro_states = msg

    def ee_pose_cb(self, msg):
        """末端执行器位姿的回调函数"""
        with self.state_lock:
            self.ee_pose = msg

    def publish_loop(self):
        """固定频率发布状态"""
        while not rospy.is_shutdown():
            self.publish_combined_states()
            self.publish_rate.sleep()

    def publish_combined_states(self):
        """发布组合后的状态"""
        with self.state_lock:
            if not all([self.allegro_states, self.ee_pose]):
                return

            # 创建新的关节状态消息
            combined_msg = JointState()
            combined_msg.header.stamp = rospy.Time.now()
            combined_msg.header.frame_id = "world"
            
            # 合并名称和位置数据
            combined_msg.name = list(self.ee_pose.name) + list(self.allegro_states.name)
            combined_msg.position = list(self.ee_pose.position) + list(self.allegro_states.position)

            # 发布组合后的消息
            self.pub.publish(combined_msg)

if __name__ == '__main__':
    try:
        node = CombinedStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
