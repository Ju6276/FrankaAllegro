#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class JointStateSplitter:
    def __init__(self):
        rospy.init_node('joint_state_splitter')
        
        # 定义 Franka 和 Allegro 的关节名称
        self.franka_joints = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1',
            'panda_finger_joint2'
        ]
        
        self.allegro_joints = [
            'allegro_joint_0', 'allegro_joint_1', 'allegro_joint_2', 'allegro_joint_3',
            'allegro_joint_4', 'allegro_joint_5', 'allegro_joint_6', 'allegro_joint_7',
            'allegro_joint_8', 'allegro_joint_9', 'allegro_joint_10', 'allegro_joint_11',
            'allegro_joint_12', 'allegro_joint_13', 'allegro_joint_14', 'allegro_joint_15'
        ]
        
        # 创建发布者
        self.franka_pub = rospy.Publisher('franka_states', JointState, queue_size=10)
        self.allegro_pub = rospy.Publisher('allegro_states', JointState, queue_size=10)
        
        # 订阅完整的关节状态
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)
        
    def callback(self, msg):
        # 创建 Franka 的关节状态消息
        franka_msg = JointState()
        franka_msg.header = msg.header
        
        # 创建 Allegro 的关节状态消息
        allegro_msg = JointState()
        allegro_msg.header = msg.header
        
        # 分离关节状态
        for i, name in enumerate(msg.name):
            if name in self.franka_joints:
                franka_msg.name.append(name)
                if msg.position: franka_msg.position.append(msg.position[i])
                if msg.velocity: franka_msg.velocity.append(msg.velocity[i])
                if msg.effort: franka_msg.effort.append(msg.effort[i])
            
            if name in self.allegro_joints:
                allegro_msg.name.append(name)
                if msg.position: allegro_msg.position.append(msg.position[i])
                if msg.velocity: allegro_msg.velocity.append(msg.velocity[i])
                if msg.effort: allegro_msg.effort.append(msg.effort[i])
        
        # 发布分离后的消息
        if franka_msg.name:
            self.franka_pub.publish(franka_msg)
        if allegro_msg.name:
            self.allegro_pub.publish(allegro_msg)

if __name__ == '__main__':
    try:
        splitter = JointStateSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 