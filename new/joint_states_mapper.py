#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

## 将Allegro手关节状态映射到FrankaAllegro关节状态
## 在movelt中显示完整FrankaAllegro模型可视化
class JointStateMapper:
    def __init__(self):
        rospy.init_node('joint_states_mapper')
        
        # 发布到/joint_states
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 订阅/allegroHand/joint_states
        self.sub = rospy.Subscriber('/allegroHand/joint_states', JointState, self.callback)
        
        # 创建映射字典
        self.mapping = {}
        for i in range(16):
            self.mapping[f'joint_{i}.0'] = f'allegro_joint_{i}'

    def callback(self, msg):
        # 创建新的JointState消息
        new_msg = JointState()
        new_msg.header = Header()
        new_msg.header.stamp = rospy.Time.now()
        
        # 映射并添加allegro关节
        new_msg.name = [self.mapping[old_name] for old_name in msg.name]
        
        # 复制位置、速度和力矩数据(如果存在)（通常不需要？）
        if msg.position:
            new_msg.position = msg.position
            
        if msg.velocity:
            new_msg.velocity = msg.velocity
            
        if msg.effort:
            new_msg.effort = msg.effort
        
        # 发布新消息
        self.pub.publish(new_msg)

if __name__ == '__main__':
    try:
        mapper = JointStateMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 