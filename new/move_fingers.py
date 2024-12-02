#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def move_fingers_to_closed():
    # Init Ros node
    rospy.init_node('fingers_closed_position')

    # create Publisher，发布JointState消息到/allegroHand/commanded_joint_states
    pub = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=10)

    # msg joint state
    joint_state = JointState()
    
    # msg head
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()

    # 关节名
    joint_state.name = ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0',# 食指
                        'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0',# 中指
                        'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0',# 无名指
                        'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']# 拇指

    # 设置关闭状态的关节位置
    joint_state.position = [
        0.10999355416140008, 1.7384174400596923, 1.42467947276103, 1.2781769337539903,  # 食指
        0.10, 1.6273915850411134, 1.6285694291176303, 1.0637178941019823,  # 中指
        0.10, 1.609659818979084, 1.6125264236602763, 1.1006645808503066,  # 无名指
        1.4293128855527328, 0.5828161960396628, 0.5040733874145974, 1.264350815179784      # 拇指
    ]

    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # 更新时间戳
        joint_state.header.stamp = rospy.Time.now()
        
        # 发布消息
        pub.publish(joint_state)
        rospy.loginfo("发送关节位置命令...")
        
        # 按照设定的频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        move_fingers_to_closed()
    except rospy.ROSInterruptException:
        pass
