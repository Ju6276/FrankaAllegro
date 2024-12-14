#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import moveit_commander
import numpy as np
import threading

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # 初始化MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 设置机械臂组和末端执行器
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_end_effector_link("franka_flange")
        
        # 设置规划参数
        self.arm_group.set_planning_time(5)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        
        # AllegroHand发布器
        self.hand_pub = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=10)
        
        # 初始化Allegro手的设置
        self.setup_allegro_hand()
        
        # 订阅来自训练机器的预测动作
        rospy.Subscriber("/predicted_action", JointState, self.action_callback)
        
        rospy.loginfo("Robot controller initialized")
        
        self.executing = False  # 动作执行状态标志
        self.execution_lock = threading.Lock()
        
        # 添加执行完成的发布器
        self.execution_done_pub = rospy.Publisher('/execution_done', JointState, queue_size=1)

    def setup_allegro_hand(self):
        # 初始化关节状态，设置关节名和初始值
        self.joint_state = JointState()
        self.joint_state.name = [
            'joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0',  # 食指
            'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0',  # 中指
            'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0',  # 无名指
            'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0'  # 拇指
        ]
        self.joint_state.position = [0.0] * 16

        # 定义关节限制
        self.joint_limits = {
            "joint_0.0": (-0.47, 0.47),
            "joint_1.0": (-0.196, 1.61),
            "joint_2.0": (-0.174, 1.709),
            "joint_3.0": (-0.227, 1.618),
            "joint_4.0": (-0.47, 0.47),
            "joint_5.0": (-0.196, 1.61),
            "joint_6.0": (-0.174, 1.709),
            "joint_7.0": (-0.227, 1.618),
            "joint_8.0": (-0.47, 0.47),
            "joint_9.0": (-0.196, 1.61),
            "joint_10.0": (-0.174, 1.709),
            "joint_11.0": (-0.227, 1.618),
            "joint_12.0": (0.263, 1.396),
            "joint_13.0": (-0.105, 1.163),
            "joint_14.0": (-0.189, 1.644),
            "joint_15.0": (-0.162, 1.719)
        }

    def check_joint_limits(self, positions):
        """检查关节角度是否在限制范围内"""
        for joint_name, position in zip(self.joint_state.name, positions):
            min_pos, max_pos = self.joint_limits[joint_name]
            if position < min_pos or position > max_pos:
                return False
        return True

    def action_callback(self, msg):
        # 如果正在执行动作，直接返回
        with self.execution_lock:
            if self.executing:
                rospy.logwarn("上一个动作还在执行中，忽略新的动作")
                return
            self.executing = True
        
        try:
            # 执行Franka的运动
            target_pose = Pose()
            target_pose.position.x = msg.position[0]
            target_pose.position.y = msg.position[1]
            target_pose.position.z = msg.position[2]
            target_pose.orientation.x = msg.position[3]
            target_pose.orientation.y = msg.position[4]
            target_pose.orientation.z = msg.position[5]
            target_pose.orientation.w = msg.position[6]

            plan_success = self.arm_group.go(wait=True)
            if not plan_success:
                rospy.logwarn("Franka运动规划失败")
                return

            # 执行Allegro手的运动
            allegro_positions = msg.position[7:23]
            if not self.check_joint_limits(allegro_positions):
                rospy.logwarn("Allegro手关节角度超出限制范围")
                return

            self.joint_state.header.stamp = rospy.Time.now()
            self.joint_state.position = allegro_positions
            self.hand_pub.publish(self.joint_state)

            # 等待一小段时间确保动作执行完成
            rospy.sleep(0.1)

            # 发布执行完成信号
            self.execution_done_pub.publish(msg)
            
        finally:
            # 无论执行成功与否，都要释放执行状态
            with self.execution_lock:
                self.executing = False

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass