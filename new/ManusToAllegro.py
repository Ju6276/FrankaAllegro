#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math

class GloveToAllegro:
    def __init__(self):
        rospy.init_node('glove_to_allegro', anonymous=True)

        # 订阅 /manus_glove_data_left 数据 (std_msgs/Float32MultiArray)
        rospy.Subscriber('/manus_glove_data_left', Float32MultiArray, self.callback)

        # 发布到 AllegroHand 控制器
        self.pub = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=10)

        # 初始化关节状态，设置关节名和初始值
        self.joint_state = JointState()
        self.joint_state.name = [
            'joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0',  # 食指
            'joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0',  # 中指
            'joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0',  # 无名指
            'joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0'  # 拇指
        ]
        self.joint_state.position = [0.0] * 16  # 设置初始值

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

        rospy.spin()

    def callback(self, msg):
        """
        将手套数据转换为弧度制，并约束在 URDF 的关节限制范围内。
        """
        glove_data = msg.data

        # 检查数据长度是否符合预期
        if len(glove_data) >= 16:
            # 重新排列手套数据顺序（manus和allegro的关节匹配）
            reordered_data = self.reorder_glove_data(glove_data)

            # 转换为弧度制
            radians_data = [math.radians(angle) for angle in reordered_data]

            # 应用关节限制
            constrained_data = [
                self.apply_joint_limits(joint_name, radians)
                for joint_name, radians in zip(self.joint_state.name, radians_data)
            ]

            # 更新关节状态
            self.joint_state.position = constrained_data
            self.joint_state.header.stamp = rospy.Time.now()

            # 发布关节命令
            self.pub.publish(self.joint_state)
        else:
            rospy.logwarn("Received glove data with insufficient length!")

    def reorder_glove_data(self, glove_data):
        """
        将手套数据按照 AllegroHand 的关节顺序重新排列：
        原始顺序为：大拇指、食指、中指、无名指
        目标顺序为：食指、中指、无名指、大拇指
        """
        # 手套数据按原始顺序划分
        thumb = glove_data[0:4]  # 大拇指
        index = glove_data[4:8]  # 食指
        middle = glove_data[8:12]  # 中指
        ring = glove_data[12:16]  # 无名指
        # 返回重新排列后的顺序
        return index + middle + ring + thumb

    def apply_joint_limits(self, joint_name, value):
        """
        根据关节名称和关节限制约束值。
        :param joint_name: 关节名称
        :param value: 需要约束的值
        :return: 约束后的值
        """
        if joint_name in self.joint_limits:
            lower, upper = self.joint_limits[joint_name]
            return max(min(value, upper), lower)
        else:
            rospy.logwarn(f"Joint {joint_name} has no limits defined!")
            return value


if __name__ == '__main__':
    try:
        GloveToAllegro()
    except rospy.ROSInterruptException:
        pass
