#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
import threading
from control_msgs.msg import JointJog
from tf.transformations import quaternion_multiply, quaternion_from_euler

class ServoTracker:
    def __init__(self):
        rospy.init_node('servo_tracking_control')
        
        # 初始化互斥锁和数据
        self.target_pose_lock = threading.Lock()
        self.target_pose = None
        self.last_command_time = rospy.Time.now()
        
        # 控制参数
        self.control_rate = 100  # Hz
        self.pos_gain = 1.0      
        self.rot_gain = 0.5
        self.max_vel = 0.3       
        self.min_vel = 0.001     
        
        # 坐标转换参数
        self.y_offset = 0.5  # 0.5米的y轴偏移
        self.z_rotation = quaternion_from_euler(0, 0, np.pi/2)  # 90度旋转
        
        # 订阅目标位姿
        self.target_sub = rospy.Subscriber(
            "/natnet_ros/RigidBody001/pose",
            PoseStamped,
            self.target_callback
        )
        
        # 发布Servo命令
        self.servo_twist_pub = rospy.Publisher(
            "/servo_server/delta_twist_cmds",
            TwistStamped,
            queue_size=1
        )
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.sleep(1.0)
        
    def transform_target_pose(self, pose):
        """应用坐标转换"""
        transformed_pose = PoseStamped()
        transformed_pose.header = pose.header
        transformed_pose.header.frame_id = "world"  # 确保在world坐标系下
        
        # 应用y轴偏移
        transformed_pose.pose.position.x = pose.pose.position.x
        transformed_pose.pose.position.y = pose.pose.position.y - self.y_offset
        transformed_pose.pose.position.z = pose.pose.position.z
        
        # 应用旋转
        original_quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        
        # 合并旋转
        rotated_quat = quaternion_multiply(self.z_rotation, original_quat)
        
        transformed_pose.pose.orientation.x = rotated_quat[0]
        transformed_pose.pose.orientation.y = rotated_quat[1]
        transformed_pose.pose.orientation.z = rotated_quat[2]
        transformed_pose.pose.orientation.w = rotated_quat[3]
        
        return transformed_pose
        
    def target_callback(self, msg):
        with self.target_pose_lock:
            # 应用坐标转换
            self.target_pose = self.transform_target_pose(msg)
            self.last_command_time = rospy.Time.now()
            rospy.loginfo_throttle(1.0, 
                f"转换后目标位置: x={self.target_pose.pose.position.x:.3f}, "
                f"y={self.target_pose.pose.position.y:.3f}, "
                f"z={self.target_pose.pose.position.z:.3f}")
            
    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world",
                "franka_flange",  # 使用franka_flange作为末端执行器
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"获取当前位姿失败: {e}")
            return None
            
    def compute_servo_command(self, current_pose, target_pose):
        # 计算位置误差
        pos_error = np.array([
            target_pose.pose.position.x - current_pose.transform.translation.x,
            target_pose.pose.position.y - current_pose.transform.translation.y,
            target_pose.pose.position.z - current_pose.transform.translation.z
        ])
        
        # 计算误差大小
        error_magnitude = np.linalg.norm(pos_error)
        rospy.loginfo_throttle(1.0, f"位置误差: {error_magnitude:.3f}")
        
        if error_magnitude < self.min_vel:
            return None
        
        # 创建twist命令
        cmd = TwistStamped()
        cmd.header.frame_id = "franka_flange"  # 使用franka_flange作为命令帧
        cmd.header.stamp = rospy.Time.now()
        
        # 应用增益和速度限制
        linear_vel = self.pos_gain * pos_error
        vel_magnitude = np.linalg.norm(linear_vel)
        
        if vel_magnitude > self.max_vel:
            linear_vel = linear_vel * self.max_vel / vel_magnitude
            
        cmd.twist.linear.x = linear_vel[0]
        cmd.twist.linear.y = linear_vel[1]
        cmd.twist.linear.z = linear_vel[2]
        
        return cmd
        
    def run(self):
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            with self.target_pose_lock:
                target_pose = self.target_pose
                command_age = (rospy.Time.now() - self.last_command_time).to_sec()
            
            if target_pose is None or command_age > 0.5:
                rate.sleep()
                continue
                
            current_pose = self.get_current_pose()
            if current_pose is None:
                rate.sleep()
                continue
                
            try:
                servo_cmd = self.compute_servo_command(current_pose, target_pose)
                if servo_cmd is not None:
                    self.servo_twist_pub.publish(servo_cmd)
                    rospy.loginfo_throttle(1.0, 
                        f"速度命令: [{servo_cmd.twist.linear.x:.3f}, "
                        f"{servo_cmd.twist.linear.y:.3f}, "
                        f"{servo_cmd.twist.linear.z:.3f}]")
                
            except Exception as e:
                rospy.logerr(f"控制错误: {e}")
                
            rate.sleep()

if __name__ == '__main__':
    try:
        tracker = ServoTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass