#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo, JointState
import os
from cv_bridge import CvBridge
import argparse
import cv2
from datetime import datetime
from IPython import embed 
import pyrealsense2 as rs
def parse_args():
    parser = argparse.ArgumentParser()
    default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data_raw')
    parser.add_argument('--root_dir', type=str, default=default_path, help='保存数据的目录')
    args = parser.parse_args()
    
    os.makedirs(args.root_dir, exist_ok=True)
    return args

class DataCollector:
    def __init__(self, args):
        rospy.init_node('data_collector')
        self.bridge = CvBridge()
        self.root_dir = args.root_dir
        
        # 参数设置
        self.fps = 10  # 10Hz
        
        # 数据存储
        self.episode_idx = None
        self.current_episode_dir = None
        
        # 状态控制
        self.is_recording = False
        self.current_frame = 0
        
        # 数据存储列表（用于临时存储一个episode的数据）
        self.state_arrays = []
        self.action_arrays = []
        self.timestamps = []
        
        # 同步订阅器和回调设置
        self._setup_subscribers()
        
        self.previous_state = None
        rospy.loginfo("数据采集器已初始化。按 's' 开始采集，'e' 结束采集")
        
        # 添加键盘监听
        import threading
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

    def _setup_subscribers(self):
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.joint_sub = message_filters.Subscriber('/joint_states', JointState)
        self.camera_info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)
        self.camera_intrinsics = None
        self.camera_info_sub.registerCallback(self.camera_info_callback)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.joint_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

    def _create_episode_directory(self):
        """创建新的episode目录"""
        episode_name = f"episode_{self.episode_idx:03d}"
        episode_dir = os.path.join(self.root_dir, episode_name)
        os.makedirs(os.path.join(episode_dir, "img"), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, "depth"), exist_ok=True)
        return episode_dir

    def sync_callback(self, img_msg, depth_msg, joint_msg):
        if not self.is_recording:
            return

        try:
            # 转换图像数据
            color_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            rospy.loginfo(f"彩色图像尺寸: {color_image.shape}, 数据类型: {color_image.dtype}")
            rospy.loginfo(f"深度图像尺寸: {depth_image.shape}, 数据类型: {depth_image.dtype}")
            rospy.loginfo(f"深度图像范围: 最小值={np.min(depth_image)}, 最大值={np.max(depth_image)}")
            
            # 获取当前关节位置作为状态
            current_state = np.array(joint_msg.position, dtype=np.float32)
            
            # 将当前状态同时记录为状态和动作
            self.state_arrays.append(current_state)
            self.action_arrays.append(current_state)  # 修改为记录当前状态
            
            # 保存原始图像和深度数据
            frame_name = f"frame_{self.current_frame:04d}"
            cv2.imwrite(os.path.join(self.current_episode_dir, "img", f"{frame_name}.png"), color_image)
            np.save(os.path.join(self.current_episode_dir, "depth", f"{frame_name}.npy"), depth_image)
            
            # 收集时间戳数据
            self.timestamps.append(rospy.Time.now().to_sec())
            
            self.current_frame += 1
            
            if self.current_frame % 10 == 0:
                time_elapsed = self.current_frame / 10.0
                rospy.loginfo(f"轨迹 {self.episode_idx}: 已收集 {self.current_frame} 帧 ({time_elapsed:.1f}秒)")
            
        except Exception as e:
            rospy.logerr(f"Error in sync_callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def _start_recording(self):
        """开始记录"""
        while True:
            try:
                idx = input("请输入当前轨迹的序号（从0开始的整数）: ")
                self.episode_idx = int(idx)
                if self.episode_idx < 0:
                    print("序号必须大于等于0！")
                    continue
                break
            except ValueError:
                print("请输入有效的整数！")
        
        self.current_episode_dir = self._create_episode_directory()
        self.is_recording = True
        self.current_frame = 0
        self.state_arrays = []
        self.action_arrays = []
        self.timestamps = []
        self.previous_state = None
        rospy.loginfo(f"开始记录第 {self.episode_idx} 条轨迹...")
        rospy.loginfo("按 'e' 结束当前轨迹收集")

    def _stop_recording(self):
        """停止记录"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        if len(self.state_arrays) > 0:
            # 保存状态、动作和时间戳数据
            np.save(os.path.join(self.current_episode_dir, "state.npy"), np.array(self.state_arrays))
            np.save(os.path.join(self.current_episode_dir, "action.npy"), np.array(self.action_arrays))
            np.save(os.path.join(self.current_episode_dir, "timestamps.npy"), np.array(self.timestamps))
            
            rospy.loginfo(f"结束记录第 {self.episode_idx} 条轨迹，共 {self.current_frame} 帧")
            rospy.loginfo(f"数据已保存到: {self.current_episode_dir}")
            rospy.loginfo("按 's' 开始收集新的轨迹，按 'q' 退出程序")
        else:
            rospy.loginfo("没有收集到数据，放弃保存")

    def _keyboard_listener(self):
        """监听键盘输入"""
        while not rospy.is_shutdown():
            key = input().lower()
            if key == 's' and not self.is_recording:
                self._start_recording()
            elif key == 'e' and self.is_recording:
                self._stop_recording()
            elif key == 'q':
                if self.is_recording:
                    self._stop_recording()
                rospy.signal_shutdown("User requested shutdown")
                break

    def camera_info_callback(self, msg):
        """处理相机内参信息"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.K[0],  # 焦距x
                'fy': msg.K[4],  # 焦距y
                'cx': msg.K[2],  # 主点x
                'cy': msg.K[5]   # 主点y
            }
            rospy.loginfo("已获取相机内参")

def main():
    args = parse_args()
    collector = DataCollector(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        if collector.is_recording:
            collector._stop_recording()
        rospy.loginfo("数据收集已停止")

if __name__ == '__main__':
    main()