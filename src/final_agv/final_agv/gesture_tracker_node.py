#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import time
import os
from ament_index_python.packages import get_package_share_directory

# --- MediaPipe Task API Aliases ---
package_share_directory = get_package_share_directory('final_agv')
model_path = os.path.join(package_share_directory, 'models', 'hand_landmarker.task') 

BaseOptions = mp.tasks.BaseOptions(model_asset_path=model_path)
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode 


class GestureTrackerNode(Node):
    def __init__(self):
        super().__init__('gesture_tracker_node')
        
        self.publisher_ = self.create_publisher(Twist, '/gesture_cmd', 10)
        
        self.latest_result = None
        
        options = HandLandmarkerOptions(
            base_options=BaseOptions,
            running_mode=VisionRunningMode.LIVE_STREAM,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            result_callback=self.result_callback
        )
        self.landmarker = HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("ไม่สามารถเปิดกล้องได้ โปรดตรวจสอบการเชื่อมต่อ")
            raise SystemExit
        
        self.MAX_LINEAR = 0.5  # m/s
        self.MAX_ANGULAR = 1.0 # rad/s
        self.DEADZONE = 50     # รัศมี pixel ที่ให้หุ่นยนต์หยุดนิ่ง

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Gesture Tracker Node เริ่มทำงานแล้ว (กด 'q' ที่หน้าต่างภาพเพื่อออก)")

    def result_callback(self, result, output_image, timestamp_ms):
        self.latest_result = result

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warning("ไม่สามารถอ่านภาพจากกล้องได้")
            return

        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        center_x, center_y = w // 2, h // 2

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        timestamp = int(time.time() * 1000)
        self.landmarker.detect_async(mp_image, timestamp)

        msg = Twist()

        cv2.circle(frame, (center_x, center_y), self.DEADZONE, (255, 0, 0), 2)
        cv2.line(frame, (center_x, 0), (center_x, h), (200, 200, 200), 1)
        cv2.line(frame, (0, center_y), (w, center_y), (200, 200, 200), 1)

        if self.latest_result and self.latest_result.hand_landmarks:
            for i, hand_landmarks in enumerate(self.latest_result.hand_landmarks):
                label = self.latest_result.handedness[i][0].category_name
                
                lm = hand_landmarks[9]
                hx = int(lm.x * w)
                hy = int(lm.y * h)

                
                for landmark in hand_landmarks:
                    cx, cy = int(landmark.x * w), int(landmark.y * h)
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

                if label == 'Right':
                    dx = hx - center_x
                    dy = hy - center_y
                    
                    if abs(dx) > self.DEADZONE:
                        msg.linear.y = -(dx / (w / 2.0)) * self.MAX_LINEAR
                    if abs(dy) > self.DEADZONE:
                        msg.linear.x = -(dy / (h / 2.0)) * self.MAX_LINEAR

                elif label == 'Left':
                    dx = hx - center_x
                    if abs(dx) > self.DEADZONE:
                        msg.angular.z = -(dx / (w / 2.0)) * self.MAX_ANGULAR

        ui_text = f"Lin X: {msg.linear.x:.2f} | Lin Y: {msg.linear.y:.2f} | Ang Z: {msg.angular.z:.2f}"
        cv2.putText(frame, ui_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        self.publisher_.publish(msg)

        cv2.imshow("Gesture Control (Tasks API)", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("ได้รับคำสั่งปิดหน้าต่าง (กด 'q')...")
            raise SystemExit

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        try:
            self.landmarker.close()
        except Exception as e:
            self.get_logger().warning(f"ปิด Landmarker ไม่สำเร็จ: {e}")
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureTrackerNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('กำลังหยุดการทำงานของ Gesture Tracker Node...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
