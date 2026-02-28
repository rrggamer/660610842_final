#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# นำเข้า Custom Service ที่เราสร้างไว้
from final_agv_interfaces.srv import ToggleControl 

class CoreMotionServer(Node):
    def __init__(self):
        super().__init__('core_motion_server')

        # --- 1. Publishers & Subscribers ---
        # ส่งคำสั่งไปที่ล้อหุ่นยนต์โดยตรง
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # รับคำสั่งจากกล้อง (Gesture)
        self.gesture_sub = self.create_subscription(Twist, '/gesture_cmd', self.gesture_callback, 10)
        
        # รับข้อมูล Lidar (ต้องใช้ QoS แบบ BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        # --- 2. Service Server ---
        # เปิด Service ให้คนอื่นมาสั่งตัดระบบได้
        self.srv = self.create_service(ToggleControl, '/toggle_human_control', self.toggle_callback)

        # --- 3. ตัวแปรสถานะ ---
        self.human_control_enabled = True # ค่าเริ่มต้นให้อนุญาตคนคุม
        self.latest_gesture_cmd = Twist() # เก็บค่าล่าสุดที่ได้จากมือ
        
        self.REPULSE_DIST = 0.12 # 120 mm
        self.ALERT_DIST = 0.15   # 150 mm
        self.ESCAPE_SPEED = 0.15 # ความเร็วตอนถอยหนี (m/s)

        self.get_logger().info('Core Motion Server เริ่มทำงานแล้ว (รอรับ Lidar และ Gesture)')

    # --- ฟังก์ชันจัดการ Service ---
    def toggle_callback(self, request, response):
        self.human_control_enabled = request.enable_human
        
        response.success = True
        if self.human_control_enabled:
            response.status_message = "เปิดระบบ: อนุญาตให้มนุษย์ควบคุมหุ่นยนต์ได้"
        else:
            response.status_message = "ปิดระบบ: ตัดการควบคุมจากมนุษย์ทั้งหมด!"
            
        self.get_logger().info(response.status_message)
        return response

    # --- ฟังก์ชันรับข้อมูลจากมือ ---
    def gesture_callback(self, msg):
        self.latest_gesture_cmd = msg

    # --- ฟังก์ชันรับข้อมูล Lidar & ระบบตัดสินใจ (Core Logic) ---
    def scan_callback(self, msg):
        closest_dist = float('inf')
        closest_angle_rad = 0.0
        
        # 1. หาจุดที่ใกล้ที่สุด
        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or distance <= msg.range_min or math.isinf(distance):
                continue
            if distance < closest_dist:
                closest_dist = distance
                closest_angle_rad = msg.angle_min + (i * msg.angle_increment)

        # คำสั่งความเร็วสุดท้ายที่จะส่งให้หุ่นยนต์
        final_cmd = Twist()

        # 2. เงื่อนไขที่ 1: ต่ำกว่า 120 มม. (อันตรายสูงสุด! บังคับหลบหลีกทันที)
        if closest_dist < self.REPULSE_DIST:
            # คำนวณทิศทางหนี (ผลักไปฝั่งตรงข้าม 180 องศา)
            escape_angle = closest_angle_rad + math.pi
            
            # หุ่นยนต์ Mecanum สามารถสไลด์ออกด้านข้างและเดินหน้าหลังได้พร้อมกัน
            final_cmd.linear.x = self.ESCAPE_SPEED * math.cos(escape_angle)
            final_cmd.linear.y = self.ESCAPE_SPEED * math.sin(escape_angle)
            final_cmd.angular.z = 0.0 # ไม่ต้องหมุน แค่สไลด์หนี
            
            self.get_logger().warn(
                f"อันตราย! วัตถุอยู่ใกล้ {closest_dist*1000:.0f} มม. -> ดึงสิทธิ์การควบคุมเพื่อหลบหลีก!",
                throttle_duration_sec=0.5
            )

        # 3. เงื่อนไขที่ 2: ปลอดภัย (มากกว่า 120 มม.)
        else:
            # แจ้งเตือนถ้าระยะอยู่ระหว่าง 120 - 150 มม.
            if closest_dist < self.ALERT_DIST:
                # แปลงเรเดียน -> องศา (ทวนเข็ม)
                deg_ccw = math.degrees(closest_angle_rad) % 360
                # แปลงเป็นองศา (ตามเข็ม) ให้ 0 อยู่ด้านหน้า
                deg_cw = (360 - deg_ccw) % 360
                self.get_logger().info(
                    f"️แจ้งเตือน: พบสิ่งกีดขวางที่ระยะ {closest_dist*1000:.0f} มม. ทิศทาง {deg_cw:.0f}°",
                    throttle_duration_sec=0.5
                )

            # --- ตรวจสอบสถานะ Service ว่าอนุญาตให้คนคุมหรือไม่ ---
            if self.human_control_enabled:
                # ถ้าอนุญาต: นำคำสั่งจากมือมาใช้
                final_cmd = self.latest_gesture_cmd
            else:
                # ถ้าไม่อนุญาต: หุ่นยนต์หยุดนิ่ง (final_cmd เป็น 0 ทั้งหมดอยู่แล้ว)
                pass 

        # 4. ส่งคำสั่งไปยังล้อหุ่นยนต์
        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CoreMotionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('กำลังปิด Core Motion Server...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()