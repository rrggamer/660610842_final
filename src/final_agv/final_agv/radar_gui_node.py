#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import math
import time

class RadarGuiNode(Node):
    def __init__(self):
        super().__init__('radar_gui_node')
        
        # Subscriber รับข้อความเตือน
        self.subscription = self.create_subscription(String, '/obstacle_alert', self.alert_callback, 10)
        
        # ตัวแปรเก็บข้อมูลเพื่อวาดจอ
        self.status = "SAFE"
        self.distance = 0.0
        self.angle_cw = 0.0
        self.last_update_time = time.time()
        
        # ตั้งค่าหน้าจอ OpenCV
        self.img_size = 500
        self.center = (self.img_size // 2, self.img_size // 2)
        
        # Timer สำหรับรีเฟรชภาพ 10Hz
        self.timer = self.create_timer(0.1, self.draw_radar)
        
        self.get_logger().info('Radar GUI Node เริ่มทำงาน (รอรับข้อมูลเตือน...)')

    def alert_callback(self, msg):
        # แยกข้อความที่ส่งมาจากหุ่นยนต์ (เช่น "DANGER,180,45")
        try:
            parts = msg.data.split(',')
            self.status = parts[0]
            self.distance = float(parts[1])
            self.angle_cw = float(parts[2])
            self.last_update_time = time.time()
        except Exception as e:
            self.get_logger().warning(f"อ่านข้อมูลเตือนไม่สำเร็จ: {e}")

    def draw_radar(self):
        # 1. สร้างภาพพื้นหลังสีดำ
        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        
        # วาดวงกลมระยะ (เปรียบเทียบจากตัวหุ่น)
        cv2.circle(img, self.center, 100, (50, 50, 50), 1)  # ระยะใกล้
        cv2.circle(img, self.center, 200, (50, 50, 50), 1)  # ระยะไกล
        
        # วาดแกนกากบาท (หน้า-หลัง, ซ้าย-ขวา)
        cv2.line(img, (self.center[0], 0), (self.center[0], self.img_size), (100, 100, 100), 1)
        cv2.line(img, (0, self.center[1]), (self.img_size, self.center[1]), (100, 100, 100), 1)
        
        # วาดตัวหุ่นยนต์ตรงกลาง
        cv2.rectangle(img, (self.center[0]-20, self.center[1]-30), 
                      (self.center[0]+20, self.center[1]+30), (255, 255, 255), 2)
        cv2.putText(img, "FRONT", (self.center[0]-35, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 2. ถ้าเพิ่งได้รับข้อมูลมาไม่เกิน 1 วินาที ให้วาดจุดสิ่งกีดขวาง
        if time.time() - self.last_update_time < 1.0:
            color = (0, 0, 255) if self.status == "DANGER" else (0, 255, 255) # แดง หรือ เหลือง
            
            # แปลงองศาบนจอคอม (0 อยู่ด้านบน หมุนตามเข็ม) -> เรเดียน
            # ใน OpenCV 0 องศา (แกน X) คือทางขวา ดังนั้นถ้าให้ 0 อยู่ด้านบน ต้องลบ 90 องศา
            angle_rad = math.radians(self.angle_cw - 90)
            
            # สมมติรัศมีวาดภาพให้ดูง่าย (วงใน 100px, วงนอก 200px)
            draw_radius = 120 if self.status == "DANGER" else 180
            
            obj_x = int(self.center[0] + draw_radius * math.cos(angle_rad))
            obj_y = int(self.center[1] + draw_radius * math.sin(angle_rad))
            
            # วาดจุดและเส้นเชื่อม
            cv2.line(img, self.center, (obj_x, obj_y), color, 2)
            cv2.circle(img, (obj_x, obj_y), 15, color, -1)
            
            # แสดงข้อมูล Text
            cv2.putText(img, f"STATUS: {self.status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(img, f"DIST: {self.distance:.0f} mm", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(img, f"ANG: {self.angle_cw:.0f} deg", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        else:
            # ถ้าไม่มีข้อมูลเกิน 1 วิ แปลว่าปลอดภัย
            cv2.putText(img, "STATUS: CLEAR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 3. แสดงหน้าต่างภาพ
        cv2.imshow("Robot Safety Radar", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("ปิดหน้าต่าง Radar...")
            raise SystemExit

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RadarGuiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()