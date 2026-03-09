import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox
import threading

# Import Custom Service ที่เราสร้างไว้
from final_agv_interfaces.srv import ToggleControl

class GuiClientNode(Node):
    def __init__(self):
        super().__init__('gui_control_node')
        # สร้าง Service Client
        self.cli = self.create_client(ToggleControl, '/toggle_human_control')
        
        # รอจนกว่า Service Server จะพร้อมทำงาน
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('กำลังรอ Service /toggle_human_control...')

    def send_toggle_request(self, enable_status):
        req = ToggleControl.Request()
        req.enable_human = enable_status
        # ส่ง Request ไปหา Server แบบ Async
        future = self.cli.call_async(req)
        return future

class AppGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        
        # ตั้งค่าหน้าต่าง Tkinter
        self.root = tk.Tk()
        self.root.title("ระบบควบคุม myAGV")
        self.root.geometry("350x200")
        
        # ข้อความแสดงสถานะ
        self.label = tk.Label(self.root, text="เลือกโหมดการควบคุม", font=("Arial", 14))
        self.label.pack(pady=20)

        # ปุ่มเปิดระบบ (สีเขียว)
        self.btn_enable = tk.Button(self.root, text="✅ เปิดการควบคุมจากมือ", 
                                    font=("Arial", 12), bg="lightgreen",
                                    command=lambda: self.toggle_system(True))
        self.btn_enable.pack(pady=5, fill=tk.X, padx=50)

        # ปุ่มปิดระบบ (สีแดง)
        self.btn_disable = tk.Button(self.root, text="❌ ตัดการควบคุม (หยุดหุ่น)", 
                                     font=("Arial", 12), bg="salmon",
                                     command=lambda: self.toggle_system(False))
        self.btn_disable.pack(pady=5, fill=tk.X, padx=50)

    def toggle_system(self, status):
        # เรียกใช้ฟังก์ชันส่ง Service ของ ROS Node
        future = self.ros_node.send_toggle_request(status)
        
        # ผูก Callback เมื่อ Server ตอบกลับมา
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            # อัปเดตข้อความบน GUI
            self.label.config(text=response.status_message)
            self.ros_node.get_logger().info(f"Server ตอบกลับ: {response.status_message}")
        except Exception as e:
            self.ros_node.get_logger().error(f"เรียก Service ไม่สำเร็จ: {e}")

    def run(self):
        self.root.mainloop()

def ros_spin_thread(node):
    # ฟังก์ชันสำหรับรัน ROS 2 ให้อยู่ใน Thread แยก
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiClientNode()

    # สร้าง Thread เพื่อรัน ROS 2 (ไม่ให้บล็อก GUI)
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()

    # รันหน้าต่าง GUI ใน Main Thread
    gui_app = AppGUI(ros_node)
    gui_app.run()

    # เมื่อปิดหน้าต่าง GUI ให้ปิด ROS Node ด้วย
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()