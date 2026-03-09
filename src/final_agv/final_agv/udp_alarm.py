#!/usr/bin/env python3
import socket

def main():
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"📡 กำลังรอรับสัญญาณแจ้งเตือนผ่าน UDP พอร์ต {UDP_PORT}...")

    try:
        while True:
            data, addr = sock.recvfrom(1024) 
            message = data.decode('utf-8')
            
            parts = message.split(',')
            status = parts[0]
            
            if status == "DANGER":
                print(f"\033[91m\033[1m🚨 อันตราย! วัตถุอยู่ใกล้หุ่นยนต์มาก! (ระยะ: {parts[1]} mm)\033[0m")
                print('\a', end='', flush=True) 
            elif status == "WARNING":
                print(f"\033[93m⚠️ ระวัง! พบวัตถุ (ระยะ: {parts[1]} mm)\033[0m")
    except KeyboardInterrupt:
        print("\nปิดระบบแจ้งเตือน")
    finally:
        sock.close()

if __name__ == '__main__':
    main()