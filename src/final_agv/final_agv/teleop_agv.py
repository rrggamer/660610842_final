import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- settings ---
msg = """
Control Your Mecanum AGV!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : linear X (forward/back)
a/d : linear Y (strafe left/right) <-- NEW for Mecanum
q/e : angular Z (rotate left/right)
s   : force stop

CTRL-C to quit
"""

# Key mappings (x, y, z, th)
moveBindings = {
    'w': (1, 0, 0, 0),   # Forward
    'x': (-1, 0, 0, 0),  # Backward
    'a': (0, 1, 0, 0),   # Strafe Left
    'd': (0, -1, 0, 0),  # Strafe Right
    'q': (0, 0, 0, 1),   # Rotate Left
    'e': (0, 0, 0, -1),  # Rotate Right
    's': (0, 0, 0, 0),   # Stop
}

# Default speeds
LINEAR_SPEED = 0.5  # m/s
ANGULAR_SPEED = 1.0 # rad/s

def getKey(settings):
    """Reads a single keypress from stdin without requiring Enter"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.get_logger().info("Teleop Node Started. Use WASD to move (Mecanum Mode).")

    # Added 'y' argument for strafing
    def send_vel(self, x, y, th):
        twist = Twist()
        twist.linear.x = x * LINEAR_SPEED
        twist.linear.y = y * LINEAR_SPEED # <-- Mecanum strafing
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * ANGULAR_SPEED
        self.pub.publish(twist)

def main():
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopNode()

    # Current velocity state
    target_linear_x = 0.0
    target_linear_y = 0.0
    target_angular_vel = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][3]
                
                # If 's' is pressed, stop immediately
                if key == 's':
                    target_linear_x = 0.0
                    target_linear_y = 0.0
                    target_angular_vel = 0.0
                else:
                    target_linear_x = float(x)
                    target_linear_y = float(y)
                    target_angular_vel = float(th)

                node.send_vel(target_linear_x, target_linear_y, target_angular_vel)

            # Check for CTRL+C (since 'q' is now used for rotation)
            elif key == '\x03': 
                break

    except Exception as e:
        print(e)

    finally:
        # Stop the robot on exit
        node.send_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
