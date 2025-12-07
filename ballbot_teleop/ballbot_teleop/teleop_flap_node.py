import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
# CHANGED: Import TwistStamped instead of Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

msg = """
Control Your Ballbot!
---------------------------
Moving:
   w
 a s d
   x

q/z : increase/decrease max speeds by 10%
w/x : linear movement (forward/back)
a/d : angular movement (left/right)
s   : force stop

Flap Controls:
1 : Reset Flaps (0.0, 0.0)
2 : Open Flaps  (-0.5, 0.5)
3 : Close/Grip  (-1.57, 1.57)

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'x': (-1, 0),
}

# Bindings to adjust speed (Multiplier for Linear, Multiplier for Angular)
speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

# Flap mappings: key -> [left_angle, right_angle]
flapBindings = {
    '1': [0.0, 0.0],    # Reset
    '2': [-0.5, 0.5],   # Open
    '3': [-1.57, 1.57],   # Grip
}

def getKey(settings):
    """Reads a single keypress from stdin without blocking indefinitely."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

class BallbotTeleop(Node):
    def __init__(self):
        super().__init__('ballbot_teleop')
        
        # CHANGED: Publisher uses TwistStamped to match use_stamped_vel: true
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 
            '/diff_drive_base_controller/cmd_vel', 
            10
        )
        
        self.flap_pub = self.create_publisher(
            Float64MultiArray, 
            '/flap_controller/commands', 
            10
        )

        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0

    def publish_twist(self):
        # CHANGED: Create TwistStamped message
        twist_msg = TwistStamped()
        
        # Header is required for Stamped messages
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_footprint' 
        
        twist_msg.twist.linear.x = self.x * self.speed
        twist_msg.twist.angular.z = self.th * self.turn
        
        self.cmd_vel_pub.publish(twist_msg)

    def publish_flaps(self, left_pos, right_pos):
        flap_msg = Float64MultiArray()
        flap_msg.data = [float(left_pos), float(right_pos)]
        self.flap_pub.publish(flap_msg)
        sys.stdout.write(f"\rFlaps set to: Left={left_pos}, Right={right_pos}            \n")
        sys.stdout.flush()

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    node = BallbotTeleop()
    
    print(msg)
    print(vels(node.speed, node.turn))
    
    try:
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                node.x = float(moveBindings[key][0])
                node.th = float(moveBindings[key][1])
                
            elif key in speedBindings.keys():
                node.speed = node.speed * speedBindings[key][0]
                node.turn = node.turn * speedBindings[key][1]
                
                print(vels(node.speed, node.turn))
                if (node.status == 14):
                    print(msg)
                node.status = (node.status + 1) % 15
                    
            elif key == 's':
                node.x = 0.0
                node.th = 0.0
                print("\rSTOPPING                 ")

            elif key in flapBindings.keys():
                left = flapBindings[key][0]
                right = flapBindings[key][1]
                node.publish_flaps(left, right)

            elif key == '\x03': # Ctrl-C
                break
                
            node.publish_twist()

    except Exception as e:
        print(e)

    finally:
        # Stop robot on exit (using TwistStamped)
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_footprint'
        node.cmd_vel_pub.publish(stop_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()