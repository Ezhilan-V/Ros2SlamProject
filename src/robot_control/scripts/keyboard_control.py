#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

msg = """
Robot Control with Keyboard
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity (0.5 m/s)
a/d : increase/decrease angular velocity (0.5 rad/s)
s : force stop

CTRL-C to quit
"""

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_speed = 2.0
        
        # Create a timer that publishes the current velocity
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        print(msg)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    self.linear_speed = min(self.linear_speed + 0.5, self.max_speed)
                    print(f'Linear speed: {self.linear_speed}')
                elif key == 'x':
                    self.linear_speed = max(self.linear_speed - 0.5, -self.max_speed)
                    print(f'Linear speed: {self.linear_speed}')
                elif key == 'a':
                    self.angular_speed = min(self.angular_speed + 0.5, self.max_speed)
                    print(f'Angular speed: {self.angular_speed}')
                elif key == 'd':
                    self.angular_speed = max(self.angular_speed - 0.5, -self.max_speed)
                    print(f'Angular speed: {self.angular_speed}')
                elif key == 's':
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    print('Stopped')
                elif key == '\x03':  # CTRL-C
                    break
                
        except Exception as e:
            print(e)
        
        finally:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_control = KeyboardControl()
    
    # Create a thread for the ROS 2 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(keyboard_control,))
    spin_thread.start()
    
    # Run the keyboard control
    keyboard_control.run()
    
    # Cleanup
    keyboard_control.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()