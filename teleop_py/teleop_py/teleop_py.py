#!/usr/bin/env python3
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings):
    """Đọc một phím từ bàn phím mà không cần nhấn Enter"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"


class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel1', 10)

        self.speed = 0.5
        self.turn = 1.0
        self.x = self.y = self.z = self.th = 0.0
        self.status = 0

    def run(self):
        settings = termios.tcgetattr(sys.stdin)

        try:
            print(msg)
            print(vels(self.speed, self.turn))

            while True:
                key = get_key(settings)

                if key in moveBindings:
                    self.x, self.y, self.z, self.th = moveBindings[key]
                elif key in speedBindings:
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]

                    print(vels(self.speed, self.turn))
                    if self.status == 14:
                        print(msg)
                    self.status = (self.status + 1) % 15
                else:
                    self.x = self.y = self.z = self.th = 0.0
                    if key == '\x03':  # Ctrl+C
                        break

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.z = self.th * self.turn
                self.pub.publish(twist)

        except Exception as e:
            print(f"Error: {e}")

        finally:
            twist = Twist()  # Dừng robot khi thoát
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nTeleop terminated.")


def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopTwistKeyboard()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

