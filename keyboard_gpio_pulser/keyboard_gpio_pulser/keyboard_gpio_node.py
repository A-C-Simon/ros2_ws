# import rclpy
# from rclpy.node import Node
# import Jetson.GPIO as GPIO
# import threading
# import sys
# import tty
# import termios
# import select
# from geometry_msgs.msg import Twist 

# class KeyboardGPIOPulser(Node):
#     def __init__(self):
#         super().__init__('keyboard_gpio_pulser')

#         # Define GPIO pins (BOARD numbering - use physical pin numbers)
#         self.gpio_map = {
#             'up': [7,15],      # Move Forward = left and right wheels forward
#             'down': [35,12],    # Move Backward = left and right wheels backward
#             'left': [35,7],    # Turn left = left wheel backward, right wheel forward
#             'right': [15,12],   # Turn right = left wheel forward, right wheel backward
#         }

#         self.valid_pins = [7, 12, 35, 15]
#         self.held_keys = set()
#         self.last_cmd_vel = None #stores last command vel for auto

#         # Setup GPIO
#         GPIO.setmode(GPIO.BOARD)  # Changed from BCM to BOARD
#         GPIO.setwarnings(False)   # Disable warning about carrier board
        
#         # Flatten all pin lists and get unique pins
#         for pins in set(p for plist in self.gpio_map.values() for p in plist):
#             if pins in self.valid_pins:
#                 GPIO.setup(pins, GPIO.OUT)
#                 GPIO.output(pins, GPIO.LOW)

#             else:
#                 self.get_logger().error(f"Invalid pin: {pins}")
#         self.get_logger().info('GPIO initialize, manual overrides nav2')

#         # Publisher for simulation control
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         #Subscriber Nav2
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

#         # Start listener thread
#         self.listener_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
#         self.listener_thread.start()
#         self.get_logger().warn("Press 'q' to quit the program!")

#     def set_motor_state(self, direction):
#         #setting motor pins
#         # for p in self.valid_pins:
#         #     GPIO.output(p, GPIO.LOW) #stops motors first
#         if direction in self.gpio_map:
#             for pin in self.gpio_map[direction]:
#                 GPIO.output(pin, GPIO.HIGH)

#     def cmd_vel_callback(self, msg):
#         #Handles nav2 twist messages
#         self.last_cmd_vel = msg
#         #Prioritizing manual control
#         if self.held_keys:
#             return
#         #Autonomous navigation
#         if msg.linear.x > 0:
#             self.set_motor_state('up')
#         elif msg.linear.x < 0:
#             self.set_motor_state('down')
#         elif msg.angular.z > 0:
#             self.set_motor_state('left')
#         elif msg.angular.z < 0:
#             self.set_motor_state('right')
#         else:
#             self.set_motor_state(None)

#     def get_key(self):
#         """Read a single key from terminal input"""
#         fd = sys.stdin.fileno()
#         old = termios.tcgetattr(fd)
#         try:
#             tty.setraw(fd)
#             rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#             if rlist:
#                 ch = sys.stdin.read(1)
#                 if ch == '\x1b':  # Escape sequence
#                     ch += sys.stdin.read(2)  # Read next 2 chars
#                 return ch
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old)
#         return None

#     def keyboard_listener(self):
#         key_mapping = {
#             '\x1b[A': 'up',
#             '\x1b[B': 'down',
#             '\x1b[D': 'left',
#             '\x1b[C': 'right'
#         }

#         while rclpy.ok():
#             key = self.get_key()
#             if key == 'q':
#                 print("quitting")
#                 break

#             twist = Twist()  # Always create a new Twist message

#             if key in key_mapping:
#                 direction = key_mapping[key]
#                 pins = self.gpio_map[direction]
                
#                 if direction not in self.held_keys:
#                     self.held_keys.add(direction)
#                     self.get_logger().info(f"Holding {direction.upper()} - GPIO {pins} HIGH")
#                     for pin in pins:
#                         GPIO.output(pin, GPIO.HIGH)

#                 # Set Twist values based on direction
#                 if direction == 'up':
#                     twist.linear.x = 0.5
#                 elif direction == 'down':
#                     twist.linear.x = -0.5
#                 elif direction == 'left':
#                     twist.angular.z = 1.0
#                 elif direction == 'right':
#                     twist.angular.z = -1.0

#                 self.cmd_vel_pub.publish(twist)
#             else:
#                 # Release all keys when no key is pressed
#                 for direction in list(self.held_keys):
#                     pins = self.gpio_map[direction]
#                     self.get_logger().info(f"Released {direction.upper()} - GPIO {pins} LOW")
#                     for pin in pins:
#                         GPIO.output(pin, GPIO.LOW)
#                     self.held_keys.remove(direction)

#                 # Publish zero Twist to stop simulation robot
#                 self.cmd_vel_pub.publish(Twist())

#         self.destroy_node()
#         rclpy.shutdown()

#     def destroy_node(self):
#         GPIO.cleanup()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardGPIOPulser()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()





import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import threading
import sys
import tty
import termios
import select
from geometry_msgs.msg import Twist
import time

class KeyboardGPIOPWM(Node):
    def __init__(self):
        super().__init__('keyboard_gpio_pwm')

        #PARAMETERS
        self.left_forward = 7   # BOARD pin for left motor forward
        self.left_backward = 35 # BOARD pin for left motor backward
        self.right_forward = 15 # BOARD pin for right motor forward
        self.right_backward = 12# BOARD pin for right motor backward

        self.left_en = 33   # BOARD pin for left motor EN (PWM)
        self.right_en = 32  # BOARD pin for right motor EN (PWM)

        self.wheel_base = 0.3   # meters
        self.max_lin = 1.0      # m/s
        self.max_ang = 1.0      # rad/s
        self.pwm_freq = 100     # Hz
        self.manual_timeout = 0.3 # seconds

        #GPIO SETUP
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        for pin in [self.left_forward, self.left_backward, self.right_forward, self.right_backward]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        GPIO.setup(self.left_en, GPIO.OUT)
        GPIO.setup(self.right_en, GPIO.OUT)

        self.pwm_left = GPIO.PWM(self.left_en, self.pwm_freq)
        self.pwm_right = GPIO.PWM(self.right_en, self.pwm_freq)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        #STATE 
        self.held_keys = set()
        self.last_manual_time = 0.0

        # Publisher for simulation & control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Start keyboard listener
        self.listener_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.listener_thread.start()
        self.get_logger().warn("Press 'q' to quit the program!")

    #MOTOR CONTROL
    def set_motor_pwm(self, left_speed, right_speed):
        # left_speed/right_speed in [-1, 1]
        # Left motor
        if left_speed > 0:
            GPIO.output(self.left_forward, GPIO.HIGH)
            GPIO.output(self.left_backward, GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(self.left_forward, GPIO.LOW)
            GPIO.output(self.left_backward, GPIO.HIGH)
        else:
            GPIO.output(self.left_forward, GPIO.LOW)
            GPIO.output(self.left_backward, GPIO.LOW)

        # Right motor
        if right_speed > 0:
            GPIO.output(self.right_forward, GPIO.HIGH)
            GPIO.output(self.right_backward, GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(self.right_forward, GPIO.LOW)
            GPIO.output(self.right_backward, GPIO.HIGH)
        else:
            GPIO.output(self.right_forward, GPIO.LOW)
            GPIO.output(self.right_backward, GPIO.LOW)

        self.pwm_left.ChangeDutyCycle(min(abs(left_speed) * 100, 100))
        self.pwm_right.ChangeDutyCycle(min(abs(right_speed) * 100, 100))

    def twist_to_speeds(self, twist_msg):
        v = max(min(twist_msg.linear.x, self.max_lin), -self.max_lin)
        w = max(min(twist_msg.angular.z, self.max_ang), -self.max_ang)
        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)
        max_wheel_speed = self.max_lin + (self.max_ang * self.wheel_base / 2.0)
        left_norm = v_l / max_wheel_speed
        right_norm = v_r / max_wheel_speed
        return left_norm, right_norm

    # AUTONOMOUS CALLBACK
    def cmd_vel_callback(self, msg):
        # Ignore if manual override is active
        if time.time() - self.last_manual_time < self.manual_timeout:
            return
        left, right = self.twist_to_speeds(msg)
        self.set_motor_pwm(left, right)

    # KEYBOARD LISTENER 
    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    ch += sys.stdin.read(2)
                return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return None

    def keyboard_listener(self):
        key_mapping = {
            '\x1b[A': 'up',
            '\x1b[B': 'down',
            '\x1b[D': 'left',
            '\x1b[C': 'right'
        }

        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break

            twist = Twist()
            if key in key_mapping:
                direction = key_mapping[key]
                self.held_keys.add(direction)
                self.last_manual_time = time.time()

                if direction == 'up':
                    GPIO.output(self.left_forward, GPIO.HIGH)
                    GPIO.output(self.left_backward, GPIO.LOW)
                    GPIO.output(self.right_forward, GPIO.HIGH)
                    GPIO.output(self.right_backward, GPIO.LOW)
                    self.pwm_left.ChangeDutyCycle(100)
                    self.pwm_right.ChangeDutyCycle(100)
                    twist.linear.x = 0.5
                elif direction == 'down':
                    GPIO.output(self.left_forward, GPIO.LOW)
                    GPIO.output(self.left_backward, GPIO.HIGH)
                    GPIO.output(self.right_forward, GPIO.LOW)
                    GPIO.output(self.right_backward, GPIO.HIGH)
                    self.pwm_left.ChangeDutyCycle(100)
                    self.pwm_right.ChangeDutyCycle(100)
                    twist.linear.x = -0.5
                elif direction == 'left':
                    GPIO.output(self.left_forward, GPIO.LOW)
                    GPIO.output(self.left_backward, GPIO.HIGH)
                    GPIO.output(self.right_forward, GPIO.HIGH)
                    GPIO.output(self.right_backward, GPIO.LOW)
                    self.pwm_left.ChangeDutyCycle(100)
                    self.pwm_right.ChangeDutyCycle(100)
                    twist.angular.z = 1.0
                elif direction == 'right':
                    GPIO.output(self.left_forward, GPIO.HIGH)
                    GPIO.output(self.left_backward, GPIO.LOW)
                    GPIO.output(self.right_forward, GPIO.LOW)
                    GPIO.output(self.right_backward, GPIO.HIGH)
                    self.pwm_left.ChangeDutyCycle(100)
                    self.pwm_right.ChangeDutyCycle(100)
                    twist.angular.z = -1.0

                self.cmd_vel_pub.publish(twist)

            else:
                if self.held_keys:
                    self.held_keys.clear()
                    GPIO.output(self.left_forward, GPIO.LOW)
                    GPIO.output(self.left_backward, GPIO.LOW)
                    GPIO.output(self.right_forward, GPIO.LOW)
                    GPIO.output(self.right_backward, GPIO.LOW)
                    self.pwm_left.ChangeDutyCycle(0)
                    self.pwm_right.ChangeDutyCycle(0)
                    self.cmd_vel_pub.publish(Twist())

        self.destroy_node()
        rclpy.shutdown()

    def destroy_node(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardGPIOPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()