import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Num
from numpy import interp
import pygame

class PS4_Button:
    NO_BUTTON = 0
    SQUARE = 1
    CROSS = 2
    CIRCLE = 3
    TRIANGLE = 4
    L1 = 5
    R1 = 6
    L2 = 7
    R2 = 8
    LEFT_STICK_IN = 9
    RIGHT_STICK_IN = 10
    UP = 11
    DOWN = 12
    LEFT = 13
    RIGHT = 14

class PS4ControllerPublisher(Node):

    def __init__(self):
        super().__init__('joystick_node')
        pygame.init()
        # Create publisher and subscribe up to 10 iterations
        self.publisher_ = self.create_publisher(Num, 'joystick', 10)
        # Create timer of every 0.1s
        self.timer_ = self.create_timer(0.1, self.publish_controller_data)
        self.joysticks = {}
        self.done = False
        self.joystick_connected = False
        self.joystick_connected_id = None
        self.linear_speed_limit = 20 # in cm/s
        self.rpm_limit = 200

    def publish_controller_data(self):
        msg = Num()
        msg.button = 0
        msg.lx = 0
        msg.ly = 0
        msg.rx = 0
        msg.ry = 0

        if not self.done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.done = True  # Flag that we are done so we exit this loop.

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    self.joysticks[joy.get_instance_id()] = joy
                    self.get_logger().info(f"Joystick {joy.get_instance_id()} connected")
                    self.joystick_connected = True
                    self.joystick_connected_id = joy.get_instance_id()

                if event.type == pygame.JOYDEVICEREMOVED:
                    self.joystick_connected_id = None
                    self.joystick_connected = False
                    del self.joysticks[event.instance_id]
                    self.get_logger().info(f"Joystick {event.instance_id} disconnected")

            if self.joystick_connected:
                buttons = self.joysticks[self.joystick_connected_id].get_numbuttons()
                for i in range(buttons):
                    button = self.joysticks[self.joystick_connected_id].get_button(i)
                    if button:
                        match i:
                            case 0:
                                # print("Square")
                                msg.button = PS4_Button.SQUARE
                            case 1:
                                # print("Cross")
                                msg.button = PS4_Button.CROSS
                            case 2:
                                # print("Circle")
                                msg.button = PS4_Button.CIRCLE
                            case 3:
                                # print("Triangle")
                                msg.button = PS4_Button.TRIANGLE
                            case 4:
                                # print("L1")
                                msg.button = PS4_Button.L1
                            case 5:
                                # print("R1")
                                msg.button = PS4_Button.R1
                            case 6:
                                # print("L2")
                                msg.button = PS4_Button.L2
                            case 7:
                                # print("R2")
                                msg.button = PS4_Button.R2
                            case 8:
                                # print("Share")
                                pass
                            case 9:
                                # print("Options")
                                pass
                            case 10:
                                # print("Left stick in")
                                msg.button = PS4_Button.LEFT_STICK_IN
                            case 11:
                                # print("Right stick in")
                                msg.button = PS4_Button.RIGHT_STICK_IN
                            case 12:
                                # print("12")
                                pass
                            case 13:
                                # print("13")
                                pass
                            case 14:
                                # print("14")
                                pass
                            case 15:
                                # print("15")
                                pass

                hats = self.joysticks[self.joystick_connected_id].get_numhats()
                for i in range(hats):
                    hat = self.joysticks[self.joystick_connected_id].get_hat(i)
                    if hat[0] == 1:
                        msg.button = PS4_Button.RIGHT
                    elif hat[0] == -1:
                        msg.button = PS4_Button.LEFT
                    elif hat[1] == 1:
                        msg.button = PS4_Button.UP
                    elif hat[1] == -1:
                        msg.button = PS4_Button.DOWN

                axes = self.joysticks[self.joystick_connected_id].get_numaxes()

                for i in range(axes):
                    axis = self.joysticks[self.joystick_connected_id].get_axis(i)

                    match i:
                        # for LX and RX, from left to right, axis ranges from -1 to 1
                        # for LY and RY, from top to bottom, axis ranges from -1 to 1
                        case 0:
                            axis = self.limit_axis(axis)
                            axis = interp(axis, [-1, 1], [self.linear_speed_limit, -self.linear_speed_limit])
                            msg.lx = round(axis)
                        case 1:
                            axis = self.limit_axis(axis)
                            axis = interp(axis, [-1, 1], [self.linear_speed_limit, -self.linear_speed_limit])
                            msg.ly = round(axis)
                        case 2:
                            axis = self.limit_axis(axis)
                            axis = interp(axis, [-1, 1], [-self.rpm_limit, self.rpm_limit])
                            msg.rx = round(axis)
                        case 3:
                            pass

                        case 4:
                            pass
                        case 5:
                            axis = self.limit_axis(axis)
                            axis = interp(axis, [-1, 1], [-self.rpm_limit, self.rpm_limit])
                            msg.ry = round(axis)

            self.publisher_.publish(msg)
            self.get_logger().info(f"Button: {msg.button}, lx: {msg.lx}, ly: {msg.ly}, rx: {msg.rx}, ry: {msg.ry}")

    def limit_axis(self, axis):
        if abs(axis) < 0.05:
            return 0
        else:
            return axis

    def __del__(self):
        self.done = True
        self.destroy_node()
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    controller_publisher = PS4ControllerPublisher()
    
    try:
        rclpy.spin(controller_publisher)
    except KeyboardInterrupt:
        pass
    
    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()