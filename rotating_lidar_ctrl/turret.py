import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from rclpy.exceptions import InvalidParameterValueException
from rclpy.signals import SignalHandlerOptions
import numpy as np
from sensor_msgs.msg import JointState


class Controller(Node):
    def __init__(self):
        super().__init__("turret_control")
        self.publisher_ = self.create_publisher(JointState, "angle", 10)

        self.declare_parameter("pub_period_s", 0.01)
        self.declare_parameter(
            "ctrl_period_s", 0.002
        )  # Inv proportional to the speed (min=0.001)
        self.declare_parameter("pins", [17, 18, 27, 22])
        self.declare_parameter("lower_angle_deg", -90.0)
        self.declare_parameter("upper_angle_deg", 90.0)
        self.declare_parameter(
            "step_count_per_rot", 4096
        )  # 5.625*(1/64) per step, 4096 steps is 360°

        pub_period_s = (
            self.get_parameter("pub_period_s").get_parameter_value().double_value
        )
        ctrl_period_s = (
            self.get_parameter("ctrl_period_s").get_parameter_value().double_value
        )
        self.pins_ = (
            self.get_parameter("pins").get_parameter_value().integer_array_value
        )
        self.step_per_rot_ = (
            self.get_parameter("step_count_per_rot").get_parameter_value().integer_value
        )
        lb_deg = (
            self.get_parameter("lower_angle_deg").get_parameter_value().double_value
        )
        ub_deg = (
            self.get_parameter("upper_angle_deg").get_parameter_value().double_value
        )
        self.omega_ = 2 * np.pi / (ctrl_period_s * self.step_per_rot_)
        self.ub_ = int(ub_deg * self.step_per_rot_ / 360.0)  # converted into steps
        self.lb_ = int(lb_deg * self.step_per_rot_ / 360.0)  # converted into steps
        self.step_ = 0
        self.cmd_ = 0
        self.dir_ = "+"
        self.step_seq_ = [
            [1, 0, 0, 1],
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ]

        self.timer_pub_ = self.create_timer(pub_period_s, self.feedbackCallback)
        self.timer_ctrl_ = self.create_timer(ctrl_period_s, self.controlCallback)
        self.printParameters()

    def setup(self):
        # setting up
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pins_[0], GPIO.OUT)
        GPIO.setup(self.pins_[1], GPIO.OUT)
        GPIO.setup(self.pins_[2], GPIO.OUT)
        GPIO.setup(self.pins_[3], GPIO.OUT)
        # initializing
        self.clean()

    def clean(self):
        GPIO.output(self.pins_[0], GPIO.LOW)
        GPIO.output(self.pins_[1], GPIO.LOW)
        GPIO.output(self.pins_[2], GPIO.LOW)
        GPIO.output(self.pins_[3], GPIO.LOW)

    def shutdown(self):
        self.clean()
        GPIO.cleanup()

    def feedbackCallback(self):
        msg = JointState()
        msg.name = ["turret"]
        msg.position = [self.getAngle()]
        msg.velocity = [self.getOmega()]
        msg.effort = [0.0]
        self.publisher_.publish(msg)

    def getAngle(self):
        conv = 2.0 * np.pi / self.step_per_rot_
        angle_deg = self.step_ * conv
        return angle_deg

    def getOmega(self):
        if self.dir_ == "-":
            return -self.omega_
        elif self.dir_ == "+":
            return self.omega_
        else:
            return 0.0

    def controlCallback(self):
        # change direction when the stepper reaches a limit
        if self.step_ >= self.ub_ and self.dir_ == "+":
            self.dir_ = "-"
        elif self.step_ <= self.lb_ and self.dir_ == "-":
            self.dir_ = "+"

        self.execute_rotation(self.dir_)

    def positiveRotation(self):
        self.step_ = self.step_ + 1

    def negativeRotation(self):
        self.step_ = self.step_ - 1

    def computeNextCommand(self):
        self.cmd_ = self.step_ % len(self.step_seq_)

    def execute_rotation(self, direction):
        # Dictionary mapping directions to behaviors
        behavior_map = {"+": self.positiveRotation, "-": self.negativeRotation}

        # Get the behavior function based on direction, default to a lambda if not found
        behavior = behavior_map.get(direction, lambda: print("Invalid direction"))
        self.computeNextCommand()

        for i in range(0, len(self.pins_)):
            GPIO.output(self.pins_[i], self.step_seq_[self.cmd_][i])
        behavior()

    def printParameters(self):
        pub_period_s = self.get_parameter("pub_period_s").get_parameter_value().double_value
        ctrl_period_s = self.get_parameter("ctrl_period_s").get_parameter_value().double_value
        pins = self.get_parameter("pins").get_parameter_value().integer_array_value
        step_per_rot = self.get_parameter("step_count_per_rot").get_parameter_value().integer_value
        lb_deg = self.get_parameter("lower_angle_deg").get_parameter_value().double_value
        ub_deg = self.get_parameter("upper_angle_deg").get_parameter_value().double_value

        self.get_logger().info(
            f"Parameters:\n"
            f"  pub_period_s: {pub_period_s}\n"
            f"  ctrl_period_s: {ctrl_period_s}\n"
            f"  pins: {list(pins)}\n"
            f"  step_count_per_rot: {step_per_rot}\n"
            f"  lower_angle_deg: {lb_deg}\n"
            f"  upper_angle_deg: {ub_deg}"
        )
        
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = Controller()
    node.setup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except InvalidParameterValueException as e:
        node.get_logger().error("Caught InvalidParameterValueException: %s" % e)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print("Turning off stepper.")
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
