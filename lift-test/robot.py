"""
This program is intended to be used for lift testing and lift control tuning.

Teleop mode will control the lift using the 4th axis on the second controller;
this normally corresponds to the throttle slider.

Autonomous mode will attempt to drive the lift to a height of 3 feet.

Everything can be configured either from Preferences or from the Robot class
member variables.
"""

import math
from ctre.talonsrx import TalonSRX
import wpilib

FeedbackDevice = TalonSRX.FeedbackDevice
angle_conv_factor = math.pi / (512 * 3)


class Robot(wpilib.IterativeRobot):
    # Lift talon IDs:
    main_lift_id = 14
    follower_id = 15

    # Lift of one stage of the RD4B, in inches
    ARM_LENGTH = 28

    controller_index = 1
    control_axis_index = 4

    def __load_config(self):
        self.max_height = self.__prefs.getFloat("Lift Max Height", 96)

        # get the encoder value when the bars are horizontal and form right
        # angles.  This is used for calculations.
        self.HORIZONTAL_ANGLE = self.__prefs.getFloat("Lift Pot Horizontal Position", 0)  # noqa: E501

        # get the encoder value when the RD4B is in the fully down position.
        self.initial_angle = self.__prefs.getFloat("Lift Pot Lower Limit", 0)  # noqa: E501

        # pre-convert the initial angle to radians and take the sin of the
        # angle.
        self.INITIAL_ANGLE_RADIANS_SIN = math.sin(
            (self.initial_angle - self.HORIZONTAL_ANGLE) * angle_conv_factor
        )

        # get the upper limit of the encoder, or the encoder value when the
        # RD4B is fully extended upward.
        self.LIMIT_UP = self.__prefs.getFloat("Lift Pot Upper Limit", 0)

        # set the soft limits to LIMIT_UP and the initial angle. the motors
        # should not go outside of this range.
        self.lift_main.configForwardSoftLimitThreshold(self.LIMIT_UP, 0)
        self.lift_main.configReverseSoftLimitThreshold(self.initial_angle, 0)

        self.lift_main.setSensorPhase(
            self.__prefs.getBoolean("Lift Sensor Phase", False)
        )

    def __update_smart_dashboard(self):
        final_angle_radians = (
            (self.lift_main.getSelectedSensorPosition(0) - self.HORIZONTAL_ANGLE)  # noqa: E501
            * angle_conv_factor
        )

        # calculate the height of the RD4B using the final angle and based on
        # the given equation.
        height = 2 * self.ARM_LENGTH * (
            math.sin(final_angle_radians) - self.INITIAL_ANGLE_RADIANS_SIN
        )

        wpilib.SmartDashboard.putNumber("Lift Height", height)
        wpilib.SmartDashboard.putNumber("Lift Angle", math.degrees(final_angle_radians))  # noqa: E501
        wpilib.SmartDashboard.putNumber(
            "Lift Position", self.lift_main.getSelectedSensorPosition(0)
        )
        wpilib.SmartDashboard.putNumber(
            "Lift Error", self.lift_main.getClosedLoopError(0)
        )

    def __set_lift_height(self, tgt_height):
        final_angle_radians = math.asin(
            (tgt_height / (2 * self.ARM_LENGTH))
            + self.INITIAL_ANGLE_RADIANS_SIN
        )

        # convert this final angle from radians to native units.
        native_units = (
            (final_angle_radians / angle_conv_factor)
            + self.HORIZONTAL_ANGLE
        )

        # set the left motor to run to this position (the right motor will
        # follow it)
        self.lift_main.set(TalonSRX.ControlMode.Position, native_units)

    def robotInit(self):
        self.stick = wpilib.Joystick(self.controller_index)
        self.__prefs = wpilib.Preferences.getInstance()

        self.lift_main = TalonSRX(self.main_lift_id)
        self.lift_follower = TalonSRX(self.follower_id)

        self.lift_main.configSelectedFeedbackSensor(
            TalonSRX.FeedbackDevice.PulseWidthEncodedPosition,
            0, 0
        )
        self.lift_main.selectProfileSlot(0, 0)

        self.lift_follower.set(
            TalonSRX.ControlMode.Follower,
            self.main_lift_id
        )

        self.__load_config()

    def disabledInit(self):
        self.__load_config()

    def disabledPeriodic(self):
        self.__update_smart_dashboard()

    def autonomousInit(self):
        self.__load_config()

    def autonomousPeriodic(self):
        self.__set_lift_height(36)
        self.__update_smart_dashboard()

    def teleopInit(self):
        self.__load_config()

    def teleopPeriodic(self):
        pct_pos = self.stick.getRawAxis(self.control_axis_index)
        tgt_height = self.max_height * pct_pos

        self.__set_lift_height(tgt_height)
        self.__update_smart_dashboard()


if __name__ == '__main__':
    wpilib.run(Robot)
