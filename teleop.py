"""
Contains functions for teleop logic.
"""
import wpilib
import numpy as np
import constants
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from ctre.talonsrx import TalonSRX

class Teleop:
    last_applied_control = np.array([0, 0, 0])
    foc_enabled = False

    def __init__(self, robot):
        self.robot = robot
        self.stick = wpilib.Joystick(0)
        self.throttle = wpilib.Joystick(1)

        self.prefs = wpilib.Preferences.getInstance()

        self.toggle_foc_button = ButtonDebouncer(self.stick, 7)
        self.zero_yaw_button = ButtonDebouncer(self.stick, 3)
        self.switch_camera_button = ButtonDebouncer(self.stick, 4)
        self.low_speed_button = ButtonDebouncer(self.stick, 9)
        self.high_speed_button = ButtonDebouncer(self.stick, 10)

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putBoolean(
            'FOC Enabled', self.foc_enabled
        )

    def buttons(self):
        if self.robot.imu.is_present():
            if self.zero_yaw_button.get():
                self.robot.imu.reset()

            if self.toggle_foc_button.get():
                self.foc_enabled = not self.foc_enabled

        if self.switch_camera_button.get():
            current_camera = (self.prefs.getInt('Selected Camera', 0) + 1) % 1
            self.prefs.putInt('Selected Camera', current_camera)

    def lift_control(self):
        liftPct = self.throttle.getRawAxis(constants.liftAxis)

        if constants.liftInv:
            liftPct *= -1

        if abs(liftPct) < constants.lift_deadband:
            liftPct = 0

        liftPct *= constants.lift_coeff

        wpilib.SmartDashboard.putNumber("Lift Power", liftPct)

        self.robot.lift.setLiftPower(liftPct)

    def claw_control(self):
        clawPct = self.throttle.getRawAxis(5)
        clawPct *= 0.35

        self.robot.claw.talon.set(
            TalonSRX.ControlMode.PercentOutput, clawPct
        )

    def winch_control(self, val):
        if val == 1:
            self.robot.winch.forward()
        elif val == 0:
            self.robot.winch.stop()
        elif val == -1:
            self.robot.winch.reverse()
        else:
            pass

    def drive(self):
        """
        Drive the robot directly using a joystick.
        """

        ctrl = np.array([
            self.stick.getRawAxis(1),
            self.stick.getRawAxis(0)
        ])

        if constants.fwdInv:
            ctrl[0] *= -1

        if constants.strInv:
            ctrl[1] *= -1

        linear_control_active = True
        if abs(np.sqrt(np.sum(ctrl**2))) < 0.1:
            ctrl[0] = 0
            ctrl[1] = 0
            linear_control_active = False

        if (self.robot.imu.is_present() and self.foc_enabled):
            # perform FOC coordinate transform
            hdg = self.robot.imu.get_robot_heading()

            # Right-handed passive (alias) transform matrix
            foc_transform = np.array([
                [np.cos(hdg), np.sin(hdg)],
                [-np.sin(hdg), np.cos(hdg)]
            ])

            ctrl = np.squeeze(np.matmul(foc_transform, ctrl))

        tw = self.stick.getRawAxis(2)
        if constants.rcwInv:
            tw *= -1

        rotation_control_active = True
        if abs(tw) < 0.15:
            tw = 0
            rotation_control_active = False

        tw *= constants.turn_sensitivity

        if linear_control_active or rotation_control_active:
            self.last_applied_control = np.array([
                ctrl[0],
                ctrl[1],
                tw
            ])

            speed_coefficient = 0.75
            if self.low_speed_button.get():
                speed_coefficient = 0.25
            elif self.high_speed_button.get():
                speed_coefficient = 1

            self.robot.drivetrain.drive(
                ctrl[0] * speed_coefficient,
                ctrl[1] * speed_coefficient,
                tw * speed_coefficient,
                max_wheel_speed=constants.teleop_speed
            )
        else:
            self.robot.drivetrain.drive(
                self.last_applied_control[0],
                self.last_applied_control[1],
                self.last_applied_control[2],
                max_wheel_speed=0
            )
