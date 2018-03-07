import wpilib
import constants
import swerve
import lift
import winch
from teleop import Teleop
from autonomous.fsm_auto import Autonomous
from sensors.imu import IMU


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        constants.load_control_config()

        wpilib.CameraServer.launch('driver_vision.py:main')

        self.autoPositionSelect = wpilib.SendableChooser()
        self.autoPositionSelect.addDefault('Middle', 'Middle')
        self.autoPositionSelect.addObject('Left', 'Left')
        self.autoPositionSelect.addObject('Right', 'Right')

        wpilib.SmartDashboard.putData(
            'Robot Starting Position',
            self.autoPositionSelect)

        self.drivetrain = swerve.SwerveDrive(
            constants.chassis_length,
            constants.chassis_width,
            constants.swerve_config
        )
        self.drivetrain.load_config_values()

        self.lift = lift.ManualControlLift(
            constants.lift_ids['left'],
            constants.lift_ids['right'],
            constants.lift_limit_channel,
            constants.start_limit_channel
        )

        self.winch = winch.Winch(
            constants.winch_id
        )

        self.throttle = wpilib.Joystick(1)

        self.claw = lift.Claw(
            constants.claw_id
        )

        self.imu = IMU(wpilib.SPI.Port.kMXP)

        self.sd_update_timer = wpilib.Timer()
        self.sd_update_timer.reset()
        self.sd_update_timer.start()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.lift.load_config_values()
        self.drivetrain.load_config_values()

        self.drivetrain.update_smart_dashboard()
        self.imu.update_smart_dashboard()
        self.lift.update_smart_dashboard()
        self.winch.update_smart_dashboard()

        wpilib.SmartDashboard.putNumber(
            "Throttle Pos", self.throttle.getRawAxis(constants.liftAxis)
        )

        self.lift.checkLimitSwitch()

    def autonomousInit(self):
        self.drivetrain.load_config_values()
        self.lift.load_config_values()

        self.auto = Autonomous(self, self.autoPositionSelect.getSelected())
        self.auto.periodic()
        self.lift.checkLimitSwitch()

    def autonomousPeriodic(self):
        if self.sd_timer.hasPeriodPassed(0.5):
            self.auto.update_smart_dashboard()
            self.imu.update_smart_dashboard()
            self.drivetrain.update_smart_dashboard()
            self.lift.update_smart_dashboard()
            self.winch.update_smart_dashboard()

        self.auto.periodic()
        self.lift.checkLimitSwitch()

    def teleopInit(self):
        self.teleop = Teleop(self)

        self.drivetrain.load_config_values()
        self.lift.load_config_values()
        constants.load_control_config()

        self.lift.checkLimitSwitch()

    def teleopPeriodic(self):
        self.teleop.drive()
        self.teleop.buttons()
        self.teleop.lift_control()
        self.teleop.claw_control()
        self.teleop.winch_control()

        self.claw.update()
        self.lift.checkLimitSwitch()

        if self.sd_timer.hasPeriodPassed(0.5):
            constants.load_control_config()
            self.drivetrain.load_config_values()
            self.lift.load_config_values()

            self.drivetrain.update_smart_dashboard()
            self.teleop.update_smart_dashboard()
            self.imu.update_smart_dashboard()
            self.lift.update_smart_dashboard()
            self.winch.update_smart_dashboard()


if __name__ == "__main__":
    wpilib.run(Robot)
