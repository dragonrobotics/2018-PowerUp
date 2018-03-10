import wpilib
import constants
import swerve
import lift
import winch
import sys
from teleop import Teleop
from autonomous.baseline_simple import Autonomous
from sensors.imu import IMU


def log(src, msg):
    try:
        full_msg = "[{:.3f}] [{}] {}".format(
            wpilib.Timer.getMatchTime(), str(src), str(msg)
        )

        print(full_msg, file=sys.stderr)
    except:  # noqa: E772
        full_msg = "[{:.3f}] [log] Caught exception when logging: {} {}".format(
            wpilib.Timer.getMatchTime(),
            str(sys.exc_info()[0]),
            str(sys.exc_info()[1])
        )
        
        print(full_msg, file=sys.stderr)


def log_exception(src, locstr):
    # i.e. caught {ValueError} {in my_method}: {could not cast X to Y}
    log(src, "Caught {} {}: {}".format(
        str(sys.exc_info()[0]), locstr, str(sys.exc_info()[1])
    ))


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        constants.load_control_config()

        wpilib.CameraServer.launch('driver_vision.py:main')

        self.autoPositionSelect = wpilib.SendableChooser()
        self.autoPositionSelect.addDefault('Middle-Baseline', 'Middle-Baseline')  # noqa: E501
        self.autoPositionSelect.addObject('Middle-Placement', 'Middle-Placement')  # noqa: E501
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
        try:
            self.lift.load_config_values()
            self.drivetrain.load_config_values()
        except:  # noqa: E772
            log_exception('disabled', 'when loading config')

        try:
            self.drivetrain.update_smart_dashboard()
            self.imu.update_smart_dashboard()
            self.lift.update_smart_dashboard()
            self.winch.update_smart_dashboard()

            wpilib.SmartDashboard.putNumber(
                "Throttle Pos", self.throttle.getRawAxis(constants.liftAxis)
            )
        except:  # noqa: E772
            log_exception('disabled', 'when updating SmartDashboard')

        try:
            self.lift.checkLimitSwitch()
        except:  # noqa: E772
            log_exception('disabled', 'when checking lift limit switch')

    def autonomousInit(self):
        try:
            self.drivetrain.load_config_values()
            self.lift.load_config_values()
        except:  # noqa: E772
            log_exception('auto-init', 'when loading config')

        autoPos = None
        try:
            autoPos = self.autoPositionSelect.getSelected()
        except:  # noqa: E772
            autoPos = None
            log_exception('auto-init', 'when getting robot start position')

        try:
            self.auto = Autonomous(self, autoPos)
        except:  # noqa: E772
            log_exception('auto-init', 'in Autonomous constructor')

        try:
            self.lift.checkLimitSwitch()
        except:  # noqa: E772
            log_exception('auto-init', 'when checking lift limit switch')

    def autonomousPeriodic(self):
        try:
            if self.sd_update_timer.hasPeriodPassed(0.5):
                self.auto.update_smart_dashboard()
                self.imu.update_smart_dashboard()
                self.drivetrain.update_smart_dashboard()
                self.lift.update_smart_dashboard()
                self.winch.update_smart_dashboard()
        except:  # noqa: E772
            log_exception('auto', 'when updating SmartDashboard')

        try:
            self.auto.periodic()
        except:  # noqa: E772
            # Stop everything.
            self.drivetrain.immediate_stop()
            self.lift.setLiftPower(0)
            self.claw.set_power(0)
            self.winch.stop()
            log_exception('auto', 'in auto :periodic()')

        try:
            self.lift.checkLimitSwitch()
        except:  # noqa: E772
            log_exception('auto', 'when checking lift limit switch')

    def teleopInit(self):
        try:
            self.teleop = Teleop(self)
        except:  # noqa: E772
            log_exception('teleop-init', 'in Teleop constructor')

        try:
            self.drivetrain.load_config_values()
            self.lift.load_config_values()
            constants.load_control_config()
        except:  # noqa: E772
            log_exception('teleop-init', 'when loading config')

        try:
            self.lift.checkLimitSwitch()
        except:  # noqa: E772
            log_exception('teleop-init', 'when checking lift limit switch')

    def teleopPeriodic(self):
        try:
            self.teleop.drive()
        except:  # noqa: E772
            log_exception('teleop', 'in drive control')
            self.drivetrain.immediate_stop()

        try:
            self.teleop.buttons()
        except:  # noqa: E772
            log_exception('teleop', 'in button handler')

        try:
            self.teleop.lift_control()
        except:  # noqa: E772
            log_exception('teleop', 'in lift_control')
            self.lift.setLiftPower(0)

        try:
            self.teleop.claw_control()
        except:  # noqa: E772
            log_exception('teleop', 'in claw_control')
            self.claw.set_power(0)

        try:
            self.teleop.winch_control()
        except:  # noqa: E772
            log_exception('teleop', 'in winch_control')
            self.winch.stop()

        try:
            self.lift.checkLimitSwitch()
        except:  # noqa: E772
            log_exception('teleop', 'in lift.checkLimitSwitch')

        if self.sd_update_timer.hasPeriodPassed(0.5):
            try:
                constants.load_control_config()
                self.drivetrain.load_config_values()
                self.lift.load_config_values()
            except:  # noqa: E772
                log_exception('teleop', 'when loading config')

            try:
                self.drivetrain.update_smart_dashboard()
                self.teleop.update_smart_dashboard()
                self.imu.update_smart_dashboard()
                self.lift.update_smart_dashboard()
                self.winch.update_smart_dashboard()
            except:  # noqa: E772
                log_exception('teleop', 'when updating SmartDashboard')


if __name__ == "__main__":
    wpilib.run(Robot)
