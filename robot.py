import wpilib
import constants
import swerve
import lift
from teleop import Teleop
from autonomous import Autonomous
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

        self.control_stick = wpilib.Joystick(0)
        self.drivetrain = swerve.SwerveDrive(
            constants.chassis_length,
            constants.chassis_width,
            constants.swerve_config
        )

        self.lift = lift.RD4BLift(
            constants.lift_ids['left'],
            constants.lift_ids['right']
        )

        #self.claw = lift.Claw(
        #    constants.claw_id
        #)

        self.imu = IMU(wpilib.SPI.Port.kMXP)

    def disabledInit(self):
        # We don't really _need_ to reload configuration in
        # every init call-- it's just useful for debugging.
        # (no need to restart robot code just to load new values)
        self.drivetrain.load_config_values()

    def disabledPeriodic(self):
        self.drivetrain.update_smart_dashboard()
        self.imu.update_smart_dashboard()

    def autonomousInit(self):
        self.drivetrain.load_config_values()
        self.auto = Autonomous(self, self.autoPositionSelect.getSelected())
        self.auto.periodic()

    def autonomousPeriodic(self):
        self.auto.update_smart_dashboard()
        self.imu.update_smart_dashboard()
        self.drivetrain.update_smart_dashboard()

        self.auto.periodic()

    def teleopInit(self):
        self.teleop = Teleop(self, self.control_stick)
        self.drivetrain.load_config_values()
        constants.load_control_config()

    def teleopPeriodic(self):
        # For now: basic driving
        constants.load_control_config()

        self.teleop.drive()
        self.teleop.buttons()
        self.teleop.lift_control()

        self.drivetrain.update_smart_dashboard()
        self.teleop.update_smart_dashboard()
        self.imu.update_smart_dashboard()


if __name__ == "__main__":
    wpilib.run(Robot)
