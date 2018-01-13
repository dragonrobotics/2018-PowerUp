import wpilib
import constants
import swerve
import teleop


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        constants.load_control_config()

        self.control_stick = wpilib.Joystick(0)
        self.drivetrain = swerve.SwerveDrive(
            constants.chassis_length,
            constants.chassis_width,
            constants.swerve_config
        )

    def disabledInit(self):
        # We don't really _need_ to reload configuration in
        # every init call-- it's just useful for debugging.
        # (no need to restart robot code just to load new values)
        self.drivetrain.load_config_values()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.drivetrain.load_config_values()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.drivetrain.load_config_values()
        constants.load_control_config()

    def teleopPeriodic(self):
        # For now: basic driving
        teleop.drive(self.control_stick, self.drivetrain)
        self.drivetrain.update_smart_dashboard()


if __name__ == "__main__":
    wpilib.run(Robot)
