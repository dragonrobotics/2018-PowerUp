"""
The entry point for all robot code.
"""


import wpilib
import constants
import swerve
import teleop
from autonomous import Autonomous


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        """
        Perform initialization on robot startup.

        This function is called when the robot code starts after being powered
        on; at this point everything within WPILib / NetworkTables / etc.
        has been initialized.
        """
        constants.load_control_config()

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

    def disabledInit(self):
        """
        Called once whenever the robot is disabled.

        This can't, and probably shouldn't, do much. It could be helpful to
        reload robot configuration values here (from Preferences, for instance)
        for testing purposes.
        """
        # We don't really _need_ to reload configuration in
        # every init call-- it's just useful for debugging.
        # (no need to restart robot code just to load new values)
        self.drivetrain.load_config_values()

    def disabledPeriodic(self):
        """
        Called periodically whenever the robot is disabled.

        To be specific, this function is called whenever a new packet has been
        received from the Driver Station.

        This probably shouldn't do much other than update SmartDashboard.
        """
        pass

    def autonomousInit(self):
        """
        Called once when entering Autonomous mode.
        """
        self.drivetrain.load_config_values()
        self.auto = Autonomous(self.autoPositionSelect.getSelected())

    def autonomousPeriodic(self):
        """
        Called periodically when in Autonomous mode.

        To be specific, this function is called whenever a new packet has been
        received from the Driver Station.

        Warning:
            This function **must not** block the robot program for more than
            a few milliseconds at a time. Blocking will cause all sorts of
            problems with the robot and may leave it unresponsive.
        """
        pass

    def teleopInit(self):
        """
        Called once when entering Teleop mode.
        """
        self.drivetrain.load_config_values()
        constants.load_control_config()

    def teleopPeriodic(self):
        """
        Called periodically when in Teleop mode.

        To be specific, this function is called whenever a new packet has been
        received from the Driver Station.

        Warning:
            This function **must not** block the robot program for more than
            a few milliseconds at a time. Blocking will cause all sorts of
            problems with the robot and may leave it unresponsive.
        """
        # For now: basic driving
        teleop.drive(self.control_stick, self.drivetrain)
        self.drivetrain.update_smart_dashboard()


if __name__ == "__main__":
    wpilib.run(Robot)
