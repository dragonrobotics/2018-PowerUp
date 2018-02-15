import wpilib
import claw
from robotpy_ext.control.button_debouncer import ButtonDebouncer


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.control_stick = wpilib.Joystick(0)

        self.open_claw = ButtonDebouncer(self.control_stick, 1)
        self.close_claw = ButtonDebouncer(self.control_stick, 2)

        self.claw = claw.Claw(2, 0)

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        if self.open_claw.get():
            self.claw.open()
        elif self.close_claw.get():
            self.claw.close()

        self.claw.update()


if __name__ == "__main__":
    wpilib.run(Robot)
