import wpilib
import sys


class Autonomous:
    def __init__(self, robot, robot_position):
        self.robot = robot
        if robot_position is None:
            robot_position = '[was None]'

        self.robot_position = robot_position

        self.start_timer = wpilib.Timer()
        self.start_timer.reset()
        self.start_timer.start()
        self.startup_routine = True
        self.start_timer_started = False

    def update_smart_dashboard(self):
        pass

    def periodic(self):
        # follow trajectory if need be
        try:
            if self.startup_routine:
                if not self.start_timer_started:
                    self.start_timer.reset()
                    self.start_timer.start()
                    self.start_timer_started = True
                else:
                    #
                    init_time = self.start_timer.get()
                    if init_time < 1.25:
                        self.robot.lift.setLiftPower(-0.6)
                        self.robot.drivetrain.set_all_module_angles(0)
                    elif init_time < 1.25+4:
                        self.robot.lift.setLiftPower(0)
                        self.robot.drivetrain.set_all_module_angles(0)
                        self.robot.drivetrain.set_all_module_speeds(150, True)
                    else:
                        self.robot.drivetrain.set_all_module_angles(0)
                        self.robot.drivetrain.set_all_module_speeds(0, True)
                        self.robot.drivetrain.reset_drive_position()
                        self.startup_routine = False
        except:  # noqa: E772
            print(
                "[auto] Caught exception in auto :periodic() - "
                + str(sys.exc_info()[0]),
                file=sys.stderr
            )

            self.robot.lift.setLiftPower(0)
            self.robot.claw.set_power(0)
            self.robot.drivetrain.set_all_module_angles(0)
            self.robot.drivetrain.set_all_module_speeds(0, True)
