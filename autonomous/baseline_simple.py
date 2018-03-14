import math
import wpilib
import sys


class Autonomous:
    def __init__(self, robot, robot_position):
        self.robot = robot
        if robot_position is None:
            robot_position = ''

        try:
            self.robot_position = robot_position.lower()
        except:  # noqa: E772
            self.robot_position = ''

        self.timer = wpilib.Timer()
        self.timer.reset()
        self.timer.start()

        self.field_string = ''
        ds = wpilib.DriverStation.getInstance()
        while self.timer.get() < 1 and len(self.field_string) == 0:
            fms_message = ds.getGameSpecificMessage().decode("utf-8")
            if fms_message is None:
                fms_message = ''

            self.field_string = fms_message.upper()

        self.drive_speed = 150

        # Note: positive angles = rightward
        # negative angles = leftward
        try:
            if self.robot_position == 'left':
                self.drive_angle = math.radians(-15)
            elif self.robot_position == 'right':
                self.drive_angle = math.radians(15)
            elif self.robot_position == 'middle-placement':
                self.drive_angle = 0
            else:
                self.drive_angle = 0
        except:  # noqa: E772
            self.drive_angle = 0

        if self.field_string != '':
            print("[auto] Got field string in {:.3f} ms: {}".format(
                self.timer.get(), self.field_string*1000
            ))

            # Set drive angle to zero if switch position matches robot position
            if (
                (self.field_string[0] == 'L' and self.robot_position == 'left')
                or (self.field_string[0] == 'R' and self.robot_position == 'right')  # noqa: E501
                or self.robot_position == 'middle-placement'
            ):
                self.drive_angle = 0
                self.drive_speed = 250

                if self.robot_position == 'left':
                    self.drive_angle = math.radians(15)
                elif self.robot_position == 'right':
                    self.drive_angle = math.radians(-15)
                elif self.robot_position == 'middle-placement':
                    if self.field_string[0] == 'R':
                        self.drive_angle = math.radians(25)
                    elif self.field_string[0] == 'L':
                        self.drive_angle = math.radians(-30)

                print("[auto] Driving into switch at angle={:.3f}".format(
                    self.drive_angle
                ), file=sys.stderr)
            else:
                print("[auto] Diverting at angle={:.3f}".format(
                    self.drive_angle
                ), file=sys.stderr)

            print("[auto] Driving at speed={}".format(self.drive_speed))

        self.start_timer = wpilib.Timer()
        self.start_timer.reset()
        self.start_timer.start()
        self.startup_routine = True
        self.start_timer_started = False

    def update_smart_dashboard(self):
        pass

    def periodic(self):
        try:
            if self.startup_routine:
                if not self.start_timer_started:
                    self.start_timer.reset()
                    self.start_timer.start()
                    self.start_timer_started = True
                else:
                    init_time = self.start_timer.get()
                    self.robot.drivetrain.set_all_module_angles(
                        self.drive_angle
                    )

                    if init_time < 0.5:
                        self.robot.lift.setLiftPower(-0.2)
                    elif init_time < 0.5+4:
                        self.robot.drivetrain.set_all_module_speeds(self.drive_speed, True)
                    elif init_time < 0.5+4+1.5:
                        self.robot.lift.setLiftPower(-0.6)
                        self.robot.drivetrain.set_all_module_speeds(0, True)
                    elif init_time < 0.5+4+1.5+1:
                        self.robot.lift.setLiftPower(0)
                        self.robot.claw.set_power(-0.5)
                        self.robot.drivetrain.set_all_module_speeds(0, True)
                    else:
                        self.robot.lift.setLiftPower(0)
                        self.robot.claw.set_power(0)
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
