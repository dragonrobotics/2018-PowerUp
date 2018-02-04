""" collection of functions used to manage autonomous control """
# from vision.visionmaster import VisionMaster
import math
import wpilib
import numpy as np
from collections import deque


class Autonomous:
    turn_angle_tolerance = 2.5  # degrees
    drive_dist_tolerance = 3  # inches
    lift_height_tolerance = 2  # inches

    drive_speed = 100  # talon native units / 100ms

    init_lift_height = 6  # inches above ground for initialization

    def __init__(self, robot, robot_location):
        self.robot = robot
        self.robot_location = robot_location  # set on SmartDashboard

        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()

        if field_string == "":  # this only happens during tests
            field_string = 'LLL'

        self.close_switch = field_string[0]
        self.scale = field_string[1]
        self.far_switch = field_string[2]

        self.target = None
        self.target_height = 36  # inches-- set this according to target
        self.final_drive_dist = 6  # inches

        self.waypoints = [
            np.array([-40, -40]),
            np.array([-20, 10]),
            np.array([20, 0]),
            np.array([40, -10]),
            np.array([0, 0]),
            np.array([40, 40]),
            np.array([30, 30])
        ]
        self.current_pos = np.array([0, 0])
        self.active_waypoint_idx = 0

        self.state = 'turn'
        self.__angle_err_window = deque([], 30)

        if self.robot_location == 'Middle':
            if self.close_switch == 'L':
                pass  # We are in middle and switch is on left.
            else:
                pass  # We are in middle and switch is on right.
        elif self.robot_location == 'Left':
            if self.close_switch == 'L':
                pass  # We are on left and switch is on left.
            else:
                pass  # We are on left and switch is on right.
        elif self.robot_location == 'Right':
            if self.close_switch == 'L':
                pass  # We are on right and switch is on left.
            else:
                pass  # We are on right and switch is on right.

        self.robot.drivetrain.reset_drive_position()

    def state_init(self):
        self.robot.claw.close()  # put claw into 'closing' state
        self.robot.lift.set_height(self.init_lift_height)

        if (
            self.robot.claw.state == 'closed'
            and abs(self.robot.lift.get_height() - self.init_lift_height)
            <= self.lift_height_tolerance
        ):
            self.state = 'turn'

    def state_turn(self):
        active_waypoint = self.waypoints[self.active_waypoint_idx]
        disp_vec = self.current_pos - active_waypoint

        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        self.robot.drivetrain.set_all_module_angles(tgt_angle)
        self.robot.drivetrain.set_all_module_speeds(0, direct=True)

        cur_error = np.array(
            self.robot.drivetrain.get_closed_loop_error(),
            dtype=np.float64
        )

        cur_error *= 180 / 512

        max_err = np.amax(np.abs(cur_error))
        self.__angle_err_window.append(max_err)
        avg_max_err = np.mean(self.__angle_err_window)

        if (
            len(self.__angle_err_window) >= self.__angle_err_window.maxlen / 2
            and avg_max_err < self.turn_angle_tolerance
        ):
            self.robot.drivetrain.reset_drive_position()
            self.state = 'drive'

    def state_drive(self):
        active_waypoint = self.waypoints[self.active_waypoint_idx]
        disp_vec = self.current_pos - active_waypoint

        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        dist = np.sqrt(np.sum(disp_vec**2))

        self.robot.drivetrain.set_all_module_angles(tgt_angle)

        avg_dist = np.mean(self.robot.drivetrain.get_module_distances())
        self.robot.drivetrain.set_all_module_speeds(self.drive_speed, True)
        if abs(avg_dist - dist) <= self.drive_dist_tolerance:
            # do we have waypoints left to drive to?
            self.active_waypoint_idx += 1
            self.current_pos = active_waypoint

            if self.active_waypoint_idx < len(self.waypoints):
                self.__angle_err_window.clear()
                self.robot.drivetrain.reset_drive_position()
                self.state = 'turn'
            else:
                self.state = 'lift'

    def state_lift(self):
        self.robot.drivetrain.set_all_module_speeds(0, True)
        self.robot.lift.set_height(self.target_height)

        if (
            abs(self.robot.lift.get_height() - self.target_height)
            <= self.lift_height_tolerance
        ):
            self.state = 'target-turn'

    def state_target_turn(self):
        disp_vec = self.current_pos - self.target
        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])

        self.robot.drivetrain.turn_to_angle(self.robot.navx, tgt_angle)

        hdg = self.navx.getFusedHeading()
        prefs = wpilib.Preferences.getInstance()

        if prefs.getBoolean('Reverse Heading Direction', False):
            hdg *= -1

        if abs(hdg - math.degrees(tgt_angle)) <= self.turn_angle_tolerance:
            self.robot.drivetrain.set_all_module_speeds(0, True)
            self.state = 'target-drive'

    def state_target_drive(self):
        avg_dist = np.mean(self.robot.drivetrain.get_module_distances())

        self.robot.drivetrain.set_all_module_angles(0)
        self.robot.drivetrain.set_all_module_speeds(self.drive_speed, True)

        if abs(avg_dist - self.final_drive_dist) <= self.drive_dist_tolerance:
            self.state = 'drop'

    def state_drop(self):
        self.robot.drivetrain.set_all_module_speeds(0, True)
        self.robot.claw.open()

    '''
    Maps state names to functions.
    '''
    state_table = {
        'init': state_init,
        'turn': state_turn,
        'drive': state_drive,
        'lift': state_lift,
        'target-turn': state_target_turn,
        'target-drive': state_target_drive,
        'drop': state_drop
    }

    def periodic(self):
        """
        Updates and progresses the autonomous state machine.
        """
        # TODO: Only run autonomous if in Drive or Turn states;
        # this is for testing purposes only.
        if self.state == "drive" or self.state == "turn":
            # Call function corresponding to current state.
            self.state_table[self.state](self)
        else:
            self.robot.drivetrain.set_all_module_speeds(0, direct=True)

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putString(
            'autonomous state',
            self.state
        )
