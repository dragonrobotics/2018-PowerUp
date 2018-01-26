""" collection of functions used to manage autonomous control """
# from vision.visionmaster import VisionMaster
import math
import wpilib
import numpy as np


class Autonomous:
    turn_angle_tolerance = 2.5  # degrees
    drive_dist_tolerance = 3  # inches
    lift_height_tolerance = 2  # inches

    drive_speed = 200  # talon native units / 100ms

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

        self.waypoints = []
        self.current_pos = np.array([0, 0])
        self.active_waypoint_idx = 0

        self.state = 'init'

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

        cur_angles = np.array(self.robot.drivetrain.get_module_angles())

        max_err = np.amax(np.abs(tgt_angle - cur_angles))

        if math.degrees(max_err) < self.turn_angle_tolerance:
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

            if self.active_waypoint_idx < len(self.waypoints):
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
        # Call function corresponding to current state.
        self.state_table[self.state](self)
