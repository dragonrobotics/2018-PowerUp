"""
Autonomous module.

This module contains classes and functions for autonomous, which is designed as
a finite-state automaton updated per tick in autonomousPeriodic.

Autonomous transitions between these states:

    'init': The robot closes the claw, fully lowers the lift, and transitions
            onto the 'turn' state to angle toward the next waypoint.
    'turn': The swerve modules (not the entire chassis) angle toward the next
            waypoint, then transitions into the drive state
    'drive': The robot drives over to the next waypoint, then transitions into
             the turning state or the lifting state if there are no other
             waypoints.
    'lift': the RD4B lifts to a predetermined height (either the height of the
            scale or switch), then transitions to the drop state.
    'drop': The claw opens.

"""
import math
import wpilib
import numpy as np


class Autonomous:

    PATHS = {
        """
        Put the waypoints here.
        Each path is represented in this form:
        "name of path": [list of waypoints (tuples of coordinates in inches)]
        The name of the path will correspond to the one in preferences.
        """

        "1": [
                # change this
                (-40, -40),
                (-20, 10),
                (20, 0),
                (40, -10),
                (0, 0),
                (40, 40),
                (30, 30)
             ]

        "2": [
                # add waypoints here.
             ]

        "3": [] # etc

    }


    ##################################################################
    ################### Internal Code starts here ####################
    ##################################################################

    turn_angle_tolerance = 2.5  # degrees
    drive_dist_tolerance = 3  # inches
    lift_height_tolerance = 2  # inches

    drive_speed = 100  # talon native units / 100ms

    init_lift_height = 6  # inches above ground for initialization

    def __init__(self, robot):
        """
        Initialize autonomous.

        This constructor mainly initializes the software.
        The correct path is chosen from SmartDashboard and initialized into
        numpy-based waypoints.  The current position is set.  Then, the state
        is set to 'init' and physical initializations are done there.

        @param robot the robot instance.
        """

        self.robot = robot
        self.target = None
        self.target_height = 36  # inches-- set this according to target
        self.final_drive_dist = 6  # inches

        # get preferences and the field string from the Game Data API.
        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()
        prefs = wpilib.Preferences.getInstance()

        if field_string == "":  # this only happens during tests
            field_string = 'LLL'

        # Get the corresponding path of the given field string from preferences.
        path = self.PATHS[prefs.getString(field_string)]
        self.waypoints = [np.asarray(point) for point in path]

        # set current position. TODO: implement.
        self.current_pos = np.array([0, 0])
        self.active_waypoint_idx = 0

        self.state = 'turn'
        self.__module_angle_err_window = []
        self.robot.drivetrain.rezero_distance()

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
        self.robot.drivetrain.set_all_module_speeds(0, direct=True)
        active_waypoint = self.waypoints[self.active_waypoint_idx]
        disp_vec = self.current_pos - active_waypoint

        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        self.robot.drivetrain.set_all_module_angles(tgt_angle)

        cur_error = np.array(self.robot.drivetrain.get_closed_loop_error(), dtype=np.float64)
        cur_error *= 180 / 512

        wpilib.SmartDashboard.putString(
            'error',
            str(cur_error)
        )

        max_err = np.amax(np.abs(cur_error))
        self.__module_angle_err_window.append(max_err)
        if len(self.__module_angle_err_window) > 50:
            self.__module_angle_err_window = self.__module_angle_err_window[-50:]
        avg_max_err = np.mean(self.__module_angle_err_window)

        wpilib.SmartDashboard.putNumber(
            'max_err',
            max_err
        )

        wpilib.SmartDashboard.putNumber(
            'avg_max_err',
            avg_max_err
        )

        if len(self.__module_angle_err_window) >= 30 and avg_max_err < self.turn_angle_tolerance:
            self.robot.drivetrain.rezero_distance()
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
                self.__module_angle_err_window = []
                self.robot.drivetrain.rezero_distance()
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

    """
    Maps state names to functions.
    """
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
