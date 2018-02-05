"""
Autonomous module.

This module contains classes and functions for autonomous, which is designed as
a finite-state automaton updated per tick in autonomousPeriodic.

Autonomous transitions between these states:

    `init`: The robot closes the claw, fully lowers the lift, and transitions
            onto the `turn` state to angle toward the next waypoint.
    `turn`: The swerve modules (not the entire chassis) angle toward the next
            waypoint, then transitions into the drive state
    `drive`: The robot drives over to the next waypoint, then transitions into
             the turning state or the lifting state if there are no other
             waypoints.
    `lift`: the RD4B lifts to a predetermined height (either the height of the
            scale or switch), then transitions to the target states.
    `target-turn`: Turns the entire chassis towards the target (either the
                   switch or the scale).
    `target-drive`: Drives the robot towards the target. This differs slightly
                    from the normal `drive` state because it uses sensors to
                    be more accurate.
    `drop`: The claw opens.
"""

import math
import wpilib
import numpy as np
from collections import deque


class Autonomous:
    PATHS = {
        "default": [],

        "1": [
                # change this
                (-40, -40),
                (-20, 10),
                (20, 0),
                (40, -10),
                (0, 0),
                (40, 40),
                (30, 30)
             ],

        "2": [
                # add waypoints here.
             ],

        "3": []  # etc
    }

    ##################################################################
    # Internal code starts here.
    ##################################################################

    turn_angle_tolerance = 2.5  # degrees
    drive_dist_tolerance = 3  # inches
    lift_height_tolerance = 2  # inches

    drive_speed = 100  # talon native units / 100ms

    init_lift_height = 6  # inches above ground for initialization

    def __init__(self, robot, robot_position):
        """
        Initialize autonomous.

        This constructor mainly initializes the software.
        The correct path is chosen from SmartDashboard and initialized into
        numpy-based waypoints.  The current position is set.  Then, the state
        is set to 'init' and physical initializations are done there.

        Parameters:
            robot: the robot instance.
        """

        # basic initalization.
        self.robot = robot
        self.target = None
        self.target_height = 36  # inches -- set this according to target
        self.final_drive_dist = 6  # inches
        self.robot.drivetrain.rezero_distance()

        # get preferences and the field string from the Game Data API.
        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()
        prefs = wpilib.Preferences.getInstance()

        if field_string == "":  # this only happens during tests
            field_string = 'LLL'

        # Get the corresponding path of the given field string from preferences
        path = self.PATHS[prefs.getString(field_string, backup='default')]
        self.waypoints = [np.asarray(point) for point in path]

        # set current position. TODO: implement.
        self.current_pos = np.array([0, 0])

        # active waypoint: the waypoint we are currently headed towards.
        self.active_waypoint_idx = 0

        self.state = 'init'
        self.__module_angle_err_window = deque([], 30)

    def state_init(self):
        """
        Perform robot-oriented initializations.

        Close the claw and set the lift to its lowest position, then transition
        into the turning state.
        """

        self.robot.claw.close()  # put claw into 'closing' state
        self.robot.lift.set_height(self.init_lift_height)

        # if everything is set correctly, transition to the turning state.
        if (
            self.robot.claw.state == 'closed'
            and abs(self.robot.lift.get_height() - self.init_lift_height)
            <= self.lift_height_tolerance
        ):
            self.state = 'turn'

    def state_turn(self):
        """
        Turn the swerve modules to their desired angle.
        Then transition into the `drive` state.
        """

        # stop the drivetrain. Otherwise the robot will do weird curves.
        self.robot.drivetrain.set_all_module_speeds(0, direct=True)

        # get the active waypoint, and from there calculate the displacement
        # vector relative to the robot's position.
        active_waypoint = self.waypoints[self.active_waypoint_idx]
        disp_vec = self.current_pos - active_waypoint

        # trigonometry to find the angle, then set the module angles.
        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        self.robot.drivetrain.set_all_module_angles(tgt_angle)
        self.robot.drivetrain.set_all_module_speeds(0, direct=True)

        cur_error = np.array(
            self.robot.drivetrain.get_closed_loop_error(),
            dtype=np.float64
        )

        cur_error *= 180 / 512

        max_err = np.amax(np.abs(cur_error))
        self.__module_angle_err_window.append(max_err)
        avg_max_err = np.mean(self.__module_angle_err_window)

        if (
            len(self.__module_angle_err_window) >= self.__angle_err_window.maxlen / 2  # noqa: E501
            and avg_max_err < self.turn_angle_tolerance
        ):
            self.robot.drivetrain.reset_drive_position()
            self.state = 'drive'

    def state_drive(self):
        """
        Drive toward a waypoint.
        Calculate the distance, then move the modules that distance.
        """

        # get active waypoint and calculate displacement vector.
        active_waypoint = self.waypoints[self.active_waypoint_idx]
        disp_vec = self.current_pos - active_waypoint

        # calculate distance with pythagorean theorem
        dist = np.sqrt(np.sum(disp_vec**2))

        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        self.robot.drivetrain.set_all_module_angles(tgt_angle)

        # get the average distance the robot has gone so far
        avg_dist = np.mean(self.robot.drivetrain.get_module_distances())
        self.robot.drivetrain.set_all_module_speeds(self.drive_speed, True)

        # is the distance traveled somewhere close to the distance needed to
        # travel?
        if abs(avg_dist - dist) <= self.drive_dist_tolerance:

            # do we have waypoints left to drive to?
            self.active_waypoint_idx += 1

            # update the new current position as the active waypoint.
            self.current_pos = active_waypoint

            # do we still have waypoints left to go?
            if self.active_waypoint_idx < len(self.waypoints):
                self.__module_angle_err_window.clear()
                self.robot.drivetrain.reset_drive_position()
                self.state = 'turn'
            else:
                self.state = 'lift'

    def state_lift(self):
        """
        Lift the RD4B to the height needed.
        Then transition to the target states for final adjustments.
        """

        # stop the drivetrain.
        self.robot.drivetrain.set_all_module_speeds(0, True)

        # set the height of the RD4B.
        self.robot.lift.set_height(self.target_height)

        # is the height there yet? if so, transition.
        if (
            abs(self.robot.lift.get_height() - self.target_height)
            <= self.lift_height_tolerance
        ):
            self.state = 'target-turn'

    def state_target_turn(self):
        """
        Complete a targeted, measured turn towards the target, turning the
        entire chassis this time instead of just each module.
        """

        # the usual displacement stuff.
        disp_vec = self.current_pos - self.target
        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])

        # we are actually going to turn the whole chassis this time, using the
        # navx to ensure we are doing things correctly.
        self.robot.drivetrain.turn_to_angle(self.robot.navx, tgt_angle)

        hdg = self.navx.getFusedHeading()
        prefs = wpilib.Preferences.getInstance()

        if prefs.getBoolean('Reverse Heading Direction', False):
            hdg *= -1

        # if we are at the proper angle now, move to the target-drive state.
        if abs(hdg - math.degrees(tgt_angle)) <= self.turn_angle_tolerance:
            self.robot.drivetrain.set_all_module_speeds(0, True)
            self.state = 'target-drive'

    def state_target_drive(self):
        """
        Drive a measured distance towards the target.
        This should position the claw, with the cube, right over the switch or
        scale.
        """

        avg_dist = np.mean(self.robot.drivetrain.get_module_distances())

        self.robot.drivetrain.set_all_module_angles(0)
        self.robot.drivetrain.set_all_module_speeds(self.drive_speed, True)

        if abs(avg_dist - self.final_drive_dist) <= self.drive_dist_tolerance:
            self.state = 'drop'

    def state_drop(self):
        """
        Open the claw and drop the cube.
        """
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
