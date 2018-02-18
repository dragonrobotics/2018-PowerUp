"""
Autonomous module.

This module contains classes and functions for autonomous, which is designed as
a finite-state automaton updated per tick in autonomousPeriodic.

Autonomous transitions between these states:

    - **init**: The robot closes the claw, fully lowers the lift, and
      transitions onto the **turn** state to angle toward the next waypoint.
    - **turn**: The swerve modules (not the entire chassis) angle toward the
      next waypoint, then transitions into the drive state
    - **drive**: The robot drives over to the next waypoint, then transitions
      into the turning state or the lifting state if there are no other
      waypoints.
    - **lift**: the RD4B lifts to a predetermined height (either the height of
      the scale or switch), then transitions to the target states.
    - **target-turn**: Turns the entire chassis towards the target (either the
      switch or the scale).
    - **target-drive**: Drives the robot towards the target. This differs
      slightly from the normal **drive** state because it uses sensors to be
      more accurate.
    - **drop**: The claw opens.

"""

import math
import wpilib
import numpy as np
from collections import deque
from ctre.talonsrx import TalonSRX

start_pos_left = np.array((21.25, 82.5))
start_pos_middle = np.array((21.25, 197))
start_pos_right = np.array((21.25, 263.5))

left_switch = np.array((168, 164-54))
right_switch = np.array((168, 164+54))

staging_left = np.array((120, 48.5))
staging_right = np.array((120, 279.5))

align_pt_left = np.array((168, 48.5))
align_pt_right = np.array((168, 279.5))

class Autonomous:
    """
    This class implements a finite-state automaton for controlling autonomous.
    Also, this class includes a way to input waypoint coordinates into a
    a template to be automatically selected and formatted as necessary.

    Parameters:
        robot: the robot instance.
        robot_position: the position of the robot on the field.

    """

    #: Dictionary of paths (arrays of waypoints).  The right one is chosen
    #: at runtime.
    PATHS = {
        "l-drive-left": [
            start_pos_left,
            staging_left,
            align_pt_left
        ],

        "l-drive-right": [
            start_pos_right,
            staging_right,
            align_pt_right
        ],

        "direct-left": [
            staging_left,
            align_pt_left
        ],

        "direct-right": [
            staging_right,
            align_pt_right
        ]
    }

    ##################################################################
    # Internal code starts here.
    ##################################################################

    turn_angle_tolerance = math.radians(2.5)  #: a tolerance range for turning
    drive_dist_tolerance = 3  #: a tolerance range for driving, in inches.
    lift_height_tolerance = 2  #: a tolerance range for lifting, in inches.
    drive_speed = 100  #: how fast to drive, in native units per 100ms
    init_lift_height = 6  #: initial lift height, in inches above the ground.

    def __init__(self, robot, robot_position):
        """
        Initialize autonomous.

        This constructor mainly initializes the software.
        The correct path is chosen from SmartDashboard and initialized into
        numpy-based waypoints.  The current position is set.  Then, the state
        is set to 'init' and physical initializations are done there.
        """

        # basic initalization.
        self.robot = robot
        self.target = None
        self.target_height = 36  # inches -- set this according to target
        self.final_drive_dist = 12  # inches

        self.robot.drivetrain.reset_drive_position()
        self.robot.imu.reset()

        # get preferences and the field string from the Game Data API.
        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage().upper()
        prefs = wpilib.Preferences.getInstance()

        if field_string == "":  # this only happens during tests
            field_string = 'LLL'

        if field_string[0] == 'L':
            self.target = left_switch_coords
            self.init_turn_angle = math.radians(270)

            self.waypoints = self.PATHS['direct-left']
        else:
            self.target = right_switch_coords
            self.init_turn_angle = math.radians(90)

            self.waypoints = self.PATHS['direct-right']

        # set current position. TODO: implement.
        self.current_pos = np.array([0, 0])

        if robot_position == 'Left':
            self.current_pos = start_pos_left
        elif robot_position == 'Middle':
            self.current_pos = start_pos_middle
        elif robot_position == 'Right':
            self.current_pos = start_pos_right

        # active waypoint: the waypoint we are currently headed towards.
        self.active_waypoint_idx = 0

        self.state = 'init'
        self.__module_angle_err_window = deque([], 30)

        self.hack_timer = wpilib.Timer()
        self.hack_timer_started = False

    def state_init(self):
        """
        Perform robot-oriented initializations.

        Close the claw and set the lift to its lowest position, then transition
        into the turning state.
        """
        self.robot.drivetrain.set_all_module_angles(0)

        if not self.hack_timer_started:
            self.hack_timer.reset()
            self.hack_timer.start()
            self.hack_timer_started = True
        else:
            #
            init_time = self.hack_timer.get()
            if init_time < 0.5:
                self.robot.lift.setLiftPower(-0.6)
                self.robot.claw.talon.set(
                    TalonSRX.ControlMode.PercentOutput, 1
                )
                self.robot.drivetrain.set_all_module_speeds(150, True)
            elif init_time < 1:
                self.robot.lift.setLiftPower(0)
                self.robot.claw.talon.set(
                    TalonSRX.ControlMode.PercentOutput, 0
                )
                self.robot.drivetrain.set_all_module_speeds(200, True)
            elif init_time < 1.5:
                self.robot.drivetrain.set_all_module_speeds(-200, True)
            elif self.hack_timer.get() > 1.5:
                self.robot.drivetrain.set_all_module_speeds(0, True)
                self.robot.claw.talon.set(
                    TalonSRX.ControlMode.PercentOutput, 0
                )

                self.hack_timer_started = False

                # Get distance driven forwards during the unfold maneuver
                # By default we drive in the -X direction apparently?
                dist = np.mean(self.robot.drivetrain.get_module_distances())
                dist *= (4 * math.pi) / (80 * 6.67)
                self.current_pos[0] -= dist

                self.state = 'init-turn'

    def state_init_turn(self):
        self.robot.drivetrain.drive(0, 0, 0.1)
        hdg = self.robot.imu.get_robot_heading()

        # if we are at the proper angle now, move to the target-drive state.
        if abs(hdg - self.init_turn_angle) <= self.turn_angle_tolerance:
            self.robot.drivetrain.set_all_module_speeds(0, True)
            self.robot.drivetrain.reset_drive_position()
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
        disp_vec = active_waypoint - self.current_pos

        # trigonometry to find the angle, then set the module angles.
        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        tgt_angle += self.robot.imu.get_robot_heading()

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
            len(self.__module_angle_err_window) >= self.__module_angle_err_window.maxlen  # noqa: E501
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
        disp_vec = active_waypoint - self.current_pos

        # calculate distance with pythagorean theorem
        dist = np.sqrt(np.sum(disp_vec**2))

        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])
        tgt_angle += self.robot.imu.get_robot_heading()
        self.robot.drivetrain.set_all_module_angles(tgt_angle)

        # get the average distance the robot has gone so far
        avg_dist = np.mean(self.robot.drivetrain.get_module_distances())
        avg_dist *= (4 * math.pi) / (80 * 6.67)
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
                self.state = 'target-turn'

    def state_lift(self):
        """
        Lift the RD4B to the height needed.
        Then transition to the target states for final adjustments.
        """

        # stop the drivetrain.
        self.robot.drivetrain.set_all_module_speeds(0, True)

        if not self.hack_timer_started:
            self.hack_timer.reset()
            self.hack_timer.start()
            self.hack_timer_started = True
        else:
            if self.hack_timer.get() < 1.5:
                self.robot.lift.setLiftPower(-0.6)
            else:
                self.robot.lift.setLiftPower(0)
                self.state = 'target-turn'

    def state_target_turn(self):
        """
        Complete a targeted, measured turn towards the target, turning the
        entire chassis this time instead of just each module.
        """

        # the usual displacement stuff.
        disp_vec = self.target - self.current_pos
        tgt_angle = np.arctan2(disp_vec[1], disp_vec[0])

        # we are actually going to turn the whole chassis this time, using the
        # navx to ensure we are doing things correctly.
        self.robot.drivetrain.drive(0, 0, 0.1)
        hdg = self.robot.imu.get_robot_heading()

        # if we are at the proper angle now, move to the target-drive state.
        if abs(hdg - tgt_angle) <= self.turn_angle_tolerance:
            self.robot.drivetrain.set_all_module_speeds(0, True)
            self.robot.drivetrain.reset_drive_position()
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

        if not self.hack_timer_started:
            self.hack_timer.reset()
            self.hack_timer.start()
            self.hack_timer_started = True
        else:
            t = self.hack_timer.get()
            if t < 0.5:
                self.robot.claw.talon.set(
                    TalonSRX.ControlMode.PercentOutput, -1
                )
            else:
                self.robot.claw.talon.set(
                    TalonSRX.ControlMode.PercentOutput, 0
                )


    #: Maps state names to functions.
    state_table = {
        'init': state_init,
        'init-turn': state_init_turn,
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

    def update_smart_dashboard(self):
        """
        Periodically call this function to update the smartdashboard with
        important information about the autonomous class, for troubleshooting/
        monitoring purposes.
        """

        wpilib.SmartDashboard.putString(
            'autonomous state',
            self.state
        )

        wpilib.SmartDashboard.putString(
            'Current Auto Position',
            str(self.current_pos)
        )

        if self.active_waypoint_idx < len(self.waypoints):
            active_waypoint = self.waypoints[self.active_waypoint_idx]
            wpilib.SmartDashboard.putString(
                'Active Waypoint',
                str(active_waypoint)
            )
