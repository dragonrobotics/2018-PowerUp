import os.path
import pickle
import sys
import wpilib
import numpy as np
from numpy import pi
import pathfinder as pf
from pathfinder.followers import EncoderFollower
import constants

trajectory_file = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')
trajectories = {
    'left': None,
    'right': None,
    'divert-left': None,
    'divert-right': None,
    'straght-forward': None,
}

_trajectory_dt = 0.05  # time in seconds between control updates
_max_speed = 200 * 10 * (4 * pi) / (80 * 6.67) * 0.0254

if wpilib.RobotBase.isSimulation():
    # waypoint specification:
    # relative x, y coordinates in meters; exit angle in radians

    # distance to switch fence = 140in
    start_pos_left = np.array((21.25, 82.5))
    start_pos_middle = np.array((21.25, 197))
    start_pos_right = np.array((21.25, 263.5))

    left_switch = np.array((136, 164-54))
    right_switch = np.array((136, 164+54))

    staging_left = np.array((136, 48.5))
    staging_mid = np.array((60, 164))
    staging_right = np.array((136, 279.5))

    align_pt_left = np.array((120, 164-54))
    align_pt_right = np.array((120, 164+54))

    left_leg1 = (staging_mid - start_pos_middle) * 0.0254
    left_leg2 = (align_pt_left - start_pos_middle) * 0.0254
    left_leg3 = (left_switch - start_pos_middle) * 0.0254

    right_leg1 = (align_pt_right - start_pos_middle) * 0.0254
    right_leg2 = (right_switch - start_pos_middle) * 0.0254

    ldiv_leg1 = (np.array((36, 48.5)) - start_pos_left) * 0.0254
    ldiv_leg2 = (staging_left - start_pos_left) * 0.0254

    rdiv_leg1 = (np.array((36, 279.5)) - start_pos_right) * 0.0254
    rdiv_leg2 = (staging_right - start_pos_right) * 0.0254

    straight_fwd1 = np.array((36, 0)) * 0.0254
    straight_fwd2 = np.array((132, 0)) * 0.0254

    _, trajectories['left'] = pf.generate(
        [
            pf.Waypoint(left_leg1[0], left_leg1[1], 0),
            pf.Waypoint(left_leg2[0], left_leg2[1], 0),
            pf.Waypoint(left_leg3[0], left_leg3[1], 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['divert-left'] = pf.generate(
        [
            pf.Waypoint(ldiv_leg1[0], ldiv_leg1[1], 0),
            pf.Waypoint(ldiv_leg2[0], ldiv_leg2[1], 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['right'] = pf.generate(
        [
            pf.Waypoint(right_leg1[0], right_leg1[1], 0),
            pf.Waypoint(right_leg2[0], right_leg2[1], 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['divert-right'] = pf.generate(
        [
            pf.Waypoint(rdiv_leg1[0], rdiv_leg1[1], 0),
            pf.Waypoint(rdiv_leg2[0], rdiv_leg2[1], 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['straight-forward'] = pf.generate(
        [
            pf.Waypoint(straight_fwd1[0], straight_fwd1[1], 0),
            pf.Waypoint(straight_fwd2[0], straight_fwd2[1], 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    # and then write it out
    with open(trajectory_file, 'wb') as fp:
        pickle.dump(trajectories, fp)
else:
    with open(trajectory_file, 'rb') as fp:
        trajectories = pickle.load(fp)


class Autonomous:
    # maximum auto drive speed, in m/s
    # (we convert 200 ticks/100ms to inches per second to meters per second)
    def __init__(self, robot, robot_position):
        self.robot = robot
        self.position = robot_position

        self.timer = wpilib.Timer()
        self.timer.reset()
        self.timer.start()

        self.start_timer = wpilib.Timer()
        self.start_timer.reset()
        self.start_timer.start()
        self.startup_routine = True
        self.start_timer_started = False

        self.lift_timer = wpilib.Timer()
        self.lift_timer_started = False

        target_trajectory = trajectories['straight-forward']
        self.eject_cube = False

        try:
            self.field_string = ''
            ds = wpilib.DriverStation.getInstance()
            while self.timer.get() < 1 and len(self.field_string) == 0:
                fms_message = ds.getGameSpecificMessage().decode("utf-8")
                if fms_message is None:
                    fms_message = ''

                self.field_string = fms_message.upper()

            if self.field_string != '':
                print("[auto] Got field string in {:.3f} seconds: {}".format(
                    self.timer.get(), self.field_string
                ))

            if robot_position is None:
                robot_position = '[was None]'

            if robot_position.lower() == 'middle':
                if len(self.field_string) == 0:
                    target_trajectory = trajectories['straight-forward']
                    print("[auto] Could not retrieve field string from FMS within timeout!")  # noqa: E501
                elif self.field_string[0] == 'L':
                    print("[auto] Selected trajectory: Left (attempting cube placement)")  # noqa: E501
                    target_trajectory = trajectories['left']
                    self.eject_cube = True
                elif self.field_string[0] == 'R':
                    print("[auto] Selected trajectory: Right (attempting cube placement)")  # noqa: E501
                    target_trajectory = trajectories['right']
                    self.eject_cube = True
                else:
                    target_trajectory = trajectories['straight-forward']
                    print("[auto] Found unexpected data in field string: " + str(self.field_string))  # noqa: E501
            elif robot_position.lower() == 'left':
                target_trajectory = trajectories['divert-left']
                print("[auto] Selected trajectory: Divert Left")
            elif robot_position.lower() == 'right':
                target_trajectory = trajectories['divert-right']
                print("[auto] Selected trajectory: Divert Right")
            else:
                print("[auto] Found unexpected data in robot position string: " + str(robot_position))  # noqa: E501
                target_trajectory = trajectories['straight-forward']
        except:  # noqa: E722
            # Don't re-raise exceptions-- just note it and default to
            # something sane
            print("[auto] Caught exception in auto trajectory decision logic: " + sys.exc_info()[0])  # noqa: E501
            target_trajectory = trajectories['straight-forward']
            self.eject_cube = False

        self.trajectory = pf.modifiers.SwerveModifier(
            target_trajectory
        ).modify(
            constants.chassis_width * 0.0254,  # distance between left and right wheels in m  # noqa: E501
            constants.chassis_length * 0.0254  # distance between front and back wheels in m  # noqa: E501
        )

        # Setup swerve EncoderFollowers
        # yes, the order does matter (must match constants.swerve_config)
        self.followers = [
            EncoderFollower(self.trajectory.getFrontRightTrajectory()),
            EncoderFollower(self.trajectory.getFrontLeftTrajectory()),
            EncoderFollower(self.trajectory.getBackRightTrajectory()),
            EncoderFollower(self.trajectory.getBackLeftTrajectory())
        ]

        self.traj_finished = False

        for i, follower in enumerate(self.followers):
            # in order:
            # current wheel position, ticks/rotation, wheel diameter in m
            module = robot.drivetrain.modules[i]
            follower.configureEncoder(
                module.drive_talon.getQuadraturePosition(),
                int(80 * 6.67), 4 * 0.0254
            )

            # in order:
            # porportional gain (usually from 0.8 - 1.0)
            # integral gain (unused)
            # derivative gain (tweak if tracking is off, default might work)
            # velocity ratio (= 1 / max velocity)
            # acceleration gain (tweak if we need to get to higher/lower speeds faster)  # noqa: E501
            follower.configurePIDVA(
                1.0, 0.0, 0.0, 1 / _max_speed, 0
            )

    def update_smart_dashboard(self):
        pass

    def periodic(self):
        # follow trajectory if need be
        if self.startup_routine:
            if not self.start_timer_started:
                self.start_timer.reset()
                self.start_timer.start()
                self.start_timer_started = True
            else:
                #
                init_time = self.start_timer.get()
                if init_time < 0.75:
                    self.robot.lift.setLiftPower(-0.6)
                    self.robot.drivetrain.set_all_module_angles(0)
                elif init_time < 1:
                    self.robot.lift.setLiftPower(0)
                    self.robot.drivetrain.set_all_module_angles(0)
                    self.robot.drivetrain.set_all_module_speeds(250, True)
                elif init_time < 1.5:
                    self.robot.drivetrain.set_all_module_angles(0)
                    self.robot.drivetrain.set_all_module_speeds(-250, True)
                else:
                    self.robot.drivetrain.set_all_module_angles(0)
                    self.robot.drivetrain.set_all_module_speeds(0, True)
                    #self.start_timer_started = False
                    self.startup_routine = False
        elif (
            not self.traj_finished
            and self.timer.hasPeriodPassed(_trajectory_dt)
        ):
            self.traj_finished = True
            for i, follower in enumerate(self.followers):
                if not follower.isFinished():
                    mod = self.robot.drivetrain.modules[i]

                    output = follower.calculate(
                        mod.drive_talon.getQuadraturePosition()
                    )

                    angle = follower.getHeading()

                    mod.set_drive_percent_out(output)
                    mod.set_steer_angle(angle)

                    self.traj_finished = False
        elif self.traj_finished and self.eject_cube:
            if not self.lift_timer_started:
                self.lift_timer.start()
                self.lift_timer_started = True
            else:
                if self.lift_timer.get() < 3.5:
                    self.robot.lift.setLiftPower(-0.4)
                elif self.lift_timer.get() < 5:
                    self.robot.lift.setLiftPower(0)
                    self.robot.claw.set_power(-0.75)  # eject cube
                else:
                    self.robot.lift.setLiftPower(0)
                    self.robot.claw.set_power(0)
