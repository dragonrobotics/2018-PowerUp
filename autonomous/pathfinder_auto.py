import os.path
import pickle
import sys
import wpilib
from numpy import pi
import pathfinder as pf
from pathfinder.followers import EncoderFollower
import constants

trajectory_file = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')
trajectories = {
    'left': None,
    'right': None
}

_trajectory_dt = 0.05  # time in seconds between control updates
_max_speed = 200 * 10 * (4 * pi) / (80 * 6.67) * 0.0254

if wpilib.RobotBase.isSimulation():
    # waypoint specification:
    # relative x, y coordinates in meters; exit angle in radians

    _, trajectories['left'] = pf.generate(
        [
            pf.Waypoint(1, -1, 0),
            pf.Waypoint(4, -1, 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['divert-left'] = pf.generate(
        [
            pf.Waypoint(0.5, -2.5, 0),
            pf.Waypoint(4.5, -2.5, 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['right'] = pf.generate(
        [
            pf.Waypoint(1, 1, 0),
            pf.Waypoint(4, 1, 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['divert-right'] = pf.generate(
        [
            pf.Waypoint(0.5, 2.5, 0),
            pf.Waypoint(4.5, 2.5, 0),
        ],
        pf.FIT_HERMITE_CUBIC,
        pf.SAMPLES_HIGH,
        _trajectory_dt, _max_speed, 2.0, 60.0
    )

    _, trajectories['straight-forward'] = pf.generate(
        [
            pf.Waypoint(0.5, 0, 0),
            pf.Waypoint(4.5, 0, 0),
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

        self.lift_timer = wpilib.Timer()
        self.lift_timer_started = False

        target_trajectory = trajectories['straight-forward']
        self.eject_cube = False

        try:
            self.field_string = ''
            ds = wpilib.DriverStation.getInstance()
            while self.timer.get() < 1 and len(self.field_string) == 0:
                fms_message = ds.getGameSpecificMessage()
                if fms_message is None:
                    fms_message = ''

                self.field_string = fms_message.upper()

            if robot_position is None:
                robot_position = 'was none'

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
        except:
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
        if (
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
