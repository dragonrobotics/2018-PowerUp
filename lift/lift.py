"""
@file lift.py
Code for the RD4B subsystem

This file contains code for the RD4B lift subsystem, including helper functions
to make working with the lift easier from both autonomous and teleop control
modes.  It also has functionality for reading config values from Preferences.

@author Brandon Gong
@version 0.0.1
@date January 19, 2018
 """

from ctre.cantalon import CANTalon
import wpilib
import math

class RD4BLift:

    # Length for one segment of the arm. As of the time this comment
    # was written a segment is 30 inches long.
    ARM_LENGTH = 30

    def __init__(self, left_id, right_id):
        """
        Create a new instance of the RD4B lift.

        Calling this constructor initializes the two drive motors and configures
        their control modes, and also loads the configuration values from
        preferences.

        Args:
            left_id: the ID number of the left talon.
            right_id: the ID number of the right talon.
        """

        # Create new instances of CANTalon for the left and right motors.
        self.left_motor = CANTalon(left_id)
        self.right_motor = CANTalon(right_id)

        # The right motor will follow the left motor, so we will configure
        # the left motor as Position control mode, and have the right motor
        # follow it.
        self.left_motor.changeControlMode(CANTalon.ControlMode.Position)
        self.left_motor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot)
        self.left_motor.setProfile(0)

        self.right_motor.changeControlMode(CANTalon.ControlMode.Follower)
        self.right_motor.set(left_id)

        # finally, load the configuration values.
        self.load_config_values()

    def load_config_values(self):
        """
        Internal helper function for configuration values.

        This function loads/reloads configuration values from Preferences.
        The necessary keys that need to be defined are:

        - "lift potentiometer horizontal angle"
        - "lift potentiometer base angle"
        - "lift limit up"

        This function also precalculates values that are used throughout the code
        and sets soft limits for the motor based on encoder values.
        """

        # get the preferences instance.
        preferences = wpilib.Preferences.getInstance()

        # get the encoder value when the bars are horizontal and form right
        # angles.  This is used for calculations.
        self.HORIZONTAL_ANGLE = preferences.getFloat("lift potentiometer horizontal angle")

        # get the encoder value when the RD4B is in the fully down position.
        self.initial_angle = preferences.getFloat("lift potentiometer base angle")

        # pre-convert the initial angle to radians and take the sin of the
        # angle.
        self.INITIAL_ANGLE_RADIANS_SIN = math.sin(
            (self.initial_angle - self.HORIZONTAL_ANGLE) / 512 * math.pi)

        # get the upper limit of the encoder, or the encoder value when the
        # RD4B is fully extended upward.
        self.LIMIT_UP = preferences.getFloat("lift limit up")

        # set the soft limits to LIMIT_UP and the initial angle. the motors
        # should not go outside of this range.
        left_motor.setForwardSoftLimit(self.LIMIT_UP)
        left_motor.setReverseSoftLimit(self.initial_angle)


    def fully_extend(self):
        """
        Fully extend the RD4B upward.
        This function runs the motor to the LIMIT_UP position.
        """
        self.left_motor.changeControlMode(CANTalon.ControlMode.Position)
        self.left_motor.set(self.LIMIT_UP)

    def fully_retract(self):
        """
        Fully retract the RD4B to the lowest position.
        This function runs the motor to the initial_angle position.
        """
        self.left_motor.changeControlMode(CANTalon.ControlMode.Position)
        self.left_motor.set(self.initial_angle)

    def set_height(self, inches):
        """
        Set the height for the RD4B lift.

        Given a height to move to in inches, this function will calculate the
        angle the motor needs to run to and apply it. The angle is calculated
        by this equation:

            final angle = arcsin(delta height / (2 * arm length) + sin(initial
            angle))

        Args:
            inches: the height in inches to move the RD4B to.
        """
        self.left_motor.changeControlMode(CANTalon.ControlMode.Position)

        # calculate the final angle as per the equation given above.
        final_angle_radians = math.asin(
            inches / (2 * self.ARM_LENGTH) + self.INITIAL_ANGLE_RADIANS_SIN)

        # convert this final angle from radians to native units.
        native_units = ((final_angle_radians * 512 / math.pi)
            + self.HORIZONTAL_ANGLE)

        # set the left motor to run to this position (the right motor will
        # follow it)
        self.left_motor.set(native_units)

    def getHeight(self):
        """
        Get the height of the RD4B.

        This function will get the angle of the RD4B from the encoder and
        convert this to a height given this equation:

            delta height = 2 * arm length * (sin(final angle) - sin(initial
            angle))

        Returns:
            The height of the RD4B at the time this function is called.
        """

        # get the final angle from the encoder, and convert to radians.
        final_angle_radians = ((self.left_motor.get() - self.HORIZONTAL_ANGLE)
            * 512 * math.pi)

        # calculate the height of the RD4B using the final angle and based on
        # the given equation.
        height = 2 * ARM_LENGTH * (
            math.sin(final_angle_radians) - self.INITIAL_ANGLE_RADIANS_SIN)

        return height

    def isMovementFinished(self):
        """
        Check if the RD4B is in motion.

        This function gets the closed loop error, which will be very small if
        the motor is not currently in motion.

        Returns:
            True if the RD4B is not in motion
            False if the RD4B is currently still in motion.
        """
        return self.left_motor.getClosedLoopError() < 10

    def stop(self):
        """
        Stop the RD4B, no matter what position it is in currently.

        This can be used as an emergency stop in the case of an encoder
        breaking, but it can also be used for general purposes if needed.
        """

        # change the control mode to percent vbus and then set the speed to 0.
        self.left_motor.changeControlMode(CANTalon.ControlMode.PercentVBus)
        self.left_motor.set(0)
