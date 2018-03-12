# Python Swerve Controller

## Summary

This folder contains code for controlling a four-module swerve drive.

## Structure

The code is split between two files:
 * `swerve_module.py`, which contains control routines and logic that are
 common to each individual module; for example, the logic that drives modules
 to target steering angles is contained within this file.
 * `swerve_drive.py`, which contains logic for controlling the drive as a single
 coherent unit; for example, the main `:drive()` method used during Teleop is
 contained within this file.


## Theory of Operation

Individual `SwerveModule` instances support two major operations:
 * Set module drive speed
 * Set module steering

The first operation (setting drive speeds) is relatively simple in concept:
the commanded drive speeds are, for the most part, simply passed on to the drive
motor controller unchanged; however, in certain cases the commanded speeds may
be corrected to ensure that positive speed values always correspond to forward
robot motion, relative to the direction the swerve module has been steered towards.

The second operation (setting module steering) is somewhat more complicated.

Given a target angle in the range `[-2pi, 2pi]` radians,  the operation can be
broken down into four parts:
 * Transforming the discontinuous steering angle parameter into the continuous
   rotation angle space of the swerve module
 * Transforming the raw module steering sensor value into the same continuous
   rotation angle space (in radians).
 * Determining the closest equivalent angle to the target steering angle
 * Converting the final steering target angle to a target steering sensor value
   to send to the steering motor controller.

The first part of this operation is simply accomplished by the cumulative number
of rotations the module has undergone since system start-up and adding
`2 * pi * rotation_count` radians to the given steering angle.

The second part of this operation requires the steering sensor value for a
known reference angle to be known; in our case, this reference value is measured
with the module pointing in the "forward" direction.

This known calibration offset is subtracted from the current steering sensor
value, to give a corrected steering sensor value: a value of 0 corresponds to
directly forward, and increasing values correspond to clockwise rotation.

Furthermore, the sensor values must be converted from the sensor's native units
to radians: this can be done by simply multiplying by a conversion factor of
`(pi radians) / (512 sensor units)`.

After normalizing and converting both the desired steering angle and the
module's current rotation to a common reference and common units, a list of
angles that are equivalent and complementary to the desired steering angle is
created. The angle in the list that is closest to the current module rotation
angle is selected to be the _final steering target angle_.

Note that both equivalent and complementary angles are considered: in certain
cases, the module may choose to steer to an angle "opposite" that of the desired
steering angle, and simply reverse future drive commands. This enables, for example,
quickly driving forwards and then backwards without any module rotations.

After determining the final steering target angle, it is transformed into a
target steering rotation sensor value, and this value is sent to the steering
motor controller's internal position control loop.
