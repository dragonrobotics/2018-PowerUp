"""
Contains functions for teleop logic.
"""
import numpy as np
import constants


def drive(control_stick, drivetrain):
    """
    Drive the robot directly using a joystick.
    """
    ctrl = np.array([
        control_stick.getRawAxis(constants.fwdAxis),
        control_stick.getRawAxis(constants.strAxis)
    ])

    if constants.fwdInv:
        ctrl[0] *= -1

    if constants.strInv:
        ctrl[1] *= -1

    if abs(np.sqrt(np.sum(ctrl**2))) < 0.1:
        ctrl[0] = 0
        ctrl[1] = 0

    tw = control_stick.getRawAxis(constants.rcwAxis)
    if constants.rcwInv:
        tw *= -1

    if abs(tw) < 0.1:
        tw = 0

    drivetrain.drive(
        ctrl[0],
        ctrl[1],
        tw
    )
