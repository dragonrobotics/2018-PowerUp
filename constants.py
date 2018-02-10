"""
Contains constants relating to robot configuration; for example, Talon CAN IDs
and frame dimensions.
"""
import wpilib

# Teleop control constants. Can be loaded from Preferences.
fwdAxis = 1  #: Forward/Backward axis
strAxis = 0  #: Left/Right axis
rcwAxis = 2  #: Rotation axis
liftAxis = 3  #: Lift control axis
fwdInv = True  #: Fwd/Bwd axis inverted
strInv = True  #: L/R axis inverted
rcwInv = True  #: Rot axis inverted
liftInv = True  #: lift axis inverted

teleop_speed = 370
turn_sensitivity = 0.25

lift_height = 96  # inches (== 8 ft)


# Wraps the Preferences API to provide an alternative to all of the
# getInt/getString/getWhatever methods
def __load_preference(key, backup):
    prefs = wpilib.Preferences.getInstance()

    getMethod = None
    putMethod = None

    if isinstance(backup, str):
        getMethod = prefs.getString
        putMethod = prefs.putString
    elif isinstance(backup, bool):
        getMethod = prefs.getBoolean
        putMethod = prefs.putBoolean
    elif isinstance(backup, int):
        getMethod = lambda k, b: int(prefs.getInt(k, b))  # noqa: E731
        putMethod = lambda k, v: prefs.putInt(k, int(v))  # noqa: E731
    elif isinstance(backup, float):
        getMethod = lambda k, b: float(prefs.getFloat(k, b))  # noqa: E731
        putMethod = lambda k, v: prefs.putFloat(k, float(v))  # noqa: E731

    if not prefs.containsKey(key):
        putMethod(key, backup)
        return backup
    else:
        return getMethod(key, backup)


def load_control_config():
    """
    Load configurable constants using the Robot Preferences API.
    Do not call this at module level (otherwise it might try to access parts of
    WPILib before they have been initialized).
    """
    global fwdAxis, fwdInv, strAxis, strInv, rcwAxis, rcwInv, teleop_speed
    global turn_sensitivity, liftAxis, liftInv

    fwdAxis = __load_preference('Control: Forward-Backward Axis', backup=1)
    fwdInv = __load_preference('Control: Fwd-Bwd Axis Inverted', backup=True)

    strAxis = __load_preference('Control: Left-Right Axis', backup=0)
    strInv = __load_preference('Control: L-R Axis Inverted', backup=True)

    rcwAxis = __load_preference('Control: Rotation Axis', backup=2)
    rcwInv = __load_preference('Control: Rot Axis Inverted', backup=True)

    liftAxis = __load_preference('Control: Lift Control Axis', backup=3)
    liftInv = __load_preference('Control: Lift Axis Inverted', backup=True)

    teleop_speed = __load_preference('Control: Teleop Speed', backup=370)
    turn_sensitivity = __load_preference(
        'Control: Turn Sensitivity', backup=0.25
    )


#: Swerve module hardware configuration.
#: List of tuples of form ('module name', steer_id, drive_id)
#: See swerve/swerve_drive.py
swerve_config = [
    ('Back Right', 11, 10),
    ('Back Left', 9, 8),
    ('Front Right', 7, 6),
    ('Front Left', 5, 4),
]

#: Lift motor contorller CAN IDs. Currently dummy values.
lift_ids = {
    'left': 20,
    'right': 21
}

# Claw motor controller CAN ID(s).
claw_id = 22
claw_contact_sensor_channel = 1

#: The length of the chassis (units do not matter as long as they match)
chassis_length = 24

#: The width of the chassis (units do not matter as long as they match)
chassis_width = 27
