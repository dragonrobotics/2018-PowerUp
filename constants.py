"""
Contains constants relating to robot configuration; for example, Talon CAN IDs
and frame dimensions.
"""
import wpilib


# Teleop control constants. Can be loaded from Preferences.
fwdAxis = 1  # Forward/Backward axis
strAxis = 0  # Left/Right axis
rcwAxis = 4  # Rotation axis

fwdInv = True  # Fwd/Bwd axis inverted
strInv = True  # L/R axis inverted
rcwInv = True  # Rot axis inverted


# Wraps the preferences API to provide an alternative to all of the
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
        getMethod = prefs.getInt
        putMethod = prefs.putInt
    elif isinstance(backup, float):
        getMethod = prefs.getFloat
        putMethod = prefs.putFloat

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
    global fwdAxis, fwdInv, strAxis, strInv, rcwAxis, rcwInv

    fwdAxis = int(__load_preference('Control: Forward-Backward Axis', backup=1))  # noqa: E501
    fwdInv = __load_preference('Control: Fwd-Bwd Axis Inverted', backup=True)

    strAxis = int(__load_preference('Control: Left-Right Axis', backup=0))
    strInv = __load_preference('Control: L-R Axis Inverted', backup=True)

    rcwAxis = int(__load_preference('Control: Rotation Axis', backup=4))
    rcwInv = __load_preference('Control: Rot Axis Inverted', backup=True)


# Swerve module hardware configuration.
# List of tuples of form ('module name', steer_id, drive_id)
# See swerve/swerve_drive.py
swerve_config = [
    ('Back Right', 15, 12),
    ('Back Left', 4, 10),
    ('Front Right', 1, 2),
    ('Front Left', 5, 13),
]

# Both are in inches, but exact units don't matter
# (as long as both use the same units)
chassis_length = 32
chassis_width = 28
