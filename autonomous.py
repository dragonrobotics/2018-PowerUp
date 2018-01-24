""" collection of functions used to manage autonomous control """
# from vision.visionmaster import VisionMaster
import wpilib


class Autonomous:
    """ Instantiate this from autonomousInit or robotInit. """
    UNKNOWN = 0
    LEFT = 1
    MIDDLE = 2
    RIGHT = 3

    def __init__(self, robot_location):
        self.robot_location = robot_location  # set on SmartDashboard
        # self.vision = VisionMaster('0.0.0.0', 2)

        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()

        self.close_switch = field_string[0]
        self.scale = field_string[1]
        self.far_switch = field_string[2]

        self.target = None

        if self.robot_location == 'Middle':
            if self.close_switch == 'L':
                pass  # We are in middle and switch is on left.
            else:
                pass  # We are in middle and switch is on right.
        elif self.robot_location == 'Left':
            if self.close_switch == 'L':
                pass # We are on left and switch is on left.
            else:
                pass # We are on left and switch is on right.
        elif self.robot_location == 'Right':
            if self.close_switch == 'L':
                pass # We are on right and switch is on left.
            else:
                pass # We are on right and switch is on right.

    def target(self):
        """
        call from autonomous init in robotpy
        give all location information decide where to go
        :return:
        """
        self.target = None

    def periodic(self):
        """
        this is called by robotpy auto periodic
        decide
        """
        pass
