""" collection of functions used to manage autonomous control """
from vision.visionmaster import VisionMaster
import wpilib


class Autonomous:
    """ Instantiate this from autonomousInit or robotInit. """
    UNKNOWN = 0
    LEFT = 1
    MIDDLE = 2
    RIGHT = 3

    def __init__(self, robot_location, field_string):
        self.robot_location = robot_location  # TODO: get this from dashboard
        self.field_string = field_string
        self.vision = VisionMaster('0.0.0.0', 2)  # TODO: put addr in constants

        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()

        self.close_switch = field_string[0]
        self.scale = field_string[1]
        self.far_switch = field_string[2]

        self.target = None

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
