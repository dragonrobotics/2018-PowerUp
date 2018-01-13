import json
import redis
from vision.visionconstants import COMMAND_QUEUE_KEY, LOCATION_KEY


class VisionMaster:
    """
    Runs on the RoboRIO; Tells the Jetson what to look for and
    reads the location response
    """

    def __init__(self, host, port):
        """
        create redis instance
        :param host: redis host port on jetson
        :param port: redis port
        """
        self.host = host
        self.port = port
        self.r = redis.Redis(host, port)

    def locate(self, thing):
        """
        tell jetson what you want to look for,
        check visionconstants module for list of things
        :param thing: what to look for
        :return:
        """
        command = {'locate': thing}
        serial_json = json.dumps(command)
        return self.r.lpush(COMMAND_QUEUE_KEY, serial_json)

    def clear_commands(self):
        """
        wipe out the queue if you need to
        :return:
        """
        return self.r.delete(COMMAND_QUEUE_KEY)

    def get_location(self):
        """
        read the location state, as long as you've told jetson
        to look for a valid thing, there should be a value
        :return: dict containing azimuth and altitude
        """
        try:
            serial_json = self.r.get(LOCATION_KEY).decode()
            return json.loads(serial_json)
        except AttributeError:
            return None
