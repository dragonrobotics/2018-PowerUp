import json
import redis
from vision.visionconstants import COMMAND_QUEUE_KEY, LOCATION_KEY


class VisionSlave:
    """
    Used on jetson, inside vision loop check for new commands and always
    post location for last thing requested
    """

    def __init__(self, host, port):
        """
        establish connection
        :param host: redis host
        :param port: redis port
        """
        self.host = host
        self.port = port
        self.r = redis.Redis(host, port)

    def pop_command(self):
        """ check for a command on the queue, returns None if empty queue """
        try:
            serial_json = self.r.rpop(COMMAND_QUEUE_KEY).decode()
            return json.loads(serial_json)
        except AttributeError:
            return None

    def update_location(self, azimuth, altitude):
        """
        send back the degrees relative to camera + or - from 0 degrees
        :param azimuth:
        :param altitude:
        :return:
        """
        location = {'azimuth': azimuth, 'altitude': altitude}
        serial_json = json.dumps(location)
        return self.r.set(LOCATION_KEY, serial_json)
