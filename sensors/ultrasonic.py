""" I2C sensor interface"""


class Ultrasonic:
    def __init__(self, address):
        """
        one instance per I2C address
        :param address:
        """
        self.address = address

    def read_distance(self):
        pass

    def update_address(self):
        pass
