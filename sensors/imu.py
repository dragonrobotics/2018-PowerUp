import math
import wpilib
from robotpy_ext.common_drivers.navx.ahrs import AHRS


class IMU:
    def __init__(self, port, imu_type='navx', interface='spi'):
        self.type = imu_type
        self.iface = interface
        self.prefs = wpilib.Preferences.getInstance()
        self.angle_offset = 0

        if imu_type == 'navx':
            try:
                if interface == 'spi':
                    self.__imu = AHRS.create_spi(port)
                elif interface == 'i2c':
                    self.__imu = AHRS.create_i2c(port)

                self.__imu.reset()
            except Exception as e:
                print("Caught exception while trying to initialize AHRS: "+e.args)  # noqa: E501
                self.__imu = None
                self.type = 'none'
        else:
            raise NotImplementedError('IMU types other than NavX are not supported.')  # noqa: E501

    def is_present(self):
        if self.type == 'none':
            return False
        elif self.type == 'navx':
            return self.__imu.isConnected()

    def set_angle_offset(self, angle):
        self.angle_offset = angle

    def get_robot_heading(self):
        abs_hdg = 0

        if self.type == 'none':
            return 0
        elif self.type == 'navx':
            abs_hdg = self.__imu.getFusedHeading() * (math.pi / 180)

        abs_hdg += self.angle_offset

        if abs_hdg > (2*math.pi):
            abs_hdg -= (2*math.pi)
        elif abs_hdg < 0:
            abs_hdg += (2*math.pi)

        if self.prefs.getBoolean('Reverse Heading Direction', False):
            abs_hdg *= -1

        return abs_hdg

    def get_continuous_heading(self):
        yaw = 0

        if self.type == 'none':
            return 0
        elif self.type == 'navx':
            yaw = self.__imu.getAngle() * (math.pi / 180)

        yaw += self.angle_offset

        if self.prefs.getBoolean('Reverse Heading Direction', False):
            yaw *= -1

        return yaw

    def get_yaw_rate(self):
        if self.type == 'none':
            return 0
        elif self.type == 'navx':
            return self.__imu.getRate() * (math.pi / 180)

    def reset(self):
        if self.type == 'none':
            return
        elif self.type == 'navx':
            self.__imu.reset()

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putBoolean(
            'IMU Present',
            self.is_present()
        )

        wpilib.SmartDashboard.putNumber(
            'Heading',
            self.get_robot_heading()
        )

        wpilib.SmartDashboard.putNumber(
            'Accumulated Yaw',
            self.get_continuous_heading()
        )
