import math
import wpilib
from ctre.talonsrx import TalonSRX


class ManualControlLift:
    def __init__(self, main_lift_id, follower_id):
        self.main_lift_id = main_lift_id
        self.follower_id = follower_id

        self.lift_main = TalonSRX(main_lift_id)
        self.lift_follower = TalonSRX(follower_id)

        self.lift_main.configSelectedFeedbackSensor(
            TalonSRX.FeedbackDevice.PulseWidthEncodedPosition,
            0, 0
        )
        self.lift_main.setPulseWidthPosition(0, 0)

        self.lift_follower.configSelectedFeedbackSensor(
            TalonSRX.FeedbackDevice.PulseWidthEncodedPosition,
            0, 0
        )
        self.lift_follower.setPulseWidthPosition(0, 0)

        self.lift_follower.set(
            TalonSRX.ControlMode.Follower,
            main_lift_id
        )
        self.lift_follower.setInverted(True)

        self.lift_timer = wpilib.Timer()
        self.timer_started = False

    def update_smart_dashboard(self):
        horiz = 1048
        bt = -1876

        pos = self.lift_main.getSelectedSensorPosition(0)

        bottom = (bt - horiz) * (math.pi / (3*512))
        cur = (pos - horiz) * (math.pi / (3*512))

        height = (2 * 28) * (math.sin(cur) - math.sin(bottom))

        wpilib.SmartDashboard.putNumber(
            "Lift Position",
            pos
        )

        wpilib.SmartDashboard.putNumber(
            "2nd Lift Position",
            self.lift_follower.getSelectedSensorPosition(0)
        )

        wpilib.SmartDashboard.putNumber(
            "Bottom Angle",
            math.degrees(bottom)
        )

        wpilib.SmartDashboard.putNumber(
            "Current Angle",
            math.degrees(cur)
        )

        wpilib.SmartDashboard.putNumber(
            "Current Height",
            height
        )

    def moveTimed(self, time, power):
        if not self.timer_started:
            self.timer_started = True
            self.lift_timer.reset()
            self.lift_timer.start()
        elif self.lift_timer.get() < time:
            self.lift_main.set(TalonSRX.ControlMode.PercentOutput, power)
        else:
            self.lift_main.set(TalonSRX.ControlMode.PercentOutput, 0)

    def setLiftPower(self, power):
        self.lift_main.set(TalonSRX.ControlMode.PercentOutput, power)
