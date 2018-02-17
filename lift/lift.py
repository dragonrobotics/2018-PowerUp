import wpilib
from ctre.talonsrx import TalonSRX


class ManualControlLift:
    def __init__(self, main_lift_id, follower_id):
        self.main_lift_id = main_lift_id
        self.follower_id = follower_id

        self.lift_main = TalonSRX(main_lift_id)
        self.lift_follower = TalonSRX(follower_id)

        self.lift_follower.set(
            TalonSRX.ControlMode.Follower,
            main_lift_id
        )
        self.lift_follower.setInverted(True)

        self.lift_timer = wpilib.Timer()
        self.timer_started = False

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
