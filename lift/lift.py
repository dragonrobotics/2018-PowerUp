import wpilib
from ctre.talonsrx import TalonSRX


class ManualControlLift:
    def __init__(
        self,
        main_lift_id, follower_id,
        bottom_limit_channel, start_lim_channel
    ):
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

        self.lift_zero_found = False
        self.bottom_limit_switch = wpilib.DigitalInput(bottom_limit_channel)
        self.start_limit_switch = wpilib.DigitalInput(start_lim_channel)

    def load_config_values(self):
        prefs = wpilib.Preferences.getInstance()

        phase = prefs.getBoolean("Lift: Invert Sensor Phase", True)
        self.upper_limit = prefs.getInt("Lift: Upper Limit", None)

        # Note: positive / forward power to the motors = lift moves down
        # negative / reverse power to the motors = lift moves up
        if self.upper_limit is not None:
            self.lift_main.configReverseSoftLimitThreshold(
                int(self.upper_limit), 0
            )
            self.lift_main.configReverseSoftLimitEnable(True, 0)
        else:
            self.lift_main.configReverseSoftLimitEnable(False, 0)

        self.lift_main.setSensorPhase(phase)
        self.lift_follower.setSensorPhase(phase)

    def set_soft_limit_status(self, status):
        if self.upper_limit is not None:
            self.lift_main.configReverseSoftLimitEnable(status, 0)
        else:
            self.lift_main.configReverseSoftLimitEnable(False, 0)

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putNumber(
            "Lift Main Position",
            self.lift_main.getSelectedSensorPosition(0)
        )

        wpilib.SmartDashboard.putNumber(
            "Lift Follower Position",
            self.lift_follower.getSelectedSensorPosition(0)
        )

        wpilib.SmartDashboard.putBoolean(
            "Lift Bottom Limit Switch",
            not self.bottom_limit_switch.get()
        )

        wpilib.SmartDashboard.putBoolean(
            "Lift Start Position Switch",
            not self.start_limit_switch.get()
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

    def checkLimitSwitch(self):
        self.set_soft_limit_status(self.lift_zero_found)

        if not self.bottom_limit_switch.get():
            self.lift_zero_found = True

            self.lift_main.setPulseWidthPosition(0, 0)
            self.lift_follower.setPulseWidthPosition(0, 0)

            self.lift_main.setQuadraturePosition(0, 0)
            self.lift_follower.setQuadraturePosition(0, 0)

            if self.lift_main.getMotorOutputPercent() < 0:
                self.lift_main.set(TalonSRX.ControlMode.PercentOutput, 0)

    def driveToStartingPosition(self):
        if not self.start_limit_switch.get():
            self.lift_main.set(TalonSRX.ControlMode.PercentOutput, 0.25)
        else:
            self.lift_main.set(TalonSRX.ControlMode.PercentOutput, 0)
