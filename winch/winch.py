from ctre.talonsrx import TalonSRX
import wpilib


"""
Author: Chris McKinnon
"""
class Winch:
    def __init__(self, talon_id):
        self.talon = TalonSRX(talon_id)
        self.talon.configSelectedFeedbackSensor(
            TalonSRX.FeedbackDevice.QuadEncoder, 0, 0
        )
        self.talon.setQuadraturePosition(0, 0)
        self.talon.setInverted(True)

    def forward(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput, 0.75)

    def reverse(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput, -0.75)

    def stop(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput, 0)

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putNumber(
            "Winch Position",
            self.talon.getSelectedSensorPosition(0)
        )

        wpilib.SmartDashboard.putNumber(
            "Winch Quad Position",
            self.talon.getQuadraturePosition()
        )
