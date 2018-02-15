import wpilib
from ctre.talonsrx import TalonSRX

class Winch:

    def __init__(self, talon_id):
        self.talon = TalonSRX(talon_id)


    def forward(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput,.5)#NEED TO FINND VALUE


    def reverse(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput,-.5)#NEED TO FINND VALUE


    def stop(self):
        self.talon.set(TalonSRX.ControlMode.PercentOutput,0)#NEED TO FINND VALUE
