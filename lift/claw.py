"""
@file claw.py
Code for the claw subsystem

This file contains code for the claw, which is implemented as a finite-state
automaton with these states:

- ``neutral``: the claw is fully open and not in action.
- ``closing``: the claw is transitioning from the "neutral" state to the
  "closed" state.
- ``closed``: the claw is in the closed position, but still exerting light
  pressure to maintain grip on the cube.
- ``opening``: the claw is transitioning from the "closed" state to the
  "neutral" state.
- ``manual_ctrl``: a disconnected state from the other four; used when the
  driver wants full control over the claw without assistance
  from the state automaton.

@author Brandon Gong, Sebastian Mobo
@version 0.0.1
@date January 19, 2018
 """

import wpilib
from ctre.talonsrx import TalonSRX


class Claw:
    claw_open_time = 0.5  # time to allow for the claw to open, in seconds
    claw_adjust_time = 0.25

    def __init__(self, talon_id):
        """
        Create a new instance of the claw subsystem.

        This function constructs and configures the CANTalon instance and the
        touch sensor. It also sets the state machine to the neutral starting
        state.

        Args:
            talon_id: the ID number of the talon on the claw.
            contact_sensor_channel: the channel that the contact sensor is
                                    connected to.
        """
        self.talon = TalonSRX(talon_id)

        # Forward limit switch = claw opening limit switch
        self.talon.configForwardLimitSwitchSource(
            TalonSRX.LimitSwitchSource.FeedbackConnector,
            TalonSRX.LimitSwitchNormal.NormallyOpen,
            0
        )

        self.state = 'neutral'

        self.closeAdjustTimer = wpilib.Timer()
        self.openTimer = wpilib.Timer()

    def close(self):
        """
        Close the claw.

        This function does a quick check to make sure it is a valid state
        change before actually changing the state to "closing".
        For example you cannot set the state to "closing" if the claw is
        already gripping the cube.
        """
        if self.state != 'closed' and self.state != 'closing':
            self.state = 'closing'

    def open(self):
        """
        Open the claw.

        The idea of checking the state change is similar to that of the close()
        function, but it also resets the opening start time.
        """
        if self.state != 'neutral' and self.state != 'opening':
            self.state = 'opening'
            self.openTimer.reset()
            self.openTimer.start()

    def toggle(self):
        """
        Toggle the claw state.
        If the claw is closed, open it, and if the claw is opened, close it.
        """
        if self.state == 'closed' or self.state == 'closing':
            self.open()
        elif self.state == 'neutral' or self.state == 'opening':
            self.close()

    def update(self):
        """
        The update function is called periodically and progresses the state
        machine.
        """

        # if the state is manual control, stop update immediately and return.
        if self.state == 'manual_ctrl':
            self.openTimer.reset()
            self.openTimer.stop()
            return

        # if the state is neutral, clear the start time and set the talon speed
        # to 0%
        elif self.state == 'neutral':
            self.openTimer.reset()
            self.openTimer.stop()
            self.talon.set(TalonSRX.ControlMode.PercentVbus, 0)

        # if the state is "closing", set the talon speed to -100%, and if the
        # touch sensor is depressed, transition to the "closed" state.
        elif self.state == 'closing':
            self.talon.set(TalonSRX.ControlMode.PercentVbus, -1.0)
            fwdLim, revLim = self.talon.getLimitSwitchState()

            if revLim:  # contact sensor pressed?
                self.state = 'closed'
                self.closeAdjustTimer.reset()
                self.closeAdjustTimer.start()

        # if the state is "closed", use less power to the motors--
        # Maintain grip, but don't squeeze too hard
        elif self.state == 'closed':
            if self.closeAdjustTimer.get() < self.claw_adjust_time:
                self.talon.set(TalonSRX.ControlMode.PercentVbus, -0.5)
            else:
                self.talon.set(TalonSRX.ControlMode.PercentVbus, 0)

        # if the state is "opening", save the current time if the start time
        # is currently empty and set the motor speed to +100%.
        # Run until reaching claw_open_time, then transition to the "neutral"
        # state.
        elif self.state == 'opening':
            self.talon.set(TalonSRX.ControlMode.PercentVbus, 1.0)
            if self.openTimer.get() < self.claw_open_time:
                self.state = 'neutral'
