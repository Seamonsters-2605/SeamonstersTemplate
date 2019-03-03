__author__ = "seamonsters"

import traceback
import hal
from wpilib.robotbase import RobotBase
from wpilib import RobotController

class GeneratorBot(RobotBase):
    """
    A robot which runs generators throughout the cycles of autonomous, teleop,
    and test mode. The generators are iterated 50 times per second, synchronized
    with the rate that data is received from Driver Station.
    """

    period = 0.02

    def __init__(self):
        RobotBase.__init__(self)
        self.iterator = None
        self.earlyStop = False

        hal.report(hal.UsageReporting.kResourceType_Framework,
                   hal.UsageReporting.kFramework_Timed)

        self._expirationTime = 0
        self._notifier = hal.initializeNotifier()

    def free(self): # called by python
        hal.stopNotifier(self._notifier)
        hal.cleanNotifier(self._notifier)

    def startCompetition(self):
        self.robotInit()

        # Tell the DS that the robot is ready to be enabled
        hal.observeUserProgramStarting()

        self._expirationTime = RobotController.getFPGATime() * 1e-6 + self.period
        self._updateAlarm()

        while True:
            if hal.waitForNotifierAlarm(self._notifier) == 0:
                if self.iterator is not None:
                    self.iterator.close()
                    self.iterator = None
                print("Robot Break!")
                break

            self._expirationTime += self.period
            self._updateAlarm()

            # Wait for new data to arrive
            self.ds.waitForData()
            if self.isDisabled():
                self.earlyStop = False
                if self.iterator:
                    self.iterator.close()
                    self.iterator = None
                hal.observeUserProgramDisabled()
            else: # not disabled
                if not self.iterator and not self.earlyStop:
                    try:
                        if self.isTest():
                            self.iterator = self.test()
                        elif self.isAutonomous():
                            self.iterator = self.autonomous()
                        else:
                            self.iterator = self.teleop()
                    except:
                        print("Exception while starting sequence!")
                        traceback.print_exc()
                        self.earlyStop = True

                if self.isTest():
                    hal.observeUserProgramTest()
                elif self.isAutonomous():
                    hal.observeUserProgramAutonomous()
                else:
                    hal.observeUserProgramTeleop()

                if self.iterator:
                    try:
                        next(self.iterator)
                    except StopIteration:
                        print("Robot done.")
                        self.iterator = None
                        self.earlyStop = True
                    except:
                        print("Exception in robot code!")
                        traceback.print_exc()
                        self.iterator = None
                        self.earlyStop = True

    def _updateAlarm(self) -> None:
        """Update the alarm hardware to reflect the next alarm."""
        hal.updateNotifierAlarm(self._notifier, int(self._expirationTime * 1e6))

    def robotInit(self):
        """
        Override this for robot initialization. This should NOT be a generator.
        """
        print("No robotInit!")

    def teleop(self):
        """
        Override this to make a generator for teleop
        """
        print("No teleop!")
        yield

    def autonomous(self):
        """
        Override this to make a generator for autonomous
        """
        print("No autonomous!")
        yield

    def test(self):
        """
        Override this to make a generator for test mode
        """
        print("No test!")
        yield


class IterativeRobotInstance:
    """
    Allows an "instance" of an IterativeRobot to be created without connecting
    to HAL. Allows running teleop/autonomous sequences as a Generator.
    """

    def __init__(self, robotType):
        # https://stackoverflow.com/a/19476841
        self.robotObject = robotType.__new__(robotType)
        self.robotType = robotType
        robotType.robotInit(self.robotObject)

    def teleopGenerator(self):
        """
        A generator which runs teleopInit then teleopPeriodic continuously.
        """
        self.robotType.teleopInit(self.robotObject)
        while True:
            yield
            self.robotType.teleopPeriodic(self.robotObject)

    def autonomousGenerator(self):
        """
        A generator which runs autonomousInit then autonomousPeriodic
        continuously.
        """
        self.robotType.autonomousInit(self.robotObject)
        while True:
            yield
            self.robotType.autonomousPeriodic(self.robotObject)
