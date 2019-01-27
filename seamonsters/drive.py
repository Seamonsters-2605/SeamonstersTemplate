__author__ = "seamonsters"

import ctre
import math


class AccelerationFilterDrive:
    """
    Wraps another drive interface, and provides acceleration filtering.
    """

    def __init__(self, interface, accelerationRate=.08):
        """
        ``interface`` is the DriveInterface to provide acceleration filtering
        for. ``accelerationRate`` defaults to .08 (0 to full speed in .25
        seconds).
        """
        self.interface = interface

        self.accelerationRate = accelerationRate
        self.previousX = 0.0
        self.previousY = 0.0
        self.previousTurn = 0.0

    def drive(self, magnitude, direction, turn):
        magnitude, direction, turn = \
            self._accelerationFilter(magnitude, direction, turn)
        return self.interface.drive(magnitude, direction, turn)

    def getFilteredMagnitude(self):
        return math.sqrt(self.previousX ** 2 + self.previousY ** 2)

    def getFilteredDirection(self):
        return math.atan2(self.previousY, self.previousX)

    def getFilteredTurn(self):
        return self.previousTurn

    # returns an tuple of: (magnitude, direction, turn)
    def _accelerationFilter(self, magnitude, direction, turn):
        if abs(self.previousTurn - turn) <= self.accelerationRate:
            newTurn = turn
        else:
            if turn > self.previousTurn:
                newTurn = self.previousTurn + self.accelerationRate
            else:
                newTurn = self.previousTurn - self.accelerationRate

        x = magnitude * math.cos(direction)
        y = magnitude * math.sin(direction)
        distanceToNew = math.sqrt((x - self.previousX) ** 2
                                + (y - self.previousY) ** 2)

        if distanceToNew <= self.accelerationRate:
            newX = x
            newY = y
            newMagnitude = magnitude
            newDirection = direction
        else:
            newX = self.previousX \
                + (x - self.previousX) * self.accelerationRate / distanceToNew
            newY = self.previousY \
                + (y - self.previousY) * self.accelerationRate / distanceToNew
            newMagnitude = math.sqrt(newX ** 2 + newY ** 2)
            newDirection = math.atan2(newY, newX)

        self.previousX = newX
        self.previousY = newY
        self.previousTurn = newTurn
        return newMagnitude, newDirection, newTurn


class MultiDrive:
    """
    Wraps another DriveInterface, and allows ``drive()`` to be called multiple
    times in a loop. The values for all of these calls are averaged together,
    and sent to the wrapped interface when ``update()`` is called.
    """

    def __init__(self, interface):
        super().__init__()
        self.interface = interface
        self._reset()

    def _reset(self):
        self.totalX = 0
        self.totalY = 0
        self.totalTurn = 0
        self.numDriveCalls = 0
        self.numTurnCalls = 0

    def drive(self, magnitude, direction, turn):
        self.totalX += magnitude * math.cos(direction)
        self.totalY += magnitude * math.sin(direction)
        self.totalTurn += turn

        if magnitude != 0:
            self.numDriveCalls += 1
        if turn != 0:
            self.numTurnCalls += 1

    def update(self):
        if self.numDriveCalls == 0:
            x = 0
            y = 0
        else:
            x = float(self.totalX) / float(self.numDriveCalls)
            y = float(self.totalY) / float(self.numDriveCalls)
        if self.numTurnCalls == 0:
            turn = 0
        else:
            turn = float(self.totalTurn) / float(self.numTurnCalls)
        magnitude = math.sqrt(x ** 2 + y ** 2)
        direction = math.atan2(y, x)
        scale = self.interface.drive(magnitude, direction, turn)
        self._reset()
        return scale


if __name__ == "__main__":
    # test acceleration filter drive
    filterDrive = AccelerationFilterDrive(TestDriveInterface())
    for i in range(0, 20):
        filterDrive.drive(1.0, 1.0, 1.0)
