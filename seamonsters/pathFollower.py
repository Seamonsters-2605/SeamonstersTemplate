import math
import seamonsters.generators

class PathFollower:
    """
    Controls a SuperHolonomicDrive to follow paths on the field.
    """

    def __init__(self, drive, ahrs=None):
        """
        :param drive: a SuperHolonomicDrive
        :param x: starting x position of the robot, in feet
        :param y: starting y position of the robot, in feet
        :param angle: starting angle of the robot, radians.
            0 means the robot's local XY coordinates line up with the field XY
            coordinates.
        :param ahrs: an optional AHRS (NavX) instance. If provided, this will
            be used to track the robot's rotation; if not, the rotation will
            be calculated based on the movement of the motors.
        """
        self.drive = drive
        self._drivePositionState = None
        self.ahrs = ahrs
        self._ahrsOrigin = 0
        if ahrs is not None:
            self._ahrsOrigin = self._getAHRSAngle()
        self.robotX = 0
        self.robotY = 0
        self.robotAngle = 0

    def setPosition(self, x, y, angle):
        self.robotX = x
        self.robotY = y
        self.robotAngle = angle
        if self.ahrs is not None:
            self._ahrsOrigin = 0
            self._ahrsOrigin = self._getAHRSAngle() - angle
        self._drivePositionState = None

    def _getAHRSAngle(self):
        return -math.radians(self.ahrs.getAngle()) - self._ahrsOrigin

    def waitForOrientWheelsGenerator(self, magnitude, direction, turn,
            angleTolerance):
        """
        Orient wheels to prepare to drive with the given mag/dir/turn, and wait
        until all wheels are within ``angleTolerance`` radians of their target
        direction.
        """
        if magnitude == 0 and turn == 0:
            return
        done = False
        while not done:
            done = True
            self.drive.orientWheels(magnitude, direction, turn)
            yield
            for wheel in self.drive.wheels:
                if abs(wheel.getTargetDirection() - wheel.getRealDirection()) \
                        > angleTolerance:
                    done = False

    def driveToPointGenerator(self, x, y, angle, time, wheelAngleTolerance):
        """
        A generator to drive to a location on the field while simultaneously
        pointing the robot in a new direction. This will attempt to move the
        robot at a velocity so it reaches the target position angle in ``time``
        seconds. This generator never exits, but yields ``True`` or ``False``
        if the robot is close enough to reach its position/angle in the next
        cycle.

        If ``time`` is zero, the robot will attempt to move to the position as
        fast as possible.

        Position mode is recommended!
        """
        dist, moveDir = self._robotVectorToPoint(x, y)
        aDiff = angle - self.robotAngle
        # actual velocities don't matter for orientWheels as long as the ratios
        # are correct
        yield from self.waitForOrientWheelsGenerator(dist, moveDir, aDiff,
            wheelAngleTolerance)

        if dist < 0.1: # TODO: constant
            dist = 0
        if abs(aDiff) < math.radians(1): # TODO
            aDiff = 0
        targetMag = 0
        targetAVel = 0
        if time != 0:
            targetMag = dist / time
            targetAVel = aDiff / time

        while True:
            moveDist, moveDir, moveTurn, newState = \
                self.drive.getRobotPositionOffset(self._drivePositionState,
                                                  target=True)
            self._drivePositionState = newState
            if self.ahrs is not None:
                self.robotAngle = self._getAHRSAngle()
            else:
                self.robotAngle += moveTurn
            self.robotX += math.cos(moveDir + self.robotAngle) * moveDist
            self.robotY += math.sin(moveDir + self.robotAngle) * moveDist

            dist, moveDir = self._robotVectorToPoint(x, y)
            aDiff = angle - self.robotAngle

            atPosition = targetMag == 0 or dist < targetMag / 50
            if atPosition:
                mag = dist * 50
            else:
                mag = targetMag
            atAngle = targetAVel == 0 or abs(aDiff) < abs(targetAVel / 50)
            if atAngle:
                aVel = aDiff * 50
            else:
                aVel = abs(targetAVel)
                if aDiff < 0:
                    aVel = -aVel

            self.drive.drive(mag, moveDir, aVel)
            try:
                yield atPosition and atAngle
            except GeneratorExit:
                self.drive.drive(0, 0, 0)
                return

    # return magnitude, direction
    def _robotVectorToPoint(self, x, y):
        xDiff = x - self.robotX
        yDiff = y - self.robotY
        return (math.sqrt(xDiff ** 2 + yDiff ** 2),
                math.atan2(yDiff, xDiff) - self.robotAngle)

    def _readDataLine(self, line):
        return (float(n) for n in line)

    def followPathData(self, data, wheelAngleTolerance):
        """
        Follow path data read from a file. ``data`` should be a list of line
        tuples returned by ``sea.readDataFile``.
        """
        lastTime, lastX, lastY, lastAngle = self._readDataLine(data[0])
        self.setPosition(lastX, lastY, math.radians(lastAngle))
        for point in data[1:]:
            t, x, y, angle = self._readDataLine(point)
            if lastX == x and lastY == y and lastAngle == angle:
                yield from seamonsters.generators.wait(int((t - lastTime) * 50))
            else:
                yield from seamonsters.generators.untilTrue(
                    self.driveToPointGenerator(x, y, math.radians(angle),
                        t - lastTime, wheelAngleTolerance))
            lastTime = t
            lastX = x
            lastY = y
            lastAngle = angle
