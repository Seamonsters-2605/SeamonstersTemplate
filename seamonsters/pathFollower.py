import math
import seamonsters as sea

class PathFollower:
    """
    Controls a SuperHolonomicDrive to follow paths on the field.
    """

    NAVX_LAG = 7 # frames
    NAVX_ERROR_CORRECTION = 0.1 # out of 1

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

        self._robotAngleHistory = []

    def setPosition(self, x, y, angle):
        self.robotX = x
        self.robotY = y
        if angle is not None:
            self.robotAngle = angle
            if self.ahrs is not None:
                self._ahrsOrigin = 0
                self._ahrsOrigin = self._getAHRSAngle() - angle
            self._robotAngleHistory.clear()
        self._drivePositionState = None

    def _getAHRSAngle(self):
        return -math.radians(self.ahrs.getAngle()) - self._ahrsOrigin

    def waitForOrientWheelsGenerator(self, magnitude, direction, turn):
        """
        Orient wheels to prepare to drive with the given mag/dir/turn.
        """
        if magnitude == 0 and turn == 0:
            return
        for _ in range(0, 10):
            self.drive.orientWheels(magnitude, direction, turn)
            yield

    def updateRobotPosition(self):
        moveDist, moveDir, moveTurn, self._drivePositionState = \
            self.drive.getRobotPositionOffset(self._drivePositionState, target=True)

        self.robotAngle += moveTurn
        self._robotAngleHistory.append(self.robotAngle)
        # pretty sure this isn't off by 1
        if len(self._robotAngleHistory) >= PathFollower.NAVX_LAG:
            laggedAngle = self._robotAngleHistory.pop(0)
            if self.ahrs is not None:
                navxAngle = self._getAHRSAngle()
                error = (navxAngle - laggedAngle) * PathFollower.NAVX_ERROR_CORRECTION
                self.robotAngle += error
                for i in range(0, len(self._robotAngleHistory)):
                    self._robotAngleHistory[i] += error

        self.robotX += math.cos(moveDir + self.robotAngle) * moveDist
        self.robotY += math.sin(moveDir + self.robotAngle) * moveDist

    def driveToPointGenerator(self, x, y, angle, time,
            robotPositionTolerance=0, robotAngleTolerance=0):
        """
        A generator to drive to a location on the field while simultaneously
        pointing the robot in a new direction. This will attempt to move the
        robot at a velocity so it reaches the target position angle in ``time``
        seconds. This generator never exits, but yields ``True`` or ``False``
        if the robot is close enough to its target position, within tolerance.

        If ``time`` is zero, the robot will attempt to move to the position as
        fast as possible.

        Position mode is recommended!
        """
        dist, moveDir = self._robotVectorToPoint(x, y)
        aDiff = angle - self.robotAngle
        # actual velocities don't matter for orientWheels as long as the ratios
        # are correct
        yield from self.waitForOrientWheelsGenerator(dist, moveDir, aDiff)
        for wheel in self.drive.wheels:
            wheel.resetPosition()

        if dist < 0.1: # TODO: constant
            dist = 0
        if abs(aDiff) < math.radians(1): # TODO
            aDiff = 0
        targetMag = 0
        targetAVel = 0
        if time != 0:
            targetMag = dist / time
            targetAVel = aDiff / time

        accel = 0
        while True:
            accel += 0.1
            if accel > 1:
                accel = 1

            self.updateRobotPosition()

            dist, dir = self._robotVectorToPoint(x, y)
            aDiff = angle - self.robotAngle

            # is the robot close enough to the target position to reach it in
            # the next iteration?
            atPosition = targetMag == 0 or dist < targetMag / sea.ITERATIONS_PER_SECOND
            if atPosition:
                mag = dist * sea.ITERATIONS_PER_SECOND
            else:
                mag = targetMag
            atAngle = targetAVel == 0 or abs(aDiff) < abs(targetAVel / sea.ITERATIONS_PER_SECOND)
            if atAngle:
                aVel = aDiff * sea.ITERATIONS_PER_SECOND
            else:
                aVel = abs(targetAVel)
                if aDiff < 0:
                    aVel = -aVel

            self.drive.drive(mag * accel, dir, aVel * accel)
            yield (atPosition or dist <= robotPositionTolerance) \
                and (atAngle or abs(aDiff) <= robotAngleTolerance)

    # return magnitude, direction
    def _robotVectorToPoint(self, x, y):
        xDiff = x - self.robotX
        yDiff = y - self.robotY
        return (math.sqrt(xDiff ** 2 + yDiff ** 2),
                math.atan2(yDiff, xDiff) - self.robotAngle)

    def _readDataLine(self, line):
        return (float(n) for n in line)

    def followPathData(self, data):
        """
        Follow path data read from a file. ``data`` should be a list of line
        tuples returned by ``sea.readDataFile``.
        """
        lastTime, lastX, lastY, lastAngle = self._readDataLine(data[0])
        self.setPosition(lastX, lastY, math.radians(lastAngle))
        for point in data[1:]:
            t, x, y, angle = self._readDataLine(point)
            if lastX == x and lastY == y and lastAngle == angle:
                yield from sea.wait(int((t - lastTime) * sea.ITERATIONS_PER_SECOND))
            else:
                yield from sea.untilTrue(
                    self.driveToPointGenerator(x, y, math.radians(angle),
                        t - lastTime))
            lastTime = t
            lastX = x
            lastY = y
            lastAngle = angle
