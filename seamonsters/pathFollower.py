import math

class PathFollower:
    """
    Controls a SuperHolonomicDrive to follow paths on the field.
    """

    def __init__(self, drive, x, y, angle, ahrs=None):
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
        self.robotX = x
        self.robotY = y
        self.robotAngle = angle
        self.ahrs = ahrs
        self._ahrsOrigin = 0
        if ahrs is not None:
            self._ahrsOrigin = self._getAHRSAngle() - angle

    def _getAHRSAngle(self):
        return -math.radians(self.ahrs.getAngle()) - self._ahrsOrigin

    def driveToPointGenerator(self, x, y, angle, time, positionTolerance, angleTolerance):
        """
        A generator to drive to a location on the field while simultaneously
        pointing the robot in a new direction. This will attempt to move the
        robot at a velocity so it reaches the target position angle in ``time``
        seconds. This generator never exits, but yields ``True`` or ``False``
        if the robot is within ``positionTolerance`` and ``angleTolerance`` of
        its target.

        If ``time`` is zero, the robot will attempt to move to the position as
        fast as possible.

        Position mode is recommended!
        """
        distToPoint = math.sqrt((x - self.robotX) ** 2 + (y - self.robotY) ** 2)
        targetMag = 0
        targetAVel = 0
        if time != 0:
            targetMag = distToPoint / time
            targetAVel = (angle - self.robotAngle) / time

        while True:
            moveDist, moveDir, moveTurn, newState = \
                self.drive.getRobotPositionOffset(self._drivePositionState)
            if self.ahrs is not None:
                self.robotAngle = self._getAHRSAngle()
            else:
                self.robotAngle += moveTurn
            self.robotX += math.cos(moveDir + self.robotAngle) * moveDist
            self.robotY += math.sin(moveDir + self.robotAngle) * moveDist
            self._drivePositionState = newState

            xDiff = x - self.robotX
            yDiff = y - self.robotY
            aDiff = angle - self.robotAngle
            dist = math.sqrt(xDiff ** 2 + yDiff ** 2)
            moveDir = math.atan2(yDiff, xDiff) - self.robotAngle

            if targetMag == 0 or dist < targetMag / 50:
                mag = dist * 50
            else:
                mag = targetMag
            if targetAVel == 0 or abs(aDiff) < abs(targetAVel / 50):
                aVel = aDiff * 50
            else:
                aVel = abs(targetAVel)
                if aDiff < 0:
                    aVel = -aVel

            self.drive.drive(mag, moveDir, aVel)
            try:
                yield dist < positionTolerance and aDiff < angleTolerance
            except GeneratorExit:
                self.drive.drive(0, 0, 0)
                return
