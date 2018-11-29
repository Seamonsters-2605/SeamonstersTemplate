import math

class PathFollower:

    def __init__(self, drive, x, y, angle, ahrs):
        self.drive = drive
        self._drivePositionState = None
        self.robotX = x
        self.robotY = y
        self.robotAngle = angle
        self.ahrs = ahrs

    def driveToPointGenerator(self, x, y, angle, time):
        distToPoint = math.sqrt((x - self.robotX) ** 2 + (y - self.robotY) ** 2)
        targetMag = distToPoint / time
        targetAVel = (angle - self.robotAngle) / time

        while True:
            moveDist, moveDir, moveTurn, newState = \
                self.drive.getRobotPositionOffset(self._drivePositionState)
            if self.ahrs is not None:
                pass
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

            if dist < targetMag / 50:
                mag = dist * 50
            else:
                mag = targetMag
            if abs(aDiff) < abs(targetAVel / 50):
                aVel = aDiff * 50
            else:
                aVel = abs(targetAVel)
                if aDiff < 0:
                    aVel = -aVel

            self.drive.drive(mag, moveDir, aVel)
            yield dist, aDiff
