import math
import ctre
import seamonsters.drive

# if circle = math.pi*2, returns the smallest angle between two directions
# on a circle
def _circleDistance(a, b, circle):
    diff = b - a
    while diff > circle / 2:
        diff -= circle
    while diff < -circle / 2:
        diff += circle
    return diff

def _iteratePairs(list):
    for i in range(0, len(list) - 1):
        for j in range(i + 1, len(list)):
            yield list[i], list[j]


class Wheel:
    """
    Interface for wheels. A Wheel has a location and can put out force in a
    direction.
    """

    def __init__(self, x, y):
        """
        :param x: X offset from origin in feet
        :param y: Y offset from origin in feet
        """
        self.x = float(x)
        self.y = float(y)

    def limitMagnitude(self, magnitude, direction):
        """
        Return the scaling factor necessary to keep the wheel speed within its
        limits.

        :param magnitude: speed in feet per second
        :param direction: radians. 0 is right, positive counter-clockwise
        :return: 1.0 if wheel speed is within it limits, otherwise a value
        between 0 and 1 to scale the wheel down to its maximum speed.
        """
        return 1.0

    def drive(self, magnitude, direction):
        """
        Spin the wheel. This should be called 50 times per second.

        :param magnitude: speed in feet per second.
        :param direction: radians. 0 is right, positive counter-clockwise
        """

    def getMovementDirection(self):
        """
        :return: the current direction the wheel is moving in radians. This may be based on sensors.
        """
        return 0

    def getMovementMagnitude(self):
        """
        :return: the current velocity of the wheel in feet per second, based on sensors.
        """
        return 0


class CasterWheel(Wheel):
    """
    Doesn't drive a motor, only stores its drive parameters and echoes them
    back for getMovementDirection and getMovementMagnitude.
    """

    def drive(self, magnitude, direction):
        self._storedMagnitude = magnitude
        self._storedDirection = direction

    def getMovementDirection(self):
        return self._storedDirection

    def getMovementMagnitude(self):
        return self._storedMagnitude


class TestWheel(CasterWheel):
    """
    Logs parameters for drive(), for testing.
    """

    def __init__(self, name, x, y):
        """
        :param name: Name to include when logging parameters.
        :param x: X offset from origin in feet
        :param y: Y offset from origin in feet
        """
        super().__init__(x, y)
        self.name = name

    def drive(self, magnitude, direction):
        super().drive(magnitude, direction)
        print(self.name, "Mag:", magnitude, "Dir:", math.degrees(direction))


class AngledWheel(Wheel):
    """
    An AngledWheel is a wheel oriented in a fixed direction, which it can't
    change on its own. It uses a TalonSRX to drive.
    """

    def __init__(self, motor, x, y, angle, encoderCountsPerFoot,
                 maxVoltageVelocity, reverse=False):
        """
        :param motor: a TalonSRX
        :param x: X position in feet
        :param y: Y position in feet
        :param angle: radians, direction of force. 0 is right, positive
            counter-clockwise
        :param encoderCountsPerFoot: number of encoder counts to travel 1 foot
        :param maxVoltageVelocity: velocity at 100% in voltage mode, in feet
            per second
        :param reverse: boolean, optional
        """
        super().__init__(x, y)
        self.motor = motor
        self.angle = angle
        self.encoderCountsPerFoot = encoderCountsPerFoot
        self.maxVoltageVelocity = maxVoltageVelocity
        self.reverse = reverse

        self.driveMode = ctre.ControlMode.PercentOutput

        self._motorState = None
        self._positionTarget = 0
        self._errorCheckCount = 0

    def limitMagnitude(self, magnitude, direction):
        # TODO: check position error in this function instead, and factor it
        # into the scale
        magnitude *= math.cos(direction - self.angle)
        if abs(magnitude) > self.maxVoltageVelocity:
            return self.maxVoltageVelocity / abs(magnitude)
        return 1.0

    def drive(self, magnitude, direction):
        magnitude *= math.cos(direction - self.angle)
        if self.reverse:
            magnitude = -magnitude

        if self.driveMode == ctre.ControlMode.Disabled:
            if self._motorState != self.driveMode:
                self.motor.disable()
                self._motorState = self.driveMode

        elif self.driveMode == ctre.ControlMode.PercentOutput:
            self.motor.set(self.driveMode, magnitude / self.maxVoltageVelocity)
            self._motorState = self.driveMode

        elif self.driveMode == ctre.ControlMode.Velocity:
            encoderCountsPerSecond = magnitude * self.encoderCountsPerFoot
            self.motor.set(self.driveMode, encoderCountsPerSecond / 10.0)
            self._motorState = self.driveMode

        elif self.driveMode == ctre.ControlMode.Position:
            if self._motorState != self.driveMode:
                self._positionTarget = self.motor.getSelectedSensorPosition(0)
                self._motorState = self.driveMode
                self._errorCheckCount = 0

            encoderCountsPerSecond = magnitude * self.encoderCountsPerFoot
            self._positionTarget += encoderCountsPerSecond / 50.0

            self._errorCheckCount += 1
            if self._errorCheckCount % 20 == 0:
                # getSelectedSensorPosition is slow so only check a few times
                # per second
                currentPos = self.motor.getSelectedSensorPosition(0)
                # TODO: this is arbitrary
                maxError = self.maxVoltageVelocity \
                           * self.encoderCountsPerFoot / 2
                if abs(currentPos - self._positionTarget) \
                        > maxError:
                    print("Incremental position error!", currentPos)
                    self._positionTarget = currentPos

            self.motor.set(self.driveMode, self._positionTarget)

    def getMovementDirection(self):
        return self.angle

    def getMovementMagnitude(self):
        sensorVel = self.motor.getSelectedSensorVelocity(0)
        if self.reverse:
            sensorVel = -sensorVel
        return sensorVel * 10.0 / self.encoderCountsPerFoot


class MecanumWheel(AngledWheel):
    """
    An angled Mecanum wheel. The velocity of the wheel is scaled up to
    compensate for the effect of the rollers. For the angle of the wheel, use
    the diagonal direction of force in the diamond pattern.
    """

    SQRT_2 = math.sqrt(2)

    def limitMagnitude(self, magnitude, direction):
        return super().limitMagnitude(magnitude * MecanumWheel.SQRT_2, direction)

    def drive(self, magnitude, direction):
        return super().drive(magnitude * MecanumWheel.SQRT_2, direction)

    def getMovementMagnitude(self):
        return super().getMovementMagnitude() / MecanumWheel.SQRT_2


class SwerveWheel(Wheel):
    """
    A wheel which can rotate. A SwerveWheel drives using an AngledWheel, and
    rotates using a TalonSRX.
    """

    def __init__(self, angledWheel, steerMotor, encoderCountsPerRev,
                 reverseSteerMotor=False):
        """
        ``zeroSteering()`` is called in __init__.

        :param angledWheel: an AngledWheel for driving. Its angle will be
            updated as the swerve wheel rotates. The SwerveWheel will borrow its
            X/Y position when it's initialized.
        :param steerMotor: a TalonSRX for rotating the swerve wheel. It should
            have a feedback sensor set for position mode.
        :param encoderCountsPerRev: number of encoder counts in a full rotation
            of the steer motor
        :param reverseSteerMotor: boolean, optional
        """
        super().__init__(angledWheel.x, angledWheel.y)
        self.angledWheel = angledWheel
        self.steerMotor = steerMotor
        self.encoderCountsPerRev = encoderCountsPerRev
        self.reverseSteerMotor = reverseSteerMotor
        self.zeroSteering()

    def zeroSteering(self):
        """
        Reset the origin (rotation of wheel when facing right) to the current
        position of the steer motor.
        """
        self._steerOrigin = self.steerMotor.getSelectedSensorPosition(0)

    def limitMagnitude(self, magnitude, direction):
        return self.angledWheel.limitMagnitude(magnitude, direction)

    def _getCurrentSteeringAngle(self):
        offset = self.steerMotor.getSelectedSensorPosition(0) \
                 - self._steerOrigin
        if self.reverseSteerMotor:
            offset = -offset
        return offset * 2 * math.pi / self.encoderCountsPerRev

    def _setSteering(self, direction):
        pos = direction * self.encoderCountsPerRev / math.pi / 2
        if self.reverseSteerMotor:
            pos = -pos
        self.steerMotor.set(ctre.ControlMode.Position, pos + self._steerOrigin)

    def drive(self, magnitude, direction):
        #print("Wheel", math.degrees(direction))
        currentAngle = self._getCurrentSteeringAngle()
        if magnitude != 0:
            # steering should never rotate more than 90 degrees from any position
            angleDiff = _circleDistance(currentAngle, direction, math.pi)
            #print("Target", math.degrees(currentAngle + angleDiff))
            #print(math.degrees(currentAngle), math.degrees(currentAngle + angleDiff))
            self._setSteering(currentAngle + angleDiff)

        self.angledWheel.angle = currentAngle
        self.angledWheel.drive(magnitude, direction)

    def getMovementDirection(self):
        return self.angledWheel.getMovementDirection()

    def getMovementMagnitude(self):
        return self.angledWheel.getMovementMagnitude()


class SuperHolonomicDrive(seamonsters.drive.DriveInterface):

    def __init__(self):
        self.wheels = []

    def addWheel(self, wheel):
        """
        Add a wheel to the set of drivetrain wheels.
        :param wheel: a ``Wheel``
        """
        self.wheels.append(wheel)

    def drive(self, magnitude, direction, turn):
        moveX = math.cos(direction) * magnitude
        moveY = math.sin(direction) * magnitude

        wheelMagnitudes = []
        wheelDirections = []
        wheelLimitScales = []

        for wheel in self.wheels:
            wheelVectorX = moveX - wheel.y * turn
            wheelVectorY = moveY + wheel.x * turn
            wheelMag = math.sqrt(wheelVectorX ** 2.0 + wheelVectorY ** 2.0)
            wheelDir = math.atan2(wheelVectorY, wheelVectorX)
            wheelMagnitudes.append(wheelMag)
            wheelDirections.append(wheelDir)
            wheelLimitScales.append(wheel.limitMagnitude(wheelMag, wheelDir))

        minWheelScale = min(wheelLimitScales)
        for i in range(len(self.wheels)):
            self.wheels[i].drive(wheelMagnitudes[i] * minWheelScale,
                                 wheelDirections[i])
        return minWheelScale

    def getRobotMovement(self):
        """
        Get the movement of the robot as a whole, based on wheel sensors.

        :return: (magnitude, direction, turn). Magnitude in feet per second,
        direction in radians, turn in radians per second.
        """
        wheelValues = []
        for wheel in self.wheels:
            wheelMag = wheel.getMovementMagnitude()
            wheelDir = wheel.getMovementDirection()
            dx = wheelMag * math.cos(wheelDir)
            dy = wheelMag * math.sin(wheelDir)
            wheelValues.append((wheel, dx, dy))

        totalX = 0
        totalY = 0
        totalA = 0
        pairCount = 0
        for (wheelA, aDx, aDy), (wheelB, bDx, bDy) in _iteratePairs(wheelValues):
            # calc wheel b relative to wheel a
            relPosX = wheelB.x - wheelA.x
            relPosY = wheelB.y - wheelA.y
            relPosMag = math.sqrt(relPosX ** 2 + relPosY ** 2)
            relPosDir = math.atan2(relPosY, relPosX)
            relVelX = bDx - aDx
            relVelY = bDy - aDy
            relVelDir = math.atan2(relVelY, relVelX)
            relVelMag = math.sqrt(relVelX ** 2 + relVelY ** 2)

            totalX += (aDx + bDx) / 2
            totalY += (aDy + bDy) / 2
            totalA += relVelMag * math.sin(relVelDir - relPosDir) / relPosMag
            pairCount += 1
        totalX /= pairCount
        totalY /= pairCount
        totalA /= pairCount

        return math.sqrt(totalX ** 2 + totalY ** 2), math.atan2(totalY, totalX), totalA


if __name__ == "__main__":
    drive = SuperHolonomicDrive()
    drive.addWheel(TestWheel("Wheel A", 1, 1))
    drive.addWheel(TestWheel("Wheel B", -1, 1))
    drive.addWheel(TestWheel("Wheel C", 1, -1))
    drive.addWheel(TestWheel("Wheel D", -1, -1))

    def testDrive(mag, dir, turn):
        print("Drive Input mag:", mag, "dir:", math.degrees(dir), "turn:", turn)
        drive.drive(mag, dir, turn)
        magOut, dirOut, turnOut = drive.getRobotMovement()
        print("Drive Output mag:", magOut, "dir:", math.degrees(dirOut), "turn:", turnOut)

    testDrive(0, 0, 0)
    testDrive(4, math.radians(90), 0)
    testDrive(-5, math.radians(46), 0)
    testDrive(0, 0, 1)
    testDrive(-5, math.radians(46), 3)
