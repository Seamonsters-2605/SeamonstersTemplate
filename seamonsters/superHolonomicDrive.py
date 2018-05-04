import math
import ctre

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

    def drive(self, magnitude, direction):
        """
        Spin the wheel. This should be called 50 times per second.
        :param magnitude: speed in feet per second.
        :param direction: radians
        """


class TestWheel(Wheel):

    def __init__(self, name, x, y):
        super().__init__(x, y)
        self.name = name

    def drive(self, magnitude, direction):
        print(self.name, "Mag:", magnitude, "Dir:", math.degrees(direction))


class AngledWheel(Wheel):

    def __init__(self, motor, x, y, angle, encoderCountsPerFoot,
                 maxVoltageVelocity, reverse=False):
        """
        :param motor: a TalonSRX
        :param x: X position in feet
        :param y: Y position in feet
        :param angle: radians, direction of force
        :param encoderCountsPerFoot: number of encoder counts to travel 1 foot
        :param maxVoltageVelocity: velocity at 100% in voltage mode, in feet
        per second
        :param reverse: boolean
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


class SuperHolonomicDrive:

    def __init__(self):
        self.wheels = []

    def addWheel(self, wheel):
        self.wheels.append(wheel)

    def drive(self, magnitude, direction, turn):
        """
        :param magnitude: feet per second
        :param direction: radians
        :param turn: radians per second
        """

        moveX = math.cos(direction) * magnitude
        moveY = math.sin(direction) * magnitude

        for wheel in self.wheels:
            wheelVectorX = moveX - wheel.y * turn
            wheelVectorY = moveY + wheel.x * turn
            wheelMag = math.sqrt(wheelVectorX ** 2.0 + wheelVectorY ** 2.0)
            wheelDir = math.atan2(wheelVectorY, wheelVectorX)
            wheel.drive(wheelMag, wheelDir)


if __name__ == "__main__":
    drive = SuperHolonomicDrive()
    drive.addWheel(TestWheel("Wheel A", 1, 1))
    drive.addWheel(TestWheel("Wheel B", -1, 1))
    drive.addWheel(TestWheel("Wheel C", 1, -1))
    drive.addWheel(TestWheel("Wheel D", -1, -1))

    def testDrive(mag, dir, turn):
        print("Drive mag:", mag, "dir:", math.degrees(dir), "turn:", turn)
        drive.drive(mag, dir, turn)

    testDrive(4, math.radians(90), 0)
    testDrive(-5, math.radians(46), 0)
    testDrive(0, 0, 1)
