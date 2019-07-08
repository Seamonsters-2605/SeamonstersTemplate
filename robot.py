import seamonsters as sea 
import wpilib
import ctre
import math

class PracticeBot(sea.GeneratorBot):

    def robotInit(self):
        leftTalon = ctre.WPI_TalonSRX(0)
        rightTalon = ctre.WPI_TalonSRX(1)

        for talon in [leftTalon, rightTalon]:
            talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        
        leftWheel = sea.AngledWheel(leftTalon, -1, 0, math.pi/2, 31291.1352, 16)
        rightWheel = sea.AngledWheel(rightTalon, 1, 0, math.pi/2, 31291.1352, 16)

        self.drivetrain = sea.SuperHolonomicDrive()
        self.drivetrain.addWheel(leftWheel)
        self.drivetrain.addWheel(rightWheel)

        for wheel in self.drivetrain.wheels:
            wheel.driveMode = ctre.ControlMode.PercentOutput

        sea.setSimulatedDrivetrain(self.drivetrain)

    def autonomous(self):
        turnList = [-1,1,1,-1,-1,1,1]
        for turnDir in turnList:
            self.driveASection()
            self.turn(turnDir)
        yield from self.stop()

    def turn(self, speed):
        self.drivetrain.drive(5, math.pi/2, math.radians(300) * speed)
        yield from sea.wait(93)

    def stop(self):
        yield self.drivetrain.drive(0,0,0)

    def driveASection(self):
        self.drivetrain.drive(16, math.pi/2, 0)
        yield from sea.wait(188)

if __name__ == "__main__":
    wpilib.run(PracticeBot)