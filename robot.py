import seamonsters as sea 
import wpilib
import rev
import math

class PracticeBot(sea.GeneratorBot):

    def robotInit(self):
        leftSpark = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        rightSpark = rev.CANSparkMax(2, rev.MotorType.kBrushless)

        for spark in [leftSpark, rightSpark]:
            spark.restoreFactoryDefaults()
            spark.setIdleMode(rev.IdleMode.kBrake)

        leftWheel = sea.AngledWheel(leftSpark, -1, 0, math.pi/2, 1, 16)
        rightWheel = sea.AngledWheel(rightSpark, 1, 0, math.pi/2, 1, 16)

        self.drivetrain = sea.SuperHolonomicDrive()
        self.drivetrain.addWheel(leftWheel)
        self.drivetrain.addWheel(rightWheel)

        for wheel in self.drivetrain.wheels:
            wheel.driveMode = rev.ControlType.kVelocity

        sea.setSimulatedDrivetrain(self.drivetrain)

    def autonomous(self):
        # put your code here:
        pass

if __name__ == "__main__":
    wpilib.run(PracticeBot)