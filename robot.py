import wpilib
import rev
import seamonsters as sea 
import math
import physics

class PracticeBot(sea.SimulationRobot):

    def robotInit(self):
        self.joystick = wpilib.Joystick(0)

        self.initDrivetrain()
    
    def initDrivetrain(self):
        leftSpark = sea.createSpark(1, rev.MotorType.kBrushless)
        rightSpark = sea.createSpark(2, rev.MotorType.kBrushless)

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

    def teleop(self):
        while True:
            mag = sea.deadZone(self.joystick.getY())
            mag *= 5 
            turn = sea.deadZone(self.joystick.getX())
            turn *= math.radians(300)

            self.drivetrain.drive(mag, math.pi/2, turn)
            
            yield

if __name__ == "__main__":
    wpilib.run(PracticeBot)