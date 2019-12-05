import wpilib
import ctre
import seamonsters as sea
import math

class TestBot(sea.GeneratorBot):

    def robotInit(self):

        self.leftMotors = [ctre.WPI_TalonSRX(0),ctre.WPI_TalonSRX(1)]
        self.rightMotors = [ctre.WPI_TalonSRX(2),ctre.WPI_TalonSRX(3)]

        self.gamepad = wpilib.XboxController(0)

        print("robot code started")

    def teleop(self):
        
        yield from self.mainLoop()

    def mainLoop(self):

        while True:

            leftMag = self.gamepad.getY(0)
            rightMag = self.gamepad.getY(1)

            for motor in range(len(self.leftMotors)):
                self.leftMotors[motor].set(leftMag)
            
            for motor in range(len(self.rightMotors)):
                self.rightMotors[motor].set(rightMag)

            yield

if __name__ == "__main__":
    wpilib.run(TestBot)