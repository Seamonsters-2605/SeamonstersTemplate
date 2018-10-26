import wpilib
import ctre
from pynput.keyboard import Key, Listener
class MyRobot(wpilib.IterativeRobot):
    def robotInit(self):
        self.leftFront = ctre.WPI_TalonSRX(2)
        self.rightFront = ctre.WPI_TalonSRX(1)
        self.leftBack = ctre.WPI_TalonSRX(0)
        self.rightBack = ctre.WPI_TalonSRX(3)

        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
    def teleopPeriodic(self):
        leftSpeed = self.leftJoystick.getY()
        rightSpeed = self.rightJoystick.getY()

        self.leftFront.set(leftSpeed)
        self.leftBack.set(leftSpeed)
        self.rightFront.set(rightSpeed)
        self.rightBack.set(rightSpeed)
   

if __name__=="__main__":
    wpilib.run(MyRobot,physics_enabled=True)
