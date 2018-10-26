
import wpilib
import ctre
from pynput.keyboard import Key, Listener
class MyRobot(wpilib.IterativeRobot):
    keypresses = [0]

    def robotInit(self):
        self.leftFront = ctre.WPI_TalonSRX(2)
        self.rightFront = ctre.WPI_TalonSRX(1)
        self.leftBack = ctre.WPI_TalonSRX(0)
        self.rightBack = ctre.WPI_TalonSRX(3)

        self.talons = [self.leftFront,self.leftBack,
                        self.rightFront,self.rightBack]

        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
    def teleopPeriodic(self):

        leftSpeed = self.leftJoystick.getY()
        rightSpeed = self.rightJoystick.getY()

        self.leftFront.set(leftSpeed)
        self.leftBack.set(leftSpeed)
        self.rightFront.set(rightSpeed)
        self.rightBack.set(rightSpeed)
        
def key_pressed(self):
    key = MyRobot.keypresses[0]
    if key == Key.page_up:
        print('Up pressed')
        for talon in self.talons:
            talon.set(1)
    elif key == Key.page_down:
        print('Down pressed')
        for talon in self.talons:
            talon.set(-1)
    elif key == Key.left:
        pass
    elif key == Key.left:
        pass

if __name__=="__main__":
    
    with Listener(
        on_press=key_pressed) as listener:
        listener.join()
    wpilib.run(MyRobot,physics_enabled=True)

    
