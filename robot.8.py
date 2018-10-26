import seamonsters as sea
import wpilib
import ctre
from robotpy_ext.common_drivers.navx import ahrs, AHRS
#from wpilib.command import CommandGroup, Scheduler

#from basic_commands import *  # ADDED


class Robot(sea.GeneratorBot):
    def robotInit(self):
        self.leftFront = ctre.CANTalon(2)
        self.leftBack = ctre.CANTalon(0)
        self.rightFront = ctre.CANTalon(1)
        self.rightBack = ctre.CANTalon(3)
        self.ahrs = ahrs


    def teleop(self):
        self.leftFront.set(-1)
        self.rightFront.set(1)
        self.leftBack.set(-1)
        self.rightBack.set(1)

        for i in range(100):
            yield
            self.leftFront.set(-1)
            self.leftBack.set(-1)
            self.rightFront.set(1)
            self.rightBack.set(1)
        for i in range(60):
            yield
            self.leftFront.set(0)
            self.leftBack.set(0)
            self.rightFront.set(1)
            self.rightBack.set(1)

        for i in range(100):
            yield
            self.leftFront.set(-1)
            self.rightFront.set(1)
            self.leftBack.set(-1)
            self.rightBack.set(1)

        for i in range(47):
            yield
            self.leftFront.set(0)
            self.leftBack.set(0)
            self.rightFront.set(1)
            self.rightBack.set(1)

        for i in range(85):
            yield
            self.leftFront.set(-1)
            self.rightFront.set(1)
            self.leftBack.set(-1)
            self.rightBack.set(1)

        for i in range(100):
            yield
            self.leftFront.set(0)
            self.rightFront.set(0)
            self.leftBack.set(0)
            self.rightBack.set(0)







if __name__ == "__main__":
    wpilib.run(Robot, physics_enabled=True)
