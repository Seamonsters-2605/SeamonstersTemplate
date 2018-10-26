import wpilib
import ctre
import seamonsters as sea

class TestBot(wpilib.IterativeRobot):
    def robotInit(self):
        self.talon = ctre.WPI_TalonSRX(0)

        self.talon.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder,0,0)

        self.aWheel = sea.AngledWheel(self.talon, .75, .75,0,
        encoderCountsPerFoot=31291,
        maxVoltageVelocity=16)

        self.superDrive = sea.SuperHolonomicDrive()
        self.superDrive.addWheel(self.aWheel)
        for wheel in self.superDrive.wheels:
            wheel.driveMode = ctre.ControlMode.PercentOutput

    def teleopPeriodic(self):
        self.superDrive.drive(3,0,0)
        if self.aWheel.encoderWorking is False:
            print('Encoder not working!')


if __name__=="__main__":
    wpilib.run(TestBot)