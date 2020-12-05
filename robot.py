import math
import rev
import wpilib
import seamonsters as sea
import dashboard

class PracticeBot(sea.GeneratorBot):

    def robotInit(self):
        #set up drivetrain
        self.drivetrain = self.initDrivetrain()
        sea.setSimulatedDrivetrain(self.drivetrain)

        #set up dashboard
        self.app = None
        sea.startDashboard(self, dashboard.PracticeDashboard)

    def teleop(self):
        yield from self.updateDashboardGenerator()

    #creates and returns the drivetrain
    def initDrivetrain(self):
        drivetrain = sea.SuperHolonomicDrive()

        leftSpark = sea.createSpark(1, rev.MotorType.kBrushless)
        rightSpark = sea.createSpark(2, rev.MotorType.kBrushless)

        for spark in [leftSpark, rightSpark]:
            spark.restoreFactoryDefaults()
            spark.setIdleMode(rev.IdleMode.kBrake)

        leftWheel = sea.AngledWheel(leftSpark, -1, 0, math.pi/2, 1, 16)
        rightWheel = sea.AngledWheel(rightSpark, 1, 0, math.pi/2, 1, 16)

        drivetrain.addWheel(leftWheel)
        drivetrain.addWheel(rightWheel)

        for wheel in drivetrain.wheels:
            wheel.driveMode = rev.ControlType.kVelocity

        return drivetrain

    #does the events called by the dashboard
    def updateDashboardGenerator(self):
        if self.app is not None:
            self.app.clearEvents()
        while True:
            v = None
            if self.app is not None:
                v = self.app.doEvents()
            yield v
    
    #dashboard callbacks
    @sea.queuedDashboardEvent
    def c_driveForward(self, button):
        self.drivetrain.drive(4, math.pi/2, 0)

    @sea.queuedDashboardEvent
    def c_stop(self, button):
        self.drivetrain.drive(0,0,0)

if __name__ == "__main__":
    wpilib.run(PracticeBot)