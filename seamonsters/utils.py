import sys
import time

def setSimulatedDrivetrain(drivetrain):
    if sys.argv[1] == 'sim':
        import physics
        physics.simulatedDrivetrain = drivetrain

def readDataFile(filename):
    lines = [ ]
    if sys.argv[1] == 'run':
        # running on robot
        filename = "/home/lvuser/py/" + filename
    with open(filename, 'r') as f:
        for line in f.readlines():
            values = line.split()
            if len(values) == 0:
                continue
            lines.append(tuple(values))
    return lines

class TimingMonitor:
    """
    Monitors the rate of the update loop, to see how closely it matches 50Hz.
    Check the ``realTimeRatio`` variable for the ratio of real time to virtual time.
    Greater than 1 means the update loop isn't running as frequently as it should be
    (it takes more than one second to complete a virtual second).
    """

    def __init__(self):
        self.reset()
    
    def reset(self):
        self._count = 0
        self._lastTime = time.time()
        self.realTimeRatio = 1.0
    
    def step(self):
        self._count += 1
        if self._count % 50 == 0:
            t = time.time()
            self.realTimeRatio = (t - self._lastTime)
            self._lastTime = t
    
    def updateGenerator(self):
        while True:
            self.step()
            yield
