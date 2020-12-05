import remi.gui as gui
import seamonsters as sea

class PracticeDashboard(sea.Dashboard):

    def main(self, robot, appCallback):

        root = gui.VBox(gui.Label("Drive Controls"), width = 600, margin = "0px auto")  

        driveBox = gui.VBox()
        root.append(driveBox)

        driveForwardButton = gui.Button("Drive Forward")
        driveForwardButton.set_on_click_listener(robot.c_driveForward)
        driveBox.append(driveForwardButton)

        stopButton = gui.Button("Stop")
        stopButton.set_on_click_listener(robot.c_stop)
        driveBox.append(stopButton)

        appCallback(self)
        return root