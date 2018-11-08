import sys
import queue
import threading
import remi
import remi.gui as gui
import socket

DASHBOARD_PORT = 5805

def hBoxWith(*args, **kwargs):
    box = gui.HBox(**kwargs)
    for widget in args:
        box.append(widget)
    return box

def vBoxWith(*args, **kwargs):
    box = gui.VBox(**kwargs)
    for widget in args:
        box.append(widget)
    return box

def startDashboard(robot, dashboardClass):
    robot.app = None

    def appCallback(app):
        print("Dashboard started")
        robot.app = app

    def startDashboardThread(robot, appCallback):
        if sys.argv[1] == 'sim':
            # doesn't work with 127.0.0.1 or localhost on school laptops
            address = socket.gethostbyname(socket.gethostname())
            remi.start(dashboardClass, start_browser=True,
                address=address, port=DASHBOARD_PORT, userdata=(robot, appCallback,))
        elif sys.argv[1] == 'depoly':
            pass
        elif sys.argv[1] == 'run': # run on robot
            remi.start(dashboardClass, start_browser=False,
                address='10.26.5.2', port=DASHBOARD_PORT, userdata=(robot, appCallback,))

    thread = threading.Thread(target=startDashboardThread,
                                args=(robot, appCallback))
    thread.daemon = True
    thread.start()


class Dashboard(remi.App):

    def __init__(self, *args, **kwargs):
        self.eventQueue = queue.Queue()
        super(Dashboard, self).__init__(*args, **kwargs)

    def queuedEvent(self, eventF):
        def queueTheEvent(*args, **kwargs):
            def doTheEvent():
                print("Event:", eventF.__name__)
                eventF(*args, **kwargs)
            self.eventQueue.put(doTheEvent)
        return queueTheEvent

    def clearEvents(self):
        while not self.eventQueue.empty():
            self.eventQueue.get()

    def doEvents(self):
        while not self.eventQueue.empty():
            event = self.eventQueue.get()
            event()
