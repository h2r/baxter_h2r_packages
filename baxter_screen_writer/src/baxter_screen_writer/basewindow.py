from PyQt4.QtCore import QTimer, SIGNAL
from PyQt4.QtGui import QApplication
import signal
import os

batch_mode = False

app = None
"""
Create the QT application.
"""
def makeApp():
    global app
    if app == None:
        #app = QApplication([], False) # command line mode.
        #app = QCoreApplication([]) # also command line mode
        app = QApplication([])
        
    def exitAll():
        os.killpg(os.getpgrp(), signal.SIGINT)

    def handler(sig, arg):
        #app.quit()
        #os.killpg(os.getpgrp(), signal.SIGTERM)
        os.kill(os.getpid(), signal.SIGTERM)
    signal.signal(signal.SIGINT, handler)
    #app.connect(app, SIGNAL("lastWindowClosed()"), exitAll)
    app.timer = QTimer()
    # Make sure python gets a chance to run periodically.
    def noop():
        pass
    app.connect(app.timer, SIGNAL("timeout()"), noop)
    app.timer.start(1000)
  
    return app

