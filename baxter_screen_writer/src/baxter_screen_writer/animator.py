#!/usr/bin/env python

import basewindow
import animator_ui
from PyQt4.QtCore import SIGNAL
from PyQt4.QtGui import QMainWindow
import glob
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image

class Animation:
    def __init__(self, directory):
        self.fnames = [fname for fname in glob.glob("%s/*" % directory)]
        self.fnames.sort()
        self.images = [cv2.imread(path) for path in self.fnames]

    def get_image(self, value):
        idx = int(value / 100.0 * len(self.images))
        return self.images[idx]


class MainWindow(QMainWindow, animator_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.connect(self.pictureSlider,
                     SIGNAL("valueChanged(int)"),
                     self.valueChanged) 

        self.animation = Animation("data/avery_v1")

        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image)
        
    def valueChanged(self, value):
        image = self.animation.get_image(value)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        self.image_publisher.publish(msg)

def main():
    app = basewindow.makeApp()
    wnd = MainWindow()
    wnd.show()
    rospy.init_node('animator', anonymous=True)

    app.exec_()
if __name__ == "__main__":
    main()
