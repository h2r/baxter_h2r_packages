#!/usr/bin/env python

import basewindow
import animator_ui
from PyQt4.QtCore import SIGNAL
from PyQt4.QtGui import QMainWindow
import glob
import cv2
import cv_bridge
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class Animation:
    def __init__(self, directory):
        self.fnames = [fname for fname in glob.glob("%s/*" % directory)]
        self.fnames.sort()
        self.images = [cv2.imread(path) for path in self.fnames]
        self.current_value = 0
        self.current_idx = 0
        self.target = None

        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image,
                                               queue_size=10)
        self.value_subscriber = rospy.Subscriber("/confusion/value", Int32, self.set_value)

        self.target_subscriber = rospy.Subscriber("/confusion/target", Int32, self.set_target)

        self.animation_timer = rospy.Timer(rospy.Duration(1.0/5.0), self.animate)

 

    def set_idx(self, idx):
        self.current_idx = idx;
        self.current_value = int((float(idx) / len(self.images)) * 100)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.image, encoding="bgr8")
        self.image_publisher.publish(msg)
        return self.images[self.current_idx]
        
    def set_value(self, value):
        if isinstance(value, Int32):
            value = value.data
        idx = int((value / 100.0) * len(self.images))
        return self.set_idx(idx)

    def set_target(self, target):
        print "setting target"
        self.target = target


    @property
    def image(self):
        return self.images[self.current_idx]
        
    def animate(self, time):
        if self.target != None:
            if self.target < self.current_value:
                self.set_value(self.current_value + 1)
            elif self.target > self.current_value:
                self.set_value(self.current_value - 1)
            elif self.target == self.current_value:
                self.target = None
            else:
                raise ValueError("No target: " + `self.target`)



class MainWindow(QMainWindow, animator_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.connect(self.confusionValueSlider,
                     SIGNAL("valueChanged(int)"),
                     self.confusionValueChanged) 
        self.connect(self.confusionTargetSlider,
                     SIGNAL("valueChanged(int)"),
                     self.confusionTargetChanged) 
        rospack = rospkg.RosPack()
        path = rospack.get_path('baxter_screen_writer') + "/data/avery_v1"
        self.animation = Animation(path)
        self.value_publisher = rospy.Publisher("/confusion/value", Int32,
                                               queue_size=10)
        self.value_subscriber = rospy.Subscriber("/confusion/value", Int32,
                                                 self.displayConfusionValue)
        self.target_publisher = rospy.Publisher("/confusion/target", Int32,
                                                queue_size=10)

    def displayConfusionValue(self, value):
        value = value.data
        self.confusionValueLabel.setText(str(value))

        self.currentValueSlider.setValue(int(value))

        
    def confusionTargetChanged(self, value):
        self.target_publisher.publish(value)

    def confusionValueChanged(self, value):
        print "publishing", value
        self.value_publisher.publish(value)

def main():
    app = basewindow.makeApp()
    rospy.init_node('animator', anonymous=True)
    wnd = MainWindow()
    wnd.show()


    app.exec_()
if __name__ == "__main__":
    main()
