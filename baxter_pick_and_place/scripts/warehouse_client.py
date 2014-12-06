#!/usr/bin/env python
import rospy
import time
import std_msgs
import roslib
roslib.load_manifest("baxter_pick_and_place")
from object_recognition_msgs.msg import RecognizedObjectArray
import baxter_external_devices

class WarehouseClient:
    def __init__(self):
        self.command_publisher = rospy.Publisher("/fetch_commands", 
                                                 std_msgs.msg.String, queue_size=10)
        self.objects = []
        topic = "/publish_detections_center/blue_labeled_objects" # node
        #topic = "/ar_objects" # ar tags
        rospy.Subscriber(topic, RecognizedObjectArray, self.object_callback)

             
    def object_callback(self, msg):
        self.objects = []
        self.object_poses = dict()
        for o in msg.objects:
            self.objects.append(o.type.key)

    def ask(self):
        done = False
        last_print = time.time() - 9.5
        while not done and not rospy.is_shutdown():
            if (time.time() - last_print) > 10:
                print "Which object?"
                for i, o in enumerate(self.objects):
                    print "%d.)" % (i + 1), o
                last_print = time.time()

            c = baxter_external_devices.getch()
            if c:
                if c in ['\x1b', '\x03']:
                    done = True
                else:
                    try:
                        idx = int(c)
                    except ValueError:
                        idx = 0
                    if idx != None and idx > 0 and idx <= len(self.objects):
                        print "Sending", self.objects[idx - 1]
                        self.command_publisher.publish(self.objects[idx - 1])
def main():
    rospy.init_node("warehouse_client")

    client = WarehouseClient()

    client.ask()


    
if __name__=='__main__':
    main()
