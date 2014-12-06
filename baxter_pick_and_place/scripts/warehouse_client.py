#!/usr/bin/env python
import rospy
import std_msgs
import roslib
roslib.load_manifest("baxter_pick_and_place")
from object_recognition_msgs.msg import RecognizedObjectArray

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
        self.ask_user()

    def ask_user(self):
        print "Which object?"
        for i, o in enumerate(self.objects):
            print "%d.)" % (i + 1), o

        response = raw_input()
        try:
            response = int(response)
        except ValueError:
            response = None

            
        if response == None or response == 0 or response > len(self.objects):
            return
        else:
            self.command_publisher.publish(self.objects[response - 1])

        
def main():
    rospy.init_node("warehouse_client")
    WarehouseClient()
    rospy.spin()

    
if __name__=='__main__':
    main()
