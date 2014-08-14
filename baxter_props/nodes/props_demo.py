import roslib
roslib.load_manifest("baxter_props")
from baxter_props.baxter_props import BaxterProps

if __name__ == '__main__':
    rospy.init_node("baxter_props")
    props = BaxterProps()
    props.hug(0.4, 10.0)
    props.bump(Point(x=0.9, y=0.7, z=0.25), "left")
    props.five(Point(x=0.5, y=0.7, z=0.5), "left")