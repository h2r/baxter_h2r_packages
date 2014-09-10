rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
rosrun baxter_tools enable_robot.py -e
rosrun image_view image_view image:=/camera/rgb/image_color &
rosrun image_view image_view image:=/cameras/left_hand_camera/image &