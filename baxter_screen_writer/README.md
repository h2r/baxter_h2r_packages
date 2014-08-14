baxter_screen_writer is a package to write text to baxter's screen. Currently the color and text can't be changed, but who knows what the future holds!

Usage
---------------

ROS Subscriber method

```
rosrun baxter_screen_wniter writing_listener.py

rostopic pub /robot/screen/text std_msgs/String "hello world"
```

Importing as a library in python
```
from baxter_screen_writer.screen_writer import ScreenWriter

writer = ScreenWriter()
writer.write("hello world")
```