#!/usr/bin/env python

from baxter_interface import CameraController
import sys

def main():
    args = sys.argv[1:]
    if len(args) % 2 != 0:
        raise ValueError("Must pass cameras and resolutions." + `sys.argv`)

    for i in range(len(args) / 2):
        camera_name = args[i]
        res = tuple(int(x) for x in args[i + 1].split("x"))
        set_camera(camera_name, res)

    

def set_camera(camera_name, res):
    print "setting", camera_name
    camera = CameraController(camera_name)
    # camera.resolution = (1280,800)
    camera.resolution = res
    camera.exposure = 50
    camera.gain = 30    #left_hand_camera.gain = CameraController.CONTROL_AUTO
    camera.white_balance_red = CameraController.CONTROL_AUTO
    camera.white_balance_green = CameraController.CONTROL_AUTO
    camera.white_balance_blue = CameraController.CONTROL_AUTO
    camera.open()
    print "done", camera_name

if __name__ == "__main__":

    print "Usage: ", "rosrun camera_controller camera_controller.py  right_hand_camera 1280x800"

    main()
