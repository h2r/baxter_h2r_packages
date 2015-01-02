#!/usr/bin/env python

from baxter_interface import CameraController

def main():
    # exposure = 80, gain=50 looks good on black, terrible on beige    
    for camera_name in ('left_hand_camera', 'right_hand_camera'):
        print "setting", camera_name
        camera = CameraController(camera_name)
        camera.resolution = (1280,800)
        camera.exposure = 50
        camera.gain = 30    #left_hand_camera.gain = CameraController.CONTROL_AUTO
        camera.white_balance_red = CameraController.CONTROL_AUTO
        camera.white_balance_green = CameraController.CONTROL_AUTO
        camera.white_balance_blue = CameraController.CONTROL_AUTO
        camera.open()


if __name__ == "__main__":
    main()
