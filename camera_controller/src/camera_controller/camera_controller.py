#!/usr/bin/env python

import time


from baxter_interface import CameraController

def main():
    print "camera"

    # exposure = 80, gain=50 looks good on black, terrible on beige    

    left_hand_camera = CameraController('left_hand_camera')

    left_hand_camera.resolution = (1280,800)
    left_hand_camera.exposure = 50
    #left_hand_camera.exposure = CameraController.CONTROL_AUTO
    left_hand_camera.gain = 30    #left_hand_camera.gain = CameraController.CONTROL_AUTO
    left_hand_camera.white_balance_red = CameraController.CONTROL_AUTO
    left_hand_camera.white_balance_green = CameraController.CONTROL_AUTO
    left_hand_camera.white_balance_blue = CameraController.CONTROL_AUTO
    left_hand_camera.open()


if __name__ == "__main__":
    main()
