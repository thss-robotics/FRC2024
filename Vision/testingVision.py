#!/usr/bin/env python3

import json
import time
from cscore import CameraServer, UsbCamera

def startCamera():
    """Start running the camera."""
    cameraName = "Camera 0"
    cameraPath = "/dev/video0"
    print(f"Starting camera '{cameraName}' on {cameraPath}")
    camera = UsbCamera(cameraName, cameraPath)
    server = CameraServer.startAutomaticCapture(camera=camera)

    # Example configuration, adjust as needed
    config = {
        "width": 640,
        "height": 480,
        "fps": 30
    }
    camera.setConfigJson(json.dumps(config))
    camera.setConnectionStrategy(UsbCamera.ConnectionStrategy.kConnectionKeepOpen)

    return camera

if __name__ == "__main__":
   # Start the camera
   startCamera()

   # Start NetworkTables
   # This example assumes you're using this as a client on team number 9999, adjust as necessary
   teamNumber = 9719
   from ntcore import NetworkTableInstance
   ntinst = NetworkTableInstance.getDefault()
   print(f"Setting up NetworkTables client for team {teamNumber}")
   ntinst.startClient4("wpilibpi")
   ntinst.setServerTeam(teamNumber)
   ntinst.startDSClient()

   
   # Loop forever
   while True:
      time.sleep(10)
