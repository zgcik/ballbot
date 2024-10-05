import roboflow
import os

rf = roboflow.Roboflow(os.environ["ROBOFLOW_API_KEY"])
project = rf.workspace().project("ballbot")
version = project.version(2)
version.deploy("yolov5", "runs/detect/train15")
