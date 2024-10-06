from roboflow import Roboflow
import os

rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])
project = rf.workspace("tennisballrobot").project("ballbot")
version = project.version(2)
dataset = version.download("yolov5", location="../datasets/ballbot")
