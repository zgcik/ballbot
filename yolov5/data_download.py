from roboflow import Roboflow
import os

rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])
project = rf.workspace("ball-detection-g592z").project("ball-and-box-detetction")
version = project.version(4)
dataset = version.download(
    "yolov5", location="../datasets/ball-and-box-detetction-4-yolov5"
)
