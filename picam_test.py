from picamera2 import Picamera2, Preview
from pprint import pprint

# initialise camera
cam = Picamera2()
mode = cam.sensor_modes[2]
config = cam.create_still_configuration(sensor={"output_size": mode["size"]})
cam.configure(config)  # type: ignore
cam.start()
# get numpy array HxWhC
arr = cam.capture_array("main")
print(arr.shape)  # type: ignore
# save to file
cam.capture_file("test.jpg")
