from picamera2 import Picamera2, Preview
import time, libcamera

picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"format": 'BGR888', "size": (1536, 864)})
picam2.configure(camera_config)
# picam2.start_preview(Preview.QTGL)
picam2.start()
# time.sleep(5)
picam2.capture_file("test.jpg")
