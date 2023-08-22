from picamera2 import Picamera2
import torch
import cv2
import time
import math

YOLO_MODEL = '../model/nano_model.pt'
ROBOT_HEIGHT_M = 0.12
FOCALLENGTH_PX = 627.590698 #470.325581 # picamera v3
DEVICE = "cpu"
CONFIDENCE = 0.5

class Perception:
    def __init__(self):
        # camera settings
        self.picam2 = Picamera2()
        camera_config = self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'BGR888', "size": (1536, 864)}))
        self.picam2.configure(camera_config)
        self.picam2.start()

        # prediction settings
        self.model_YOLO = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLO_MODEL, verbose=False)
        self.model_YOLO.conf = CONFIDENCE
        self.model_YOLO.to(DEVICE)

    def sense_relative_positions(self):
        # time.sleep(1)
        # return []
        # self.picam2.capture_file("test.jpg")

        # info to resize perceived image to the same aspect ratio as it was trained on
        width = math.ceil(480 * 1536/864) #360
        height = 480
        dim = (width, height)
        dx = int((width - 640)/2)

        im = self.picam2.capture_array()
        im = cv2.rotate(im, cv2.ROTATE_180)	
        
        # resize frame
        im = cv2.resize(im, dim, interpolation = cv2.INTER_AREA)
        im = im[:, dx:width-dx,:]
        
        # run detection on image frame
        res = self.model_YOLO(im) 
        bounding_boxes = res.xyxy[0][res.xyxy[0][:, 0].sort()[1]]
        
        if bounding_boxes.size(dim=0) == 0:
            
            print("no robots detected.")
            
            # visualize result: plain image
            image_result = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
            
        else:

            # compute distance
            distance = self.dist(bounding_boxes, FOCALLENGTH_PX, ROBOT_HEIGHT_M)
            
            # visualize result: image with detection boxes and distance estimates
            image_result = self.visualize(im, bounding_boxes, distance)

        cv2.imwrite("perception.png", image_result)
        print("Wrote perception.png")

        # TODO PIA: predict relative coordinates, not just distance!
        return []
    

    def dist(self, bounding_boxes, focallength_px, robot_height_m):

        distance = []

        for box in bounding_boxes:
            
            F1 = math.sqrt(FOCALLENGTH_PX**2 + (box[3]-240)**2 + (box[0]-320)**2)
            F2 = math.sqrt(FOCALLENGTH_PX**2 + (box[1]-240)**2 + (box[1]-320)**2)
            b = box[3]/F1 - box[1]/F2
            dist = ROBOT_HEIGHT_M / b
            print(dist)
            
            distance.append(dist)
                
        return distance
	
	
    def visualize(self, im, bounding_boxes, distance):
        
        i = 0
        
        for box in bounding_boxes:

            img_bb = cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            image_bb_label = cv2.rectangle(img_bb, (int(box[0]), int(box[1])-20), (int(box[0]) + 70, int(box[1])), (255,255,255), -1)
            image_bb_label_text = cv2.putText(image_bb_label, f"Dist: {(distance[i]):.3f}", (int(box[0]), int(box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1, cv2.LINE_AA)
            image_result = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
            i = i+1
        
        return image_result
    
if __name__ == '__main__':
    from vis import Visualizer

    v = Visualizer()
    p = Perception()
    while True:
        print(p.sense_relative_positions())
        v.add_image("camera", "perception.png")