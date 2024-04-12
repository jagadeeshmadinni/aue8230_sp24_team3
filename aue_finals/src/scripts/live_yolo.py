#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
import cv2
import imagezmq
import time
import onnxruntime
import numpy as np
from PIL import Image
import sys, os

image_hub = imagezmq.ImageHub()
session = onnxruntime.InferenceSession("yolov3-12.onnx")
inname = [input.name for input in session.get_inputs()]
outname = [output.name for output in session.get_outputs()] 


label =["person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
    "pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet","tvmonitor",
    "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"]

# generate different colors for different classes 
COLORS = np.random.uniform(0, 255, size=(len(label), 3))


def letterbox_image(image, size):
    iw, ih = image.size
    w, h = size
    scale = min(w/iw, h/ih)
    nw = int(iw*scale)
    nh = int(ih*scale)

    image = image.resize((nw,nh), Image.BICUBIC)
    new_image = Image.new('RGB', size, (128,128,128))
    new_image.paste(image, ((w-nw)//2, (h-nh)//2))
    return new_image

def preprocess(img):
    model_image_size = (416, 416)
    boxed_image = letterbox_image(img, tuple(reversed(model_image_size)))
    image_data = np.array(boxed_image, dtype='float32')
    image_data /= 255.
    image_data = np.transpose(image_data, [2, 0, 1])
    image_data = np.expand_dims(image_data, 0)
    return image_data, boxed_image

def get_prediction(image_data, image_size):
    input = {
        inname[0]: image_data,
        inname[1]: image_size
    }
    t0 = time.time()
    boxes, scores, indices = session.run(outname, input)
    print("Predict Time: %ss" % (time.time() - t0))
    out_boxes, out_scores, out_classes = [], [], []
    for idx_ in indices:
        out_classes.append(idx_[1])
        out_scores.append(scores[tuple(idx_)])
        idx_1 = (idx_[0], idx_[2])
        out_boxes.append(boxes[idx_1])
    return out_boxes, out_scores, out_classes

# Source for bounding box drawing: https://towardsdatascience.com/yolo-object-detection-with-opencv-and-python-21e50ac599e9
# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):

    obj_label = str(label[class_id])

    color = COLORS[class_id]

    cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

    cv2.putText(img, obj_label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    #rospy.logwarn("Current working directory is",os.getcwd())
    rospy.init_node('traffic_sign_detect', anonymous=True)
    stop_sign_pub = rospy.Publisher('/stop_sign', Bool, queue_size=10)
    bbox_pub = rospy.Publisher('/bbox',Int32MultiArray,queue_size=10)    
    rate = rospy.Rate(5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rpi_name, rpi_image = image_hub.recv_image()
        #cv2.imshow(rpi_name, rpi_image) # 1 window for each RPi
        cv2.waitKey(1)
        color_coverted = cv2.cvtColor(rpi_image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(color_coverted)
        image_hub.send_reply(b'OK')
        image_data,scaled_image = preprocess(image)
        image_size = np.array([image.size[1], image.size[0]], dtype=np.float32).reshape(1, 2)
        out_boxes, out_scores, out_classes = get_prediction(image_data, image_size)
        out_boxes = np.array(out_boxes).tolist()
        out_scores = np.array(out_scores).tolist()
        out_classes = np.array(out_classes).tolist()
        new_image = cv2.cvtColor(rpi_image, cv2.COLOR_RGB2BGR)

        for i, c in reversed(list(enumerate(out_classes))):
            xmin = int(out_boxes[i][0]-0.5*out_boxes[i][2])
            ymin = int(out_boxes[i][1]-0.5*out_boxes[i][3])
            xmax = int(out_boxes[i][0]+0.5*out_boxes[i][2])
            ymax = int(out_boxes[i][1]+0.5*out_boxes[i][3])

            draw_bounding_box(new_image, c, out_scores[i], xmin, ymin,xmax,ymax)

            if(label[c]=='person'):
                stop_sign_pub.publish(True)
                bbox_pub.publish(Int32MultiArray(data=[xmin,ymin,xmax,ymax]))
            else:
                stop_sign_pub.publish(False)

            print("box:", out_boxes[i])
            print("score:", out_scores[i],",", label[c])
        cv2.imshow(rpi_name, new_image)
        rate.sleep()

if __name__ == '__main__':
    main()




