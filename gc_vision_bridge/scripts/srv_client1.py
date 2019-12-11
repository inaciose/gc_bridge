#!/usr/bin/env python

import json
import sys
import os
import cv2
import rospy
import gc_vision_bridge
import gc_msgs.srv as gc_srvs
import sensor_msgs.msg as sensor_msgs
import numpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

class gcsrvclient:

    def __init__(self):

        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCallback, queue_size=2)

        self.img_msg = 0
        self.cv_image = 0
        self.current_image = 0
        self.have_image = False
        self.wait_window = True

        self.next_time =  rospy.get_time()

        print "init done"

    def get_image(self):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_message = self.current_image

        self.have_image = False

        return image_message, cv_image

    def get_emos(self, f):
        emos = {k:v for k, v in f.items() if k.endswith('Likelihood')}
        emos = {k:v for k, v in emos.items() if v != 'VERY_UNLIKELY'}
        return emos

    def draw_emos(self, image, f, pos):
        emos = self.get_emos(f)
        if emos:
            for k,v in emos.items():
                t = "%s : %s"%(k,v)
                x = pos[0]
                y = pos[1]
                cv2.putText(image, t, (x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    def show_detection(self, image, response):
        resp = eval(response) # convert json string to dict
        result = resp['responses'][0]

        if not 'faceAnnotations' in result:
            print "No Face"
            return

        for f in result['faceAnnotations']:
            box = [(v['x'], v['y']) for v in f['fdBoundingPoly']['vertices']]
            pts = numpy.array(box, numpy.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(image, [pts], True, (0,255,0), thickness=2)

            self.draw_emos(image, f, box[1])

    def print_labels(self, response):
        resp = eval(response)
        result = resp['responses'][0]
        print json.dumps(result, indent=2, sort_keys=True)

    def create_service_request(self, img):
        print "create service request"

        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['FACE_DETECTION']
        req.max_results = [10]
        return req

    def imageCallback(self, data):
        #print "img received"
        if rospy.get_time() < self.next_time:
            return
        self.next_time =  rospy.get_time() + 1.0
        self.current_image = data
        self.have_image = True
    
    def spin(self):
        #print "spin"

        if self.have_image:

            self.img_msg, self.cv_image = self.get_image()

            req = self.create_service_request(self.img_msg)
            GetAnnotationSrv = rospy.ServiceProxy('get_annotation',gc_srvs.RequestAnnotations)

            resp = GetAnnotationSrv(req)
            self.print_labels(resp.response)
            self.show_detection(self.cv_image, resp.response)

            cv2.imshow('modified', self.cv_image)
            #time.sleep(1)
            if self.wait_window:
                cv2.waitKey(2000)
            else:
                cv2.waitKey(1)

def main(args):
    rospy.init_node('gc_bridge_client')
    gc = gcsrvclient()
    kb_int = False
    
    while not kb_int:
        try: 
            gc.spin()
            time.sleep(0.1)
        except KeyboardInterrupt:
            kb_int = True
    
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
