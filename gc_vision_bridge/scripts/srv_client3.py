#!/usr/bin/env python

from __future__ import absolute_import

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
from std_msgs.msg import String
from std_msgs.msg import Header
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.msg import Face
from opencv_apps.msg import Rect
import time

#from .http import client

class gcsrvclient:

    def __init__(self):

        self.bridge = CvBridge()
        # set ros publishers and subscribers
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=2)
        self.result_pub = rospy.Publisher("result_topic", String, queue_size=2)
        self.faces_pub = rospy.Publisher("/face_detection/faces", FaceArrayStamped, queue_size=2)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=2)

        # main image variables
        self.image_msg = False
        self.cv_image = False
        self.current_image = False

        # results variables
        self.image_faces = False
        self.image_objects = False

        # parameters
        self.display_image = True
        self.display_labels = True
        self.update_rate = 0.5

        # control variables
        self.have_image = False
        self.wait_window = True

        # set next image update for now
        self.next_time =  rospy.get_time()

        #connection = http.client.HTTPConnection('www.python.org', 80, timeout=10)
        #print(connection)

        print "init done"

    def get_image(self):

        # convert to ros image to cv image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # set image and control variables
        image_message = self.current_image
        self.have_image = False

        return image_message, cv_image

    def draw_object_label(self, image, f, pos):
        t = "%s : %s"%(f['name'],f['score'])
        x = int(pos[0])
        y = int(pos[1])
        cv2.putText(image, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    
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
                cv2.putText(image, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    def show_detections(self, image, face_detect, object_detect):
        if 'faceAnnotations' in face_detect:
            for f in face_detect['faceAnnotations']:
                try:               
                    box = [(v['x'], v['y']) for v in f['fdBoundingPoly']['vertices']]
                    pts = numpy.array(box, numpy.int32)
                    pts = pts.reshape((-1,1,2))
                    cv2.polylines(image, [pts], True, (0,255,0), thickness=1)
                    # show the face emotion labels
                    self.draw_emos(image, f, box[1])
                except KeyError:
                    # at least one of the x, y keys is missing
                    # because its value is zero
                    # TODO try to recover them to make the rectangle 
                    fakevar = True
                            
        if 'localizedObjectAnnotations' in object_detect:
            for f in object_detect['localizedObjectAnnotations']:
                try:
                    box = [(v['x'] * 640.0, v['y'] * 480.0) for v in f['boundingPoly']['normalizedVertices']]
                    pts = numpy.array(box, numpy.int32)
                    pts = pts.reshape((-1,1,2))
                    cv2.polylines(image, [pts], True, (255,0,0), thickness=1)
                    # show the object labels
                    self.draw_object_label(image, f, box[1])
                except KeyError:
                    # at least one of the x, y keys is missing
                    # because its value is zero
                    # TODO try to recover them to make the rectangle 
                    fakevar = True

    def create_service_request_faces(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['FACE_DETECTION']
        req.max_results = [10]
        return req

    def create_service_request_objects(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['OBJECT_LOCALIZATION']
        req.max_results = [10]
        return req

    def image_callback(self, data):
        # filter image for desired rate
        if rospy.get_time() < self.next_time:
            return
        self.next_time =  rospy.get_time() + self.update_rate
        # set image and control variables
        self.current_image = data
        self.have_image = True
    
    def decode_results(self, response):
        resp = eval(response)
        result = resp['responses'][0]
        return result

    def publish_detected_faces(self, faces):
        face_array_msg = FaceArrayStamped()
        # set message header
        #msg_header = Header()
        #msg_header.stamp = rospy.Time.now()
        #msg_header.frame_id = "usb_cam"
        #msg_header = self.image_msg.header

        if 'faceAnnotations' in faces:
            for f in faces['faceAnnotations']:
                box = [(v['x'], v['y']) for v in f['fdBoundingPoly']['vertices']]

                # set face message
                face_msg = Face()

                face_msg.face.x = box[0][0] + 100
                face_msg.face.y = box[0][1] + 50
                face_msg.face.width = box[1][0] - box[0][0]
                face_msg.face.height = box[2][1] - box[1][1]

                # set eyes in face message
                eyes_msg = Rect()

                eyes_msg.x = box[0][0] + 100
                eyes_msg.y = box[0][1] + 50
                eyes_msg.width = box[1][0] - box[0][0]
                eyes_msg.height = box[2][1] - box[1][1]

                face_msg.eyes.append(eyes_msg)

                # set other face message data 
                face_msg.label = ""
                face_msg.confidence = f['detectionConfidence']

                #print box

                #print box[0][1]

                #print face_msg
                
                face_array_msg.faces.append(face_msg)

        #face_array_msg.header = msg_header
        # we need to use the same header because message filters
        face_array_msg.header = self.image_msg.header
        self.faces_pub.publish(face_array_msg)

    def publish_detection_results(self, response):
        if self.image_faces:
            self.result_pub.publish(json.dumps(self.image_faces, indent=2, sort_keys=True))
            self.publish_detected_faces(self.image_faces)
        if self.image_objects:
            self.result_pub.publish(json.dumps(self.image_objects, indent=2, sort_keys=True))
        if  self.image_faces or self.image_objects:
            #self.image_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(self.image_msg)

    def spin(self):

        if self.have_image:
            # set image for request
            self.image_msg, self.cv_image = self.get_image()

            # request, get & store results about faces
            req = self.create_service_request_faces(self.image_msg)
            GetAnnotationSrv = rospy.ServiceProxy('get_annotation',gc_srvs.RequestAnnotations)
            resp = GetAnnotationSrv(req)
            self.image_faces = self.decode_results(resp.response)

            # request, get & store results about objects
            req = self.create_service_request_objects(self.image_msg)
            GetAnnotationSrv = rospy.ServiceProxy('get_annotation',gc_srvs.RequestAnnotations)
            resp = GetAnnotationSrv(req)
            self.image_objects = self.decode_results(resp.response)

            # merge results about objects and faces with image
            self.show_detections(self.cv_image, self.image_faces, self.image_objects)

            # debug
            #print json.dumps(self.image_faces, indent=2, sort_keys=True)
            #print json.dumps(self.image_objects, indent=2, sort_keys=True)

            # reset output if no detections
            if not 'faceAnnotations' in self.image_faces:
                self.image_faces = False
                print "No faces"
            if not 'localizedObjectAnnotations' in self.image_objects:
                self.image_faces = False
                print "No objects"

            # publish results on topics
            self.publish_detection_results(resp.response)

            # show debug window
            if self.display_image:
                cv2.imshow('modified', self.cv_image)
            
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
