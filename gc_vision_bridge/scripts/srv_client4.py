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

import requests

class gcsrvclient:

    def __init__(self):

        self.bridge = CvBridge()
        # set ros publishers and subscribers
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=2)
        self.result_pub = rospy.Publisher("result_topic", String, queue_size=2)
        self.faces_pub = rospy.Publisher("/face_detection/faces", FaceArrayStamped, queue_size=2)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=2)

        self.debug_speed = False

        # main image variables
        self.image_msg = False
        self.cv_image = False
        self.current_image = False

        # results variables
        self.image_faces_json = False
        self.image_objects_json = False
        self.image_texts_json = False
        self.image_labels_json = False
        self.image_landmarks_json = False
        self.image_logos_json = False       
        
        # converted variables
        self.image_faces = False
        self.image_objects = False
        self.image_texts = False
        self.image_labels = False
        self.image_landmarks = False
        self.image_logos = False

        # parameters
        self.display_image = True
        self.display_labels = True
        self.update_image_rate = 1

        #self.update_image_faces_rate = 1
        #self.update_image_objects_rate = 1
        #self.update_image_texts_rate = 5
        #self.update_image_labels_rate = 10
        #self.update_image_landmarks_rate = 60
        #self.update_image_logos_rate = 5

        self.update_image_faces_rate = 60
        self.update_image_objects_rate = 5
        self.update_image_texts_rate = 60
        self.update_image_labels_rate = 60
        self.update_image_landmarks_rate = 60
        self.update_image_logos_rate = 60

        # control variables
        self.have_image = False
        self.wait_window = True

        # set next updates
        self.update_image_time =  rospy.get_time()
        self.update_image_faces_time =  rospy.get_time()
        self.update_image_objects_time =  rospy.get_time()
        self.update_image_texts_time =  rospy.get_time()
        self.update_image_labels_time =  rospy.get_time()
        self.update_image_landmarks_time =  rospy.get_time()
        self.update_image_logos_time =  rospy.get_time() 

        # say hello to remote engine
        r = requests.get('https://gc.rosbots.pt/m1p1/hello.php')
        print(r.content)

        print ("init done")

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

    def show_face_detections(self, image, face_detect):
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

    def show_object_detections(self, image, object_detect):                           
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

    def create_service_request_texts(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['TEXT_DETECTION']
        req.max_results = [10]
        return req

    def create_service_request_labels(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['LABEL_DETECTION']
        req.max_results = [10]
        return req

    def create_service_request_landmarks(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['LANDMARK_DETECTION']
        req.max_results = [10]
        return req

    def create_service_request_logos(self, img):
        req = gc_srvs.RequestAnnotationsRequest()
        req.image = img
        req.annotations = ['LOGO_DETECTION']
        req.max_results = [10]
        return req

    def image_callback(self, data):
        # filter image for desired rate
        if rospy.get_time() < self.update_image_time:
            return
        self.update_image_time =  rospy.get_time() + self.update_image_rate
        # set image and control variables
        self.current_image = data
        self.have_image = True
    
    def decode_results(self, response):
        resp = eval(response)
        result = resp['responses'][0]
        return result

    def publish_detected_faces(self, faces):
        face_array_msg = FaceArrayStamped()
        if 'faceAnnotations' in faces:
            for f in faces['faceAnnotations']:
                try:
                    # set box for face
                    box = [(v['x'], v['y']) for v in f['fdBoundingPoly']['vertices']]

                    # set face message
                    face_msg = Face()

                    face_msg.face.x = box[0][0] + 100
                    face_msg.face.y = box[0][1] + 50
                    face_msg.face.width = box[1][0] - box[0][0]
                    face_msg.face.height = box[2][1] - box[1][1]

                    # set eyes in face message
                    #eyes_msg = Rect()
                    #eyes_msg.x = box[0][0] + 100
                    #eyes_msg.y = box[0][1] + 50
                    #eyes_msg.width = box[1][0] - box[0][0]
                    #eyes_msg.height = box[2][1] - box[1][1]
                    #face_msg.eyes.append(eyes_msg)

                    # set other face message data 
                    face_msg.label = ""
                    face_msg.confidence = f['detectionConfidence']

                    #print box
                    #print box[0][1]
                    #print face_msg
                    
                    face_array_msg.faces.append(face_msg)
                except KeyError:
                    fakevar = True

        #face_array_msg.header = msg_header
        # we need to use the same header because message filters
        face_array_msg.header = self.image_msg.header
        self.faces_pub.publish(face_array_msg)

    def publish_detection_results(self):
        if self.image_faces:
            self.result_pub.publish(json.dumps(self.image_faces, indent=2, sort_keys=True))
            self.publish_detected_faces(self.image_faces)
        if self.image_objects:
            self.result_pub.publish(json.dumps(self.image_objects, indent=2, sort_keys=True))
        if  self.image_faces or self.image_objects:
            #self.image_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(self.image_msg)

    def scan_text_response(self):
        print ("## debug ##")
        print ("# text")
        print ("# text")
        if 'textAnnotations' in self.image_texts:
            for entry in self.image_texts['textAnnotations']:
                if 'locale' in entry:
                    print(entry['locale'])
                    print(entry['description'])                

        print ("## debug ##")
        print ("# textAnnotations")
        print ("# textAnnotations")
        if 'textAnnotations' in self.image_texts:
            for entry in self.image_texts['textAnnotations']:
                print(entry)
        print ("## debug ## ")
        print ("# fullTextAnnotation")
        print ("# fullTextAnnotation")
        if 'fullTextAnnotation' in self.image_texts:
            for entry in self.image_texts['fullTextAnnotation']:
                print(entry)                    

        if 'textAnnotations' in self.image_texts:
            try:
                for text in self.image_texts['textAnnotations']:
                    if 'description' in text:
                        print('\n"{}"'.format(text['description']))
                        vertices = (['({},{})'.format(vertex.x, vertex.y)
                            for vertex in text['boundingPoly'][vertices]])
                        print('bounds: {}'.format(','.join(vertices)))
            except NameError:
                fakevar = True

    def spin(self):

        if self.have_image:

            updated_sets = []

            # debug
            start_time = time.time()

            # set image for request
            self.image_msg, self.cv_image = self.get_image()

            # request, get & store results about faces detected
            if rospy.get_time() > self.update_image_faces_time:
                
                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                req = self.create_service_request_faces(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_faces_json = resp.response
                self.image_faces = self.decode_results(resp.response)
                #print(self.image_faces_json)
                #print(type(self.image_faces_json))

                updated_sets.append("faces")

                self.update_image_faces_time = rospy.get_time() + self.update_image_faces_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("faces", elapsed_time)

            # request, get & store results about objects detected
            if rospy.get_time() > self.update_image_objects_time:

                if self.debug_speed:
                # debug
                    req_start_time = time.time()

                req = self.create_service_request_objects(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_objects_json = resp.response
                self.image_objects = self.decode_results(resp.response)

                updated_sets.append("objects")

                self.update_image_objects_time = rospy.get_time() + self.update_image_objects_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("objects", elapsed_time)

            # request, get & store results about texts detected
            if rospy.get_time() > self.update_image_texts_time:

                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                req = self.create_service_request_texts(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_texts_json = resp.response
                self.image_texts = self.decode_results(resp.response)

                updated_sets.append("texts")

                self.update_image_texts_time = rospy.get_time() + self.update_image_texts_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("texts", elapsed_time)

            # request, get & store results about labels about image
            if rospy.get_time() > self.update_image_labels_time:

                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                req = self.create_service_request_labels(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_labels_json = resp.response
                self.image_labels = self.decode_results(resp.response)

                updated_sets.append("labels")

                self.update_image_labels_time = rospy.get_time() + self.update_image_labels_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("labels", elapsed_time)
            
            # request, get & store results about landmarks detected
            if rospy.get_time() > self.update_image_landmarks_time:

                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                req = self.create_service_request_landmarks(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_landmarks_json = resp.response
                self.image_landmarks = self.decode_results(resp.response)

                updated_sets.append("landmarks")

                self.update_image_landmarks_time = rospy.get_time() + self.update_image_landmarks_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("landmarks", elapsed_time)

            # request, get & store results about logos detected
            if rospy.get_time() > self.update_image_logos_time:

                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                req = self.create_service_request_logos(self.image_msg)
                GetAnnotationSrv = rospy.ServiceProxy('get_annotation', gc_srvs.RequestAnnotations)
                resp = GetAnnotationSrv(req)
                self.image_logos_json = resp.response
                self.image_logos = self.decode_results(resp.response)

                updated_sets.append("logos")

                self.update_image_logos_time = rospy.get_time() + self.update_image_logos_rate

                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("logos", elapsed_time)
            
            # merge results of detection with image
            #if type(self.image_faces) is not bool:
            #    print (type(self.image_faces))
            self.show_face_detections(self.cv_image, self.image_faces)
            
            #if type(self.image_objects) is not bool:
            #    print (type(self.image_faces))
            self.show_object_detections(self.cv_image, self.image_objects)

            # debug
            #print json.dumps(self.image_faces, indent=2, sort_keys=True)
            #print json.dumps(self.image_objects, indent=2, sort_keys=True)
            #print (json.dumps(self.image_texts, indent=2, sort_keys=True))
            #print("## labels ##")
            #print (json.dumps(self.image_labels, indent=2, sort_keys=True))
            #print("## landmarks ##")
            #print (json.dumps(self.image_landmarks, indent=2, sort_keys=True))
            #print("## logos ##")
            #print (json.dumps(self.image_logos, indent=2, sort_keys=True))

            #self.scan_text_response()
            
            # send data to remote engine
            if len(updated_sets) > 0:
                
                #debug
                print ("send to vision gate")

                # debug
                if self.debug_speed:
                    req_start_time = time.time()

                url = "https://gc.rosbots.pt/m1p1/visiongate.php"
                headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
                r = requests.post(url, data=json.dumps(self.image_objects), headers=headers)
                
                # debug
                if self.debug_speed:
                    elapsed_time = time.time() - req_start_time
                    print("remote", elapsed_time)
                
                print(r.content)

            # reset output if no detections
            if not 'faceAnnotations' in self.image_faces:
                self.image_faces = []
                print ("No faces")
            if not 'localizedObjectAnnotations' in self.image_objects:
                self.image_faces = []
                print ("No objects")

            # debug
            if self.debug_speed:
                req_start_time = time.time()

            # publish results on topics
            self.publish_detection_results()

            # show debug window
            if self.display_image:
                cv2.imshow('modified', self.cv_image)
            
                if self.wait_window:
                    cv2.waitKey(30)
                else:
                    cv2.waitKey(1)

            # debug
            if self.debug_speed:
                elapsed_time = time.time() - req_start_time
                print("publish", elapsed_time)

            elapsed_time = time.time() - start_time
            print("loopTime", elapsed_time, updated_sets)

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
