#!/usr/bin/env python
import rospy
import apiai
import json

from std_msgs.msg import String


import re, math
from collections import Counter

WORD = re.compile(r'\w+')

# Authentication
SESSION_ID = ''

class DialogflowNode(object):
    """
    ROS node for the Dialogflow natural language understanding.
    Dialogflow parses natural language into a json string containing the semantics of the text. 
    The received text in the subscribed ros topic is processed and the raw result published
    as a topic message.
    """
    def __init__(self):
        rospy.init_node('dialogflow_node')
        rospy.Subscriber("speechin", String, self.speech_callback, queue_size=10)
        try:
            self._client_access_token = rospy.get_param("~client_access_token")
        except rospy.ROSException:
            rospy.logfatal("Missing required ROS parameter client_access_token")
            exit(1)

        self.ai = apiai.ApiAI(self._client_access_token)

        self.resultraw_pub = rospy.Publisher("result_raw", String, queue_size=10)
        self.speechout_pub = rospy.Publisher("speechout", String, queue_size=10)
        self.request = None

        self.last_speechout = ""

        print ("dialogflow_node init done")

    def get_cosine(self, vec1, vec2):
        intersection = set(vec1.keys()) & set(vec2.keys())
        numerator = sum([vec1[x] * vec2[x] for x in intersection])

        sum1 = sum([vec1[x]**2 for x in vec1.keys()])
        sum2 = sum([vec2[x]**2 for x in vec2.keys()])
        denominator = math.sqrt(sum1) * math.sqrt(sum2)

        if not denominator:
            return 0.0
        else:
            return float(numerator) / denominator

    def text_to_vector(self, text):
        words = WORD.findall(text)
        return Counter(words)

    def speech_callback(self, msg):
        print ("Got message.")
        self.request = self.ai.text_request()
        self.request.query = msg.data

        rospy.logdebug("Waiting for response...")
        response = self.request.getresponse()
        rospy.logdebug("Got response, and publishing it.")

        # prepare response
        result = response.read()

        # import to dict
        json_str = json.loads(json.dumps(result))
        json_data = json.loads(json_str)

        # publish to raw topic
        result_msg = String()
        result_msg.data = result
        self.resultraw_pub.publish(result_msg)

        #
        # process result
        #
        speechout = json_data['result']['fulfillment']['messages'][0]['speech']
        
        # try to avoid voice feedback
        vector1 = self.text_to_vector(self.last_speechout)
        vector2 = self.text_to_vector(speechout)
        cosine = self.get_cosine(vector1, vector2)
        # debug
        print (speechout, self.last_speechout)
        print 'Cosine:', cosine

        # publish speech topic
        #result_msg.data = speechout
        #self.speechout_pub.publish(result_msg)

        # check similarity cosine level to publish
        if cosine < 0.5:
            result_msg.data = speechout
            self.speechout_pub.publish(result_msg)
            self.last_speechout = speechout
        else:
            self.last_speechout = ""

        #print (json_data)
        #print (json_data['id'])
        #print (speechout)


if __name__ == "__main__":
    dialogflow_node = DialogflowNode()
    rospy.spin()
