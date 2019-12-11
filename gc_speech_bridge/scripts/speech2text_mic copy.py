#!/usr/bin/env python

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
import Queue
import rospy
from std_msgs.msg import String


class GspeechClient(object):
    def __init__(self):
        # Audio stream input setup
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        self.CHUNK = 4096
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                      rate=RATE, input=True,
                                      frames_per_buffer=self.CHUNK,
                                      stream_callback=self._get_data)
        self._buff = Queue.Queue()  # Buffer to hold audio data
        self.closed = False

        # ROS Text Publisher
        text_topic = rospy.get_param('speech', 'speech')
        self.text_pub = rospy.Publisher(text_topic, String, queue_size=10)
       
        # debug
        print("mic: init")

    def _get_data(self, in_data, frame_count, time_info, status):
        """Daemon thread to continuously get audio data from the server and put
         it in a buffer.
        """
        # Uncomment this if you want to hear the audio being replayed.
        self._buff.put(in_data)
        
        # debug
        print("mic: _get_data")

        return None, pyaudio.paContinue

    def _generator(self):
        """Generator function that continuously yields audio chunks from the buffer.
        Used to stream data to the Google Speech API Asynchronously.
        """

        # debug
        print("mic: _generator")

        while not self.closed:

            # debug
            print("mic: _generator while")

            # Check first chunk of data
            chunk = self._buff.get()
            if chunk is None:
                # debug
                print("mic: _generator no chunk")
                return
            data = [chunk]

            # Read in a stream till the end using a non-blocking get()
            while True:
                
                # debug
                print("mic: _generator while true")

                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except Queue.Empty:
                    break

            # debug
            print("mic: _generator after while true")

            yield b''.join(data)

    def _listen_print_loop(self, responses):
        """Iterates through server responses and prints them.
        The responses passed is a generator that will block until a response
        is provided by the server.
        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.
        """

        # debug
        print("mic: _listen_print_loop")

        for response in responses:

            # debug
            print("mic: _listen_print_loop for")

            # If not a valid response, move on to next potential one
            if not response.results:

                # debug
                print("mic: _listen_print_loop for no result")

                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:

                # debug
                print("mic: _listen_print_loop for no result alternatives")

                continue

            # debug
            print("mic: _listen_print_loop for alternative")

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Parse the final utterance
            if result.is_final:
                rospy.loginfo("Google Speech result: {}".format(result))
                # Received data is Unicode, convert it to string
                transcript = transcript.encode('utf-8')
                # Strip the initial space, if any
                if transcript.startswith(' '):
                    transcript = transcript[1:]
                # Exit if needed
                if transcript.lower() == 'sair':
                    self.shutdown()

                # debug
                print (transcript.decode('utf-8'))
                # Send the transcripted text sentence to topic
                self.text_pub.publish(transcript)
    
    def gspeech_client(self):
        """Creates the Google Speech API client, configures it, and sends/gets
        audio/text data for parsing.
        """
        language_code = 'pt-PT'
        client = speech.SpeechClient()
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=language_code)
        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=True)
        # Hack from Google Speech Python docs, very pythonic c:
        requests = (types.StreamingRecognizeRequest(audio_content=content) for content in self._generator())
        responses = client.streaming_recognize(streaming_config, requests)
        self._listen_print_loop(responses)

    def shutdown(self):
        """Shut down as cleanly as possible"""
        rospy.loginfo("Shutting down")
        self.closed = True
        self._buff.put(None)
        self.stream.close()
        self.audio.terminate()
        exit()

    def start_client(self):
        """Entry function to start the client"""
        try:
            rospy.loginfo("Starting Google speech mic client")
            self.gspeech_client()
        except KeyboardInterrupt:
            self.shutdown()


if __name__ == '__main__':
    rospy.init_node('speech2text_mic')
    g = GspeechClient()
    g.start_client()

