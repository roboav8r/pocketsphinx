#!/usr/bin/env python

import argparse
import rospy

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio, math, audioop, os, time, wave

from std_msgs.msg import String, Int32
from std_srvs.srv import *
from collections import deque

class recognizer(object):

    def __init__(self):

        # initialize ROS
        self.speed = 0.2

        # Start node
        rospy.init_node("recognizer")
        rospy.on_shutdown(self.shutdown)

        self._kws_param = "~kws"
        self._stream_param = "~stream"
        self._wavpath_param = "~wavpath" #Path to .wav file, specified in .launch file

        # Initialize ROS publishers
        self.str_pub = rospy.Publisher('utterance', String, queue_size=1)
        self.prob_pub = rospy.Publisher('probability', Int32, queue_size=1)
        self.best_pub = rospy.Publisher('best_score', Int32, queue_size=1)
        
        # Get microphone parameters
        self.format = rospy.get_param('microphone/format', 'pyaudio.paInt16')
        self.channels = rospy.get_param('microphone/channels', 1)
        self.rate = rospy.get_param('microphone/rate', 16000)
        self.chunk = rospy.get_param('microphone/chunk', 1024)
        self.threshold = rospy.get_param('microphone/threshold', 4500)
        self.prev_audio = rospy.get_param('microphone/prev_audio', 0.5)
        self.silence_limit = rospy.get_param('microphone/silence_limit', 1.0)
        
        # Read in parameters to define model & dictionary
        self.model_dir=rospy.get_param('model_dir', '/usr/local/share/pocketsphinx/model')
        self.lm=rospy.get_param('lm', 'en-us/en-us.lm.bin')
        self.hmm=rospy.get_param('hmm', 'en-us/en-us')
        self.dict=rospy.get_param('dict', 'en-us/cmudict-en-us.dict')

        if rospy.has_param(self._stream_param):
            self.is_stream = rospy.get_param(self._stream_param)
            if not self.is_stream:
                if rospy.has_param(self._wavpath_param):
                    self.wavpath = rospy.get_param(self._wavpath_param)
                    if self.wavpath == "none":
                        rospy.logerr('Please set the wav path to the correct file location')
                else:
                    rospy.logerr('No wav file is set')
        else:
            rospy.logerr('Audio is not set to a stream (true) or wav file (false).')
            self.is_stream = rospy.get_param(self._stream_param)

        self.start_recognizer()

    def start_recognizer(self): # Configure / initialize pocketsphinx Decoder object
        
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        config.set_string('-hmm', os.path.join(self.model_dir, self.hmm)) # Hidden Markov model
        config.set_string('-lm', os.path.join(self.model_dir, self.lm)) # Language model
        config.set_string('-dict', os.path.join(self.model_dir, self.dict)) # Pronunciation dictionary used
        
        self.decoder = Decoder(config) # Create pocketsphinx decoder with specified configuration
        rospy.loginfo("Done initializing pocketsphinx")

    def init_mic(self, num_samples=50):
        #Gets average audio intensity of your mic sound. You can use it to get
        #The average is the avg of the .2 sec of the largest intensities recorded.
        
        print("Getting average microphone intensity")
        p = pyaudio.PyAudio()
        stream = p.open(format=eval(self.format), channels=self.channels, rate=self.rate, input=True, frames_per_buffer=self.chunk)
        
        values = [math.sqrt(abs(audioop.avg(stream.read(self.chunk), 4)))
                  for x in range(num_samples)]
        values = sorted(values, reverse=True)
        r = sum(values[:int(num_samples * 0.2)]) / int(num_samples * 0.2)
        print("Average audio intensity is ", r)
        stream.close()
        p.terminate()

        self.threshold = r + 100    
        rospy.set_param('microphone/threshold', r+100)


    def save_speech(self, data, p): # Saves mic data to .wav file & returns filename

        filename = 'output_'+str(int(time.time()))
        # writes data to WAV file
        data = ''.join(data)
        wf = wave.open(filename + '.wav', 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(data)
        wf.close()
        return filename + '.wav'


    def decode_phrase(self, wav_file): # Decode text of a .wav file
        self.decoder.start_utt()
       
        try:
            stream = open(wav_file, "rb")
        except:
            rospy.logerr('Please set the wav path to the correct location from the pocketsphinx launch file')
            rospy.signal_shutdown()
        
        in_speech_bf = False        
        
        while not rospy.is_shutdown():
          buf = stream.read(1024)
          if buf:
            self.decoder.process_raw(buf, False, False)
          else:
            break
        self.decoder.end_utt()
        
        words = []
        [words.append(seg.word) for seg in self.decoder.seg()]
        hypothesis = self.decoder.hyp()
        
        return words, hypothesis
    
    
    def run(self): # Main loop / body
        
        if not self.is_stream: # To decode a single .wav
            words,hypothesis = self.decode_phrase(self.wavpath)
            if hypothesis != None:
                self.publish_result(hypothesis)
                print(hypothesis.hypstr)     
            else:
                rospy.logwarn("Unable to form hypothesis from .wav file.")
                      
        
        else: # Continuously decode audio above threshold
            # Initialize microphone / record noise threshold
            self.init_mic()
        	   
        	   # Pocketsphinx requires 16kHz, mono, 16-bit little-Endian audio.
            # See http://cmusphinx.sourceforge.net/wiki/tutorialtuning
            p = pyaudio.PyAudio() #Create pyaudio object
            stream = p.open(format=eval(self.format), channels=self.channels, rate=self.rate, input=True, frames_per_buffer=self.chunk)
            
            audio2send = []
            cur_data = ''  # current chunk of audio data
            rel = self.rate/self.chunk
            slid_win = deque(maxlen=self.silence_limit * rel)
            
            #Prepend audio from prev_audio before noise was detected
            prev_audio = deque(maxlen=self.prev_audio * rel)
            started = False
                       
            # Main loop
            while not rospy.is_shutdown():
                	 
                cur_data = stream.read(self.chunk)
                slid_win.append(math.sqrt(abs(audioop.avg(cur_data, 4))))
                
                if sum([x > self.threshold for x in slid_win]) > 0: # If noise level exceeds threshold, start recording
                    if started == False:
                        print("Recording phrase")
                        started = True
                    audio2send.append(cur_data)
                
                elif started:
                    print("Finished recording; decoding phrase")
                    filename = self.save_speech(list(prev_audio) + audio2send, p)
                    words,hypothesis = self.decode_phrase(filename)
                    
                    if hypothesis != None:
                        self.publish_result(hypothesis)
                    
                    # Removes temp audio file
                    os.remove(filename)
                    # Reset all
                    started = False
                    slid_win = deque(maxlen=self.silence_limit * rel)
                    prev_audio = deque(maxlen=0.5 * rel)
                    audio2send = []
                    print("Listening ...")
                    
                else:
                    prev_audio.append(cur_data)
                    

    def publish_result(self, hypothesis):
        		print(hypothesis.hypstr)
        		self.str_pub.publish(hypothesis.hypstr)
        		self.prob_pub.publish(hypothesis.prob)
        		self.best_pub.publish(hypothesis.best_score)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stopping PocketSphinx")

if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = recognizer()
        start.run()
