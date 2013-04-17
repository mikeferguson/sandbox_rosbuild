#!/usr/bin/python

import roslib; roslib.load_manifest('openni_audio')
import rospy

import struct

import ao
from ao import AudioDevice 

from audio_common_msgs.msg import AudioData

dev = AudioDevice(2, bits=16, rate=48000,channels=1)

def audio_cb(msg):
    d = [ struct.unpack('h', msg.data[i]+msg.data[i+1])[0] for i in range(len(msg.data)) if i%4 == 0]
    d = [ int(x) *4 for x in d]
    data = struct.pack('h'*len(d), *d)
    dev.play(data)

rospy.init_node("ao_player")

rospy.Subscriber('/audio_in/data', AudioData, audio_cb)
rospy.spin()

