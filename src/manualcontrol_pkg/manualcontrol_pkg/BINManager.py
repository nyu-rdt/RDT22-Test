#!/usr/bin/env python

"""
BINManager.py
This file acts as the BIN Manager in the Arena, primary purpose is to receive commands from GCS
This file was derived from Manager.py, which was broken into BINManager.py and GCSManager.py
Interacts with jumptable.py, ROSTable.py and KeyMap.py

Publishes to "SendBuffer"
Subscribes to "RecvBuffer"

Hosts an MQTT client
NOTE: While this file hosts the client, other files dictate when and to what topic data gets sent

Modification 4/1 : Cleaned redundant functions and deleted unused functions
"""


# import filenames-interpreter and user-input libraries
import sys, rospy, struct
import paho.mqtt.client as mqtt

from std_msgs.msg import String
from jumptable import *


# pop up status indicator window for connection status 
# green circle for good connection, red for bad

def on_connect(client, userdata, flag, rc):
    rospy.loginfo("DEBUG: BINMANAGER CONNECTED")

class BINManager:
    def __init__(self, title="  RDT Command Framework", width=640, height=480):
        """
        Constructor for BINManager
        parameters:
            title: Title of pygame window
            width: Width of pygame window
            height: Height of pygame window
        """
        self.commands = FunctionTable() 
        self.width = width
        self.height = height
        self.keyCompressedPrev = 0
        self.keyCompressed = 0
        self.headerSize = 4

        # Variables needed to communicate with Altas Socket
        self.sendHandler = rospy.Publisher("SendBuffer", String, queue_size=10)
        self.recvHandler = rospy.Subscriber("RecvBuffer", String, self.ReceiveCallback)
        
        self.client = mqtt.Client()
        self.client.on_connect = on_connect
        self.client.connect("localhost", 1883)
    

    # https://www.pygame.org/docs/ref/key.html

    def loop(self):
        """
         Loop function for BINManager, which returns none
        """    
        while(not rospy.is_shutdown()):
            self.client.loop()

        
    def recv(self, data):
        """
        Function to handle received data
        parameters:
            data: Received data
        returns: None
        """
        data = data.data 
        sizeOfInt = 4   
        if len(data) < sizeOfInt: 
            return
            
        # Parsing command
        self.commands.parse(data, sizeOfInt )

    def ReceiveCallback(self, msg):
        """
        Callback function for subscriber
        parameters:
            msg: Received message from subscriber
        returns: None
        """
        self.recv(msg)
