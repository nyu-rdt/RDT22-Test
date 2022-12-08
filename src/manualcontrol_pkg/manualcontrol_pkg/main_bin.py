#!/usr/bin/env python

"""
Bridges commands from GCS to ESP

Main file and command handler for the BIN.
Used to send values for esp
This file is a ros node called "BIN_command_handler"
NOTE: This file invokes publish call, but BINManager hosts the MQTT client

File is tested
Modification 4/1: deleted commands associated with RosTable
"""

import rospy
from BINManager import BINManager
from KeyMap import *

# List containing bools, indicates key presses
loco_status = [0]*6
exca_status = [0]*10
stop_status = [0]

loco_speed = 1
exca_speed = 1

shovel_toggle = False
auger_toggle = False
bucket_toggle = False

loco_hex = ['A', 'B', 'C', 'D']
exca_hex = ['3', '4', '5', '7', '6', '7', '2', 'E']
stop_hex = ['9']

# Using BINManager
manager = BINManager()

def debug_loco_data(hex_code):
    """
    Function used to publish locomotion data to the robot
    """
    global manager 

    # debug: creating a string of all pressed keys
    pressed = "Pressed Keys: "
    keys = ['W', 'S', 'A', 'D', 'Q', 'E'] # Locomotion keys
    for i in range(len(loco_status)):
        pressed += keys[i] if loco_status[i] else ''

    # debug: logging data
    debug_log(pressed, loco_status, hex_code)


def debug_exca_data(hex_code):
    """
    Function used to publish excavation data to the robot
    """
    global manager

    # debug: creating a string of all pressed keys
    pressed = "Pressed Keys: "
    keys = ['R', 'F', 'B', 'N', 'M', 'V', 'Z', 'X', 'C', 'T', 'G'] # Excavation keys (in-order)
    for i in range(len(exca_status)):
        pressed += keys[i] if exca_status[i] else ''
    
    # debug: logging data
    debug_log(pressed, exca_status, hex_code)


def debug_stop_data(hex_pub):
    """
    Function used to publish emergency stop data to the robot
    """
    global manager

    # debug: creating a string of all pressed keys
    pressed = "Pressed Keys: "
    keys = ['L'] # Stop keys
    for i in range(len(stop_status)):
        pressed += keys[i] if stop_status[i] else ''
    
    # debug: logging data
    debug_log(pressed, stop_status, hex_pub)


def debug_log(pressed, status, data):
    rospy.loginfo(pressed)
    print(status)
    print("Out data: " + data)
    print('----------------')


def loco_decoder(data):
    """
    Function used to decode locomotion data for 10 keys
    data: Input data to be decoded
    """
    # Decoding the data into a list
    for i in range(6):
        loco_status[i] = (data[0] & 2**i) >> i
    
    # Getting the correct hex id based on the key
    hex = '1'
    for i in range(4):
        if loco_status[i]: 
            hex = loco_hex[i]
            break
    
    # Adjusting the speed
    global loco_speed
    
    if hex == '1': loco_speed = 1

    if loco_status[4] and loco_speed < 4:
        loco_speed += 1
    elif loco_status[5] and loco_speed > 0:
        loco_speed -= 1

    # Publishing and logging data
    hex_pub = hex + str(loco_speed)
    manager.client.publish('robot/controller', hex_pub)
    debug_loco_data(hex_pub)


def exca_decoder(data):
    """
    Function used to decode excavation data for 10 keys
    data: Input data to be decoded
    """
    # Decoding the data into a list
    # Range updated to reflect addition of 'C' key
    for i in range(11):
        exca_status[i] = (data[i//8] & 2**(i%8)) >> (i%8)
    
    # Getting the correct hex id based on the key
    hex = '1'
    for i in range(8):
        if exca_status[i]: 
            hex = exca_hex[i]
            break
    
    global exca_speed
    global auger_toggle
    global bucket_toggle
    global shovel_toggle

    second_nib = '1'

    # Getting the second nibble of the message
    if isToggle(hex): # Toggle values
        # Updated to exclude 'E'
        if hex == '2':
            bucket_toggle = not bucket_toggle
            second_nib = str(int(bucket_toggle))
        elif hex == '7':
            if exca_status[3]: shovel_toggle = 0
            else:              shovel_toggle = 1
            second_nib = str(int(shovel_toggle))
    elif hex == 'E':
        # X: E0, C: E1
        # REMINDER: CURRENT TEENSY FILE HAS E0 AND E1'S ACTIONS SWAPPED
        if exca_status[7]:
            second_nib = '0'
    else:   # Speed adjustments
        # Updated indices w additonal key
        if exca_status[9] and exca_speed < 4:
            exca_speed += 1
        elif exca_status[10] and exca_speed > 0:
            exca_speed -= 1
        second_nib = str(exca_speed)

    # Publishing and debugging data
    hex_pub = hex + second_nib
    manager.client.publish('robot/controller', hex_pub)
    debug_exca_data(hex_pub)


def isToggle(id):
    """ Function used to determine if an ID belongs to a toggle action """
    # Removed 'E' as a toggle
    return id == '2' or id == '7'


def stop_decoder(data):
    """
    Function used to decode emergency stop data for 1 key
    data: Input data to be decoded
    """
    # Decoding the data into a list
    for i in range(1):
        stop_status[i] = (data[0] & 2**i) >> i

    # Getting the correct hex value based on the key press
    hex = '1'
    if stop_status[0]: hex = stop_hex[0]

    # Publishing and debugging data
    hex_pub = hex + '0'
    manager.client.publish('robot/controller', hex_pub)
    debug_stop_data(hex_pub)


def run(argc, argv):
    """
    Main function of the file
    parameters:
        argc: TBD ?? (Unused)
        argv: TBD ?? (Unused)
    returns: None
    """
    global manager 

    # Init node
    rospy.init_node('BIN_command_handler')

    # Inserting commands
    keys_locomotion = [
        Key_library.W,
        Key_library.S,
        Key_library.A,
        Key_library.D,
        Key_library.Q,
        Key_library.E
    ]

    keys_excavation = [
        Key_library.R, # Auger up
        Key_library.F, # Auger up
        Key_library.B, # Rotate CCW
        Key_library.N, # STOP shovel
        Key_library.M, # Rotate CW
        Key_library.V, # Shovel Shake
        Key_library.Z, # Bucket toggle
        Key_library.X, # Pivot to 45 degrees
        Key_library.C, # Pivot to 90 degrees
        Key_library.T, # Speed up
        Key_library.G, # Speed down
    ]

    keys_emergency = [
        Key_library.L
    ]

    manager.commands.insert(1, keys_locomotion, loco_decoder)
    manager.commands.insert(2, keys_excavation, exca_decoder)
    manager.commands.insert(1, keys_emergency, stop_decoder)
    
    # Looping manager
    manager.loop()

# Running
run(1, 1)


