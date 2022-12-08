"""
jumptable.py
This file handles the insertion, encoding and decoding of functions
It uses keypresses to determine how to encode/decode data
Based on legacy c++ code (JumpTable.cpp found in RMC20)
Interacts with KeyMap.py
modification 4/1: Cleaned unnessary code, and deleted code associated with ROStable
"""

from KeyMap import Key_library
import  rospy 
from std_msgs.msg import String

class Functions:
    # constructor
    def __init__(self, keyboard, decoder, num_bytes=0):
        """
        Constructor for Functions
        parameters:
            keyboard: Keyboard
            decoder: Function with list param, func(chr[]) -> None
            num_bytes: Number of bytes, int
        """
        self.keyboard = keyboard
        self.decoder = decoder
        self.num_bytes = num_bytes

        # List of tuples of form (chr, int)
        self.func_vector = None
        
        #ros topic we are publishing to
        # self.topic = topic
        # self.sendHandler = rospy.Publisher(topic, return_type, queue_size=10)


    
class FunctionTable:
    # constructors
    def __init__(self):
        """
        Constructor for FunctionTable
        """
        # Current index
        self.curr_index = 0

        # Represents key presses, each bit is a different key
        self.compressed_used = 0

        # List of tuples of form (chr, int)
        self.func_vector = []

        # lst(Functions)
        self.func_table = []


    def __del__(self):
        """
        Destructor for FunctionTable
        """
        self.curr_index = None
        self.compressed_used = None
        self.manage = None
        self.func_vector = None

    
    def insert(self, num_bytes, keyboard, decoder):
        """
        Function used to insert a function into func_table by providing its components
        parameters:
            num_bytes: Number of bytes, int
            keyboard: Keyboard
            decoder: Function with list param, func(chr[]) -> None
        returns: None
        """
        func = Functions(keyboard, decoder, num_bytes)
        self.insert_function(func)

    
    def insert_function(self, function):
        """
        Function used to insert a function into func_table by providing the function
        parameters:
            f: Functions objec
        returns: None
        """
        # Adding function to func_table
        self.func_table.append(function)

        # tot is binary representation of used keys
        tot = 0
        # easy solution would be to change KeyMap from Enum to IntEnum, see how it affects other files first
        # code works with other files
        for k in function.keyboard:
            tot |= 1 << k.value
    
        # Updaing compressed_used
        self.compressed_used |= tot

        # Appending and incrementing curr_index
        self.func_vector.append((self.curr_index, tot))
        self.curr_index += 1




    def parse(self, key_val, curr):
        '''
        Taking a snippet of key_val that is length f.num_bytes and then passing it to the decode function
        parameters:
            key_val: list, chr[]
            curr: starting index, int
        returns: None
        '''
        # While within bounds of key_val
        while curr < len(key_val):
            # Getting the function
            f = self.func_table[ord(key_val[curr])]
            curr += 1

            if f.decoder != None:
                # Parsing a portion of key_val f.num_bytes long
                data = [0] * f.num_bytes
                for i in range(0, f.num_bytes):
                    data[i] = ord(key_val[i + curr])
                
                # Advancing curr and passing to decoder
                curr += f.num_bytes
                f.decoder(data)
                
