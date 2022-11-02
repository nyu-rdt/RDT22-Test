'''
nested_state_machine.py

Simple file to test state machine implementation
Creates and executes a nested state machine, composed of 3 linear state machines
'''

from nyurdt_state_test.nyurdt_state_machine import StateMachine
from time import sleep

# Callback functions for when each machine completes 
def callback_mach1():
    print('Machine 1 is done!')

def callback_mach2():
    print('Machine 2 is done!')

def callback_mach3():
    print('Machine 3 is done!')

def callback_master():
    print('Master machine is done!')

# Stages for the first machine
def stage1_mach1():
    print('Mach1 -- Stage 1')
    sleep(1)
    return 'stage_2'

def stage2_mach1():
    print('Mach1 -- Stage 2')
    sleep(1)
    return None

# Stages for the second machine
def stage1_mach2():
    print('Mach2 -- Stage 1')
    sleep(1)
    return None

# Stages for the third machine
def stage1_mach3():
    print('Mach3 -- Stage 1')
    sleep(1)
    return 'stage_2'

def stage2_mach3():
    print('Mach3 -- Stage 2')
    sleep(1)
    return None

def main():
    # Creating 3 state machines
    machine1 = StateMachine('Machine 1', terminate='mach2', callback=callback_mach1)    
    machine2 = StateMachine('Machine 2', terminate='mach3', callback=callback_mach2)
    machine3 = StateMachine('Machine 3', callback=callback_mach3)

    # Adding states to each machine
    machine1.add_state('stage_1', stage1_mach1)
    machine1.add_state('stage_2', stage2_mach1)

    machine2.add_state('stage_1', stage1_mach2)

    machine3.add_state('stage_1', stage1_mach3)
    machine3.add_state('stage_2', stage2_mach3)

    # Creating the master state machine
    master = StateMachine('Master Machine', callback=callback_master)

    # Adding the sub machines to the master
    master.add_state('mach1', machine1.execute)
    master.add_state('mach2', machine2.execute)
    master.add_state('mach3', machine3.execute)

    # Running the master
    master.execute()