'''
single_state_machine.py

Simple file to test state machine implementation
Creates and executes a linear state machine with 3 states
'''

from nyurdt_state_test.nyurdt_state_machine import StateMachine
from time import sleep

# Callback function for when the machine finishes 
def callback():
    print('Machine is done!')

# Stages for the machine
def stage1():
    print('Stage1')
    sleep(2)
    return 'stage_2'

def stage2():
    print('Stage2')
    sleep(2)
    return 'stage_3'

def stage3():
    print('Stage3')
    sleep(2)
    return None

def main():
    # Creating the machine
    machine = StateMachine('sample_machine', callback=callback)

    # Adding states
    machine.add_state('stage_1', stage1)
    machine.add_state('stage_2', stage2)
    machine.add_state('stage_3', stage3)

    # Executing the machine
    machine.execute()