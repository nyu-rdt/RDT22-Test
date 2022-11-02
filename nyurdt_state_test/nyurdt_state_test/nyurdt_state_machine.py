'''
nyurdt_state_machine.py

NYU RDT's implementation of a state machine.
States are added one at a time to the machine, then can be executed from a valid starting state.
Supports functionality for nested state machines.
'''

class StateMachine:
    
    def __init__(self, machine_name: str, terminate=None, callback=None) -> None:
        '''
        Constructor for a StateMachine

        args:
            machine_name: The name of this state machine
            terminate: Return value for when this machine stops executing
            callback: Function to be called when this machine stops executing
        
        returns: None
        '''
        self.name = machine_name
        self.terminate = terminate
        self.callback = callback

        '''
        Dictionary of states
        maps state_name -> state_function
        '''
        self.states = dict()
        # Starting state
        self.start = None

    
    def add_state(self, state_name: str, state_function) -> None:
        '''
        Add a new state to the state machine

        args:
            state_name: name of the state to be added
            state_function: function that will execute during this state
        
        returns: None
        '''
        # State names must be unique
        if state_name in self.states:
            raise ValueError('[{}] State already exists: {}'.format(
                self.name, state_name
            ))
        
        # Setting a starting node
        if self.start is None: self.start = state_name
        # Adding the state
        self.states[state_name] = state_function
    
    def execute(self, state_name=None):
        '''
        Execute this state machine

        args:
            state_name: name of the state to start execution at
        
        returns: this state machines terminator
        '''
        # Select the starting state
        current = state_name or self.start

        # While we haven't reached a terminating state
        while current is not None:
            # Get the next function to call
            action = self.states.get(current, None)
            # Check if function is valid
            if action is None:
                raise ValueError('[{}] Invalid state: {}'.format(
                    self.name, current
                ))
            # Execute state
            current = action()
        
        # Call callback function and return terminatin' value 
        if callable(self.callback): self.callback()
        return self.terminate
