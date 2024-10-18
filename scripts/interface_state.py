# class InterfaceStates:
#     def __init__(self):
#         self.STATE_INIT = 0
#         self.STATE_ACTIVE = 1
#         self.STATE_DONE = 2


"""
Based on the state message file:
    uint8 PRE_INIT = 0
    uint8 INIT_READY = 1
    uint8 ACTIVE = 2
    uint8 DONE = 3
    uint8 FAILURE = 4

"""
class InterfaceStates:
    def __init__(self):
        self.PRE_INIT = 0
        self.INIT_READY = 1
        self.ACTIVE = 2
        self.DONE = 3
        self.FAILURE = 4


