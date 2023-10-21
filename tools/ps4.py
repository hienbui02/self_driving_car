from pyPS4Controller.controller import Controller

def disconnect():
    # any code you want to run during loss of connection with the controller or keyboard interrupt
    print("1")
    pass

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


controller = MyController(interface="/dev/input/js0")
controller.listen( on_disconnect=disconnect)
