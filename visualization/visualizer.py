from pyray import *

class Visualizer:
    def __init__(self, width=800, height=450, name="Visualization"):
        init_window(width, height, name)
        self.background = BLACK
        self.on = True
        
        begin_drawing()

    def create_drawing(self):
        begin_drawing()
        # clear_background(self.background)
        
        # Add functions to show things on screen
        
        end_drawing()
        
    def window_is_open(self):
        return not window_should_close()

    def close(self):
        close_window()
        self.on = False