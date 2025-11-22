from pyray import *
from environments.environment import Environment

BORDER = 10

# TODO: add colors as parameters
# TODO: add buttons to start/pause/stop simulation

class Visualizer:
    def __init__(self, environment, width=800, height=450, name="Visualization"):
        init_window(width, height, name)
        
        self.background = BLACK
        self.on = True
        
        self.width = width
        self.height = height
        
        self.right_border = 50
        self.left_border = 50
        self.bottom_border = 100
        self.top_border = 80
        
        assert isinstance(environment, Environment)
        self.define_environment(environment)
        
    def create_drawing(self):
        begin_drawing()
        
        clear_background(self.background)
        self.add_title()
        self.add_legend()
        
        # Add functions to show "moving" things on screen
        self.draw_environment()
        self.draw_agents()
        #self.add_env_description()
        
        end_drawing()
        
    def define_environment(self, environment):
        self.environment = environment
        
        x, y = self.environment.get_dimensions()
        self.scale_env = min( 
                            (self.width - self.right_border - self.left_border) / x,
                            (self.height - self.bottom_border - self.top_border) / y
                            )
        print("Environment scale in visualizer set to: " + str(self.scale_env))
        
    def draw_environment(self):
        # Draw external outline of the environment
        draw_rectangle_lines(
            self.env_to_screen((0, 0))[0], 
            self.env_to_screen((0, 0))[1], 
            self.env_to_screen((self.environment.get_dimensions()[0], self.environment.get_dimensions()[1]))[0] - self.env_to_screen((0, 0))[0], 
            self.env_to_screen((self.environment.get_dimensions()[0], self.environment.get_dimensions()[1]))[1] - self.env_to_screen((0, 0))[1], 
            WHITE
        )
        
        if self.environment is not None:
            for wall in self.environment.get_walls():
                self.draw_walls(wall)
            for exit in self.environment.get_safety_exits():
                self.draw_exits(exit)
    
    def draw_walls(self, wall):
        w_starting_pos, w_ending_pos = wall
        draw_line(
            self.env_to_screen(w_starting_pos)[0], 
            self.env_to_screen(w_starting_pos)[1], 
            self.env_to_screen(w_ending_pos)[0], 
            self.env_to_screen(w_ending_pos)[1], 
            GRAY
        )
        
    def draw_exits(self, exit):
        e_starting_pos, e_ending_pos = exit
        draw_line(
            self.env_to_screen(e_starting_pos)[0], 
            self.env_to_screen(e_starting_pos)[1], 
            self.env_to_screen(e_ending_pos)[0], 
            self.env_to_screen(e_ending_pos)[1], 
            GREEN
        )
        
    def draw_agents(self):
        agents = self.environment.get_agents()
        if agents is None:
            return
        
        for agent in agents:
            a_pos = agent.get_position()
            draw_circle(
                int(self.env_to_screen((a_pos[0], a_pos[1]))[0]), 
                int(self.env_to_screen((a_pos[0], a_pos[1]))[1]), 
                max(2, int(3 * self.scale_env / 10)),  # radius scaled to environment size
                BLUE
            )
        
    def add_title(self):
        # the title is centered at the top of the window
        font_size = 30
        draw_text("CROWD SIMULATION IN EVACUATION", self.width // 2 - 300, BORDER, font_size, RED)
        self.top_border = BORDER + font_size + 30
        
    def add_legend(self): 
        lx_pos = BORDER
        ly_pos = self.env_to_screen((0,0))[1]
               
        draw_text("Legend:", lx_pos, ly_pos, 20, WHITE)
        draw_line(lx_pos, ly_pos + 50, lx_pos + 20, ly_pos + 50, GRAY)
        draw_text("Wall", lx_pos + 30, ly_pos + 40, 20, WHITE)
        draw_line(lx_pos, ly_pos + 80, lx_pos + 20, ly_pos + 80, GREEN)
        draw_text("Exit", lx_pos + 30, ly_pos + 70, 20, WHITE)
        
    def add_env_description(self):
        desc_x = 10
        desc_y = self.top_border
        
        draw_text("Environment Description:", desc_x, desc_y, 20, WHITE)
        draw_text(f"Dimensions: {self.environment.get_dimensions()}", desc_x, desc_y + 40, 20, WHITE)
        draw_text(f"Walls: {len(self.environment.get_walls())}", desc_x, desc_y + 70, 20, WHITE)
        draw_text(f"Exits: {len(self.environment.get_safety_exits())}", desc_x, desc_y + 100, 20, WHITE)
        
    def env_to_screen(self, env_position):
        starting_pos = (  # to center the environment in the window
            self.left_border + (self.width - self.right_border - self.left_border - self.environment.get_width() * self.scale_env) / 2,
            self.top_border + (self.height - self.bottom_border - self.top_border - self.environment.get_height() * self.scale_env) / 2 
        )
        screen_x = int(starting_pos[0] + env_position[0] * self.scale_env)
        screen_y = int(starting_pos[1] + env_position[1] * self.scale_env)
        return (screen_x, screen_y)
    
    def window_is_open(self):
        return not window_should_close()

    def close(self):
        close_window()
        self.on = False