from pyray import *
from environments.environment import Environment
from parser.config import Config
import time

# TODO: add buttons to start/pause/stop simulation

class Visualizer:
    def __init__(self, environment, config: Config):
        init_window(config.visualization_dimensions[0], config.visualization_dimensions[1], config.window_name)
        
        self.on = True
        self.width = config.visualization_dimensions[0]
        self.height = config.visualization_dimensions[1]
        
        self.padding = config.padding
        self.right_border = config.right_border
        self.left_border = config.left_border
        self.bottom_border = config.bottom_border
        self.top_border = config.top_border

        self.background_color = config.background_color   
        self.exits_color = config.exit_color
        self.walls_color = config.wall_color
        self.agents_color = config.agent_color
        self.text_color = config.text_color
        self.title_color = config.title_color
        
        assert isinstance(environment, Environment)
        self.define_environment(environment)
        
    def create_drawing(self):
        begin_drawing()
        
        clear_background(self.background_color)
        
        self.add_title()
        self.add_legend()
        
        # Add functions to show "moving" things on screen
        self.draw_environment()
        self.draw_agents()
        self.add_env_description()
        self.add_time_info()
        
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
            RED
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
            self.walls_color
        )
        
    def draw_exits(self, exit):
        e_starting_pos, e_ending_pos = exit
        draw_line(
            self.env_to_screen(e_starting_pos)[0], 
            self.env_to_screen(e_starting_pos)[1], 
            self.env_to_screen(e_ending_pos)[0], 
            self.env_to_screen(e_ending_pos)[1], 
            self.exits_color
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
                self.agents_color
            )
        
    def add_title(self):
        # the title is centered at the top of the window
        font_size = 30
        draw_text("CROWD SIMULATION IN EVACUATION", self.width // 2 - 300, self.padding, font_size, RED)
        self.top_border = self.padding + font_size + 30
        
    def add_legend(self): 
        lx_pos = self.padding
        ly_pos = self.env_to_screen((0,0))[1]
               
        draw_text("Legend:", lx_pos, ly_pos, 20, self.text_color)
        draw_line(lx_pos, ly_pos + 50, lx_pos + 20, ly_pos + 50, self.walls_color)
        draw_text("Wall", lx_pos + 30, ly_pos + 40, 20, self.text_color)
        draw_line(lx_pos, ly_pos + 80, lx_pos + 20, ly_pos + 80, self.exits_color)
        draw_text("Exit", lx_pos + 30, ly_pos + 70, 20, self.text_color)
        
    def add_env_description(self):
        desc_x = self.padding
        desc_y = self.height - self.bottom_border + 20
        
        draw_text("Environment Details:", desc_x, desc_y, 20, self.text_color)
        draw_text(f"> Number of safety exits: {len(self.environment.get_safety_exits())}", desc_x, desc_y + 40, 20, self.text_color)
        draw_text(f"> Agents still in danger: {len(self.environment.get_agents())} / {self.environment.initial_agent_count}", desc_x, desc_y + 70, 20, self.text_color)
        
    def add_time_info(self):
        info_x = self.width - self.right_border - 150
        info_y = self.height - self.bottom_border + 20
        
        #draw_text(f"Sim Time: {self.environment.simulation_time:.2f} s", info_x, info_y, 20, self.text_color)
    
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