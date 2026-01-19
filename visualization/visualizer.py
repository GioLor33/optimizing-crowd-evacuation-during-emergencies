from pyray import *
from environments.environment import Environment
from parser.config import Config
import time
import math

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
        
        self.hasGraph = False
        self.nodes = None
        self.aco_env = None
        
        self.show_grid = True # TODO: read from config file
        
        assert isinstance(environment, Environment)
        self.define_environment(environment)
        
    def create_drawing(self):
        begin_drawing()
        
        clear_background(self.background_color)
        
        self.add_title()
        self.add_legend()
        
        # Add functions to show "moving" things on screen
        self.draw_environment()
        
        # if self.show_grid:
        #     self.draw_grid()
        
        # if self.hasGraph:
        #     # self.draw_graph()
            
        #     if self.aco_env.pheromone is not None:
        #         self.draw_acoPheromone_heatmap()
            
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
        if self.environment is not None:
            for wall in self.environment.get_walls():
                self.draw_wall(wall)
            for exit in self.environment.get_safety_exits():
                self.draw_exit_with_boundaries(exit)
    
    def draw_wall(self, wall):
        w_starting_pos, w_ending_pos = wall
        w_starting_pos_screen = self.env_to_screen(w_starting_pos)
        w_ending_pos_screen = self.env_to_screen(w_ending_pos)
        draw_line(
            w_starting_pos_screen[0], 
            w_starting_pos_screen[1], 
            w_ending_pos_screen[0], 
            w_ending_pos_screen[1], 
            self.walls_color
        )
        
    def draw_exit(self, exit):
        e_starting_pos, e_ending_pos = exit
        e_starting_pos_screen = self.env_to_screen(e_starting_pos)
        e_ending_pos_screen = self.env_to_screen(e_ending_pos)
        draw_line(
            e_starting_pos_screen[0], 
            e_starting_pos_screen[1], 
            e_ending_pos_screen[0], 
            e_ending_pos_screen[1], 
            self.exits_color
        )
    

    def draw_exit_with_boundaries(self, exit, boundary_length=4):
        (x1, y1), (x2, y2) = exit

        # Convert to screen coordinates
        x1s, y1s = self.env_to_screen((x1, y1))
        x2s, y2s = self.env_to_screen((x2, y2))

        # Draw main exit line
        draw_line(x1s, y1s, x2s, y2s, self.exits_color)

        # Compute perpendicular direction
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        px = -dy / length
        py = dx / length

        # Boundary length in screen space (optional: transform)
        B = boundary_length

        # Starting point boundaries
        sx1, sy1 = x1s + px * B, y1s + py * B
        sx2, sy2 = x1s - px * B, y1s - py * B
        draw_line(int(sx1), int(sy1), int(sx2), int(sy2), self.exits_color)

        # Ending point boundaries
        ex1, ey1 = x2s + px * B, y2s + py * B
        ex2, ey2 = x2s - px * B, y2s - py * B
        draw_line(int(ex1), int(ey1), int(ex2), int(ey2), self.exits_color)

        
        
    # def draw_agents(self):
    #     agents = self.environment.get_agents()
    #     if agents is None:
    #         return
        
    #     for agent in agents:
    #         a_pos = agent.get_position()
    #         a_pos_screen = self.env_to_screen((a_pos[0], a_pos[1]))
            
    #         draw_circle(
    #             int(a_pos_screen[0]), 
    #             int(a_pos_screen[1]), 
    #             agent.radius * self.scale_env,  # radius scaled to environment size
    #             [250, 250, 250, 35]
    #         )
            
    #         # if agent.path is not None:
    #         #     color = self.agents_color
    #         # else:
    #         #     color = RED
    #         color = self.agents_color
    #         draw_circle(
    #             int(a_pos_screen[0]), 
    #             int(a_pos_screen[1]), 
    #             2,  # radius scaled to environment size
    #             color
    #         )
            
    #         scale = 5
            
    #         velocity_screen = agent.vel * self.scale_env // scale
    #         draw_line(
    #             int(a_pos_screen[0]), 
    #             int(a_pos_screen[1]),
    #             int(a_pos_screen[0] + velocity_screen[0]),
    #             int(a_pos_screen[1] + velocity_screen[1]),
    #             color
    #         )
            
    #         # force_driving_screen = agent.f_desired * self.scale_env // scale
    #         # draw_line(
    #         #     int(a_pos_screen[0]), 
    #         #     int(a_pos_screen[1]),
    #         #     int(a_pos_screen[0] + force_driving_screen[0]),
    #         #     int(a_pos_screen[1] + force_driving_screen[1]),
    #         #     RED
    #         # )
            
    #         # force_agent_screen = agent.f_agents * self.scale_env // scale
    #         # draw_line(
    #         #     int(a_pos_screen[0]), 
    #         #     int(a_pos_screen[1]),
    #         #     int(a_pos_screen[0] + force_agent_screen[0]),
    #         #     int(a_pos_screen[1] + force_agent_screen[1]),
    #         #     ORANGE
    #         # )
            
    #         # force_wall_screen = agent.f_walls * self.scale_env // scale
    #         # draw_line(
    #         #     int(a_pos_screen[0]), 
    #         #     int(a_pos_screen[1]),
    #         #     int(a_pos_screen[0] + force_wall_screen[0]),
    #         #     int(a_pos_screen[1] + force_wall_screen[1]),
    #         #     BLUE
    #         # )
    
    def draw_agents(self):
        agents = self.environment.get_agents()
        if agents is None:
            return
        
        for agent in agents:
            a_pos = agent.get_position()
            a_pos_screen = self.env_to_screen((a_pos[0], a_pos[1]))
            color = agent.color
            
            if agent.fail:
                color = RED
            
            draw_circle(
                int(a_pos_screen[0]), 
                int(a_pos_screen[1]), 
                agent.radius * self.scale_env,  # radius scaled to environment size
                [color[0], color[1], color[2], 100]
            )
            
            draw_circle(
                int(a_pos_screen[0]), 
                int(a_pos_screen[1]), 
                2,  
                [color[0], color[1], color[2], 255]
            )
            
            scale = 5
            
            velocity_screen = agent.vel * self.scale_env // scale
            draw_line(
                int(a_pos_screen[0]), 
                int(a_pos_screen[1]),
                int(a_pos_screen[0] + velocity_screen[0]),
                int(a_pos_screen[1] + velocity_screen[1]),
                [color[0], color[1], color[2], 255]
            )
            
            # target_pos = agent.target
            # if target_pos is not None:
            #     target_pos_screen = self.env_to_screen((target_pos[0], target_pos[1]))
            #     draw_circle(
            #         int(target_pos_screen[0]),
            #         int(target_pos_screen[1]),
            #         4,
            #         [color[0], color[1], color[2], 255]
            #     )
            
        
    def add_title(self):
        # the title is centered at the top of the window
        font_size = 27
        draw_text("CROWD SIMULATION", self.padding, self.padding, font_size, WHITE)
        draw_text("during", self.padding, self.padding + font_size + 5, font_size, WHITE)
        draw_text("EVACUATION", self.padding, self.padding + 2 * (font_size + 5), font_size, RED)
        
    def add_legend(self): 
        env_pos = self.env_to_screen((self.environment.get_width(), 0))
        lx_pos = self.padding + env_pos[0]
        ly_pos = env_pos[1] #self.top_border + self.padding
        length_symbol = 20
        padding_length_symbol_and_text = 10
        y_after_first = 0
               
        draw_text("Legend:", lx_pos, ly_pos, 20, self.text_color)
        y_after_first += 40
        
        draw_circle(lx_pos + 5, ly_pos + y_after_first + 10, 5, self.agents_color)
        draw_line(lx_pos + 5, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, self.agents_color)
        draw_text("Agent", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
        y_after_first += 30
        
        draw_line(lx_pos, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, self.exits_color)
        draw_line(lx_pos, ly_pos + y_after_first + 10 - 5, lx_pos, ly_pos + y_after_first +10 + 5, self.exits_color)
        draw_line(lx_pos + length_symbol, ly_pos + y_after_first + 10 - 5, lx_pos + length_symbol, ly_pos + y_after_first +10 + 5, self.exits_color)
        draw_text("Exit", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
        y_after_first += 30
        
        draw_line(lx_pos, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, self.walls_color)
        draw_text("Wall", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
        y_after_first += 30
        
        if self.show_grid:
            grid_color = [200, 200, 200, 50]
            # horizontal lines
            draw_line(lx_pos, ly_pos + y_after_first, lx_pos + length_symbol, ly_pos + y_after_first, grid_color)
            draw_line(lx_pos, ly_pos + y_after_first + 20, lx_pos + length_symbol, ly_pos + y_after_first + 20, grid_color)
            # vertical lines
            draw_line(lx_pos, ly_pos + y_after_first, lx_pos, ly_pos + y_after_first + length_symbol, grid_color)
            draw_line(lx_pos + length_symbol, ly_pos + y_after_first, lx_pos + length_symbol, ly_pos + y_after_first + length_symbol, grid_color)
            draw_text("(1x1)m cell", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
        
        if self.hasGraph:
            draw_line(lx_pos, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, [255, 105, 180, 100])
            draw_text("ACO Graph", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
        
    def add_env_description(self):
        desc_x = self.padding
        desc_y = self.height - self.bottom_border + 20
        desc_x = self.padding
        desc_y = self.height - self.bottom_border + 20
        
        draw_text("Environment Details:", desc_x, desc_y, 20, self.text_color)
        draw_text(f"> Number of safety exits: {len(self.environment.get_safety_exits())}", desc_x, desc_y + 40, 20, self.text_color)
        draw_text(f"> Agents still in danger: {len(self.environment.get_agents())} / {self.environment.initial_agent_count}", desc_x, desc_y + 70, 20, self.text_color)
        
    def add_time_info(self):
        info_x = self.width - self.right_border - 150
        info_y = self.height - self.bottom_border + 20
        
        draw_text(f"Sim Time: {self.environment.simulation_time:.2f} s", info_x, info_y, 20, self.text_color)
    
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
        
    def associate_graph(self, aco_env):
        self.hasGraph = True
        self.aco_env = aco_env
        self.nodes = aco_env.nodes
        
    def remove_graph(self):
        self.hasGraph = False
        self.aco_env = None
        self.nodes = None
        
    def disable_graph(self):
        self.hasGraph = False
        
    def enable_graph(self):
        if self.nodes is not None:
            self.hasGraph = True
        else:
            print("No graph associated to visualizer. Cannot enable graph visualization. Call enable_graph() function.")
        
        
    def draw_grid(self):
        dim = self.environment.get_dimensions()
        # vertical lines
        for x in range(int(dim[0]) + 1):
            start_pos = self.env_to_screen((x, 0))
            end_pos = self.env_to_screen((x, dim[1]))
            draw_line(
                start_pos[0],
                start_pos[1],
                end_pos[0],
                end_pos[1],
                [200, 200, 200, 20]
            )
        # horizontal lines
        for y in range(int(dim[1]) + 1):
            start_pos = self.env_to_screen((0, y))
            end_pos = self.env_to_screen((dim[0], y))
            draw_line(
                start_pos[0],
                start_pos[1],
                end_pos[0],
                end_pos[1],
                [200, 200, 200, 20]
            )
        
    def draw_graph(self):
        if self.nodes is None:
            print("No graph nodes to draw.")
            return
        
        nodes_list = list(self.nodes.values())
            
        for node in nodes_list:
            node_screen = self.env_to_screen(node.pos)
            draw_circle(
                int(node_screen[0]),
                int(node_screen[1]),
                5,
                [255, 105, 180, 100]
            )
            for neighbor_id in node.edges.keys():
                neighbor_pos = self.nodes[neighbor_id].pos
                neighbor_screen = self.env_to_screen(neighbor_pos)
                draw_line(
                    int(node_screen[0]),
                    int(node_screen[1]),
                    int(neighbor_screen[0]),
                    int(neighbor_screen[1]),
                    [255, 105, 180, 50]
                )
                
    def draw_acoPheromone_heatmap(self):
        if self.nodes is None:
            print("No graph nodes to draw.")
            return
        
        
        # let's cut out the lower pheromone levels to improve visualization
        max_pheromone = max(self.aco_env.pheromone.values())
        min_pheromone = min(self.aco_env.pheromone.values())
        # threshold = min_pheromone + 0.1 * (max_pheromone - min_pheromone)
        # pheromone_visualizer = {k: v for k, v in self.aco_env.pheromone.items() if v >= threshold}
        # min_pheromone = threshold
        
        pheromone_visualizer = self.aco_env.pheromone
        
        pheromone_range = max_pheromone - min_pheromone if max_pheromone != min_pheromone else 1.0
            
        for edge_key, pheromone_level in pheromone_visualizer.items():
            node_ids = list(edge_key)
            node1 = self.nodes[node_ids[0]]
            node2 = self.nodes[node_ids[1]]
            node1_screen = self.env_to_screen(node1.pos)
            node2_screen = self.env_to_screen(node2.pos)
            
            # Normalize pheromone level to [0, 1]
            normalized_level = (pheromone_level - min_pheromone) / pheromone_range
            
            # Map to color (e.g., from blue to red)
            r = int(255 * normalized_level)
            g = 0
            b = int(255 * (1 - normalized_level))
            color = self.colormap(normalized_level)
            
            draw_line(
                int(node1_screen[0]),
                int(node1_screen[1]),
                int(node2_screen[0]),
                int(node2_screen[1]),
                color
            )
        
    def colormap(self, t):
        t = max(0.0, min(1.0, t))

        if t < 0.25:
            # blue → cyan
            u = t / 0.25
            r = 0
            g = int(255 * u)
            b = 255
        elif t < 0.5:
            # cyan → green
            u = (t - 0.25) / 0.25
            r = 0
            g = 255
            b = int(255 * (1 - u))
        elif t < 0.75:
            # green → yellow
            u = (t - 0.5) / 0.25
            r = int(255 * u)
            g = 255
            b = 0
        else:
            # yellow → red
            u = (t - 0.75) / 0.25
            r = 255
            g = int(255 * (1 - u))
            b = 0

        return [r, g, b, 220]
