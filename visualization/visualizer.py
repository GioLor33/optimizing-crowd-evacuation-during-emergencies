from pydoc import text
from pyray import *
from raylib import rl
from raylib import ffi
from environments.environment import Environment
from parser.config import Config
import math
import numpy as np

class Visualizer:
    def __init__(self, environment, config: Config):
        
        self.play = True
        self.btn_play_pos = None
        self.btn_pause_pos = None
        self.restart = False
        
        self.on = False
        self.width = 1200
        self.height = 700
        
        self.padding = 35
        self.right_border = 170
        self.left_border = 220
        self.bottom_border = 0
        self.top_border = 20

        self.background_color = config.background_color   
        self.exits_color = config.exit_color
        self.walls_color = config.wall_color
        self.text_color = config.text_color
        self.title_color = config.title_color
        
        self.algorithm = config.algorithm
        self.config = config
        
        self.hasGraph = False
        self.nodes = None
        self.aco_env = None
        self.btn_showGraph_pos = None
        self.show_graph = True
        
        self.show_grid = False
        self.btn_showGrid_pos = None
        
        self.show_pheromone_track = False
        self.btn_showPheromoneTrack_pos = None
        
        self.show_target_node = False
        self.btn_showTargetNode_pos = None
        
        self.show_fitness_map = True
        self.btn_showFitnessMap_pos = None
        
        assert isinstance(environment, Environment)
        self.define_environment(environment)
        
        init_window(self.width, self.height, self.config.window_name)
        
    def create_drawing(self):
        
        # Manage buttons
        mouse_pos = rl.GetMousePosition()
        if rl.IsMouseButtonPressed(0):
            if self.btn_showGrid_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_showGrid_pos):
                self.show_grid = not self.show_grid
            if self.btn_showGraph_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_showGraph_pos):
                self.show_graph = not self.show_graph
            if self.btn_play_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_play_pos):
                self.play = not self.play
            if self.btn_showPheromoneTrack_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_showPheromoneTrack_pos):
                self.show_pheromone_track = not self.show_pheromone_track
            if self.btn_showTargetNode_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_showTargetNode_pos):
                self.show_target_node = not self.show_target_node
            if self.btn_showFitnessMap_pos is not None and rl.CheckCollisionPointRec(mouse_pos, self.btn_showFitnessMap_pos):
                self.show_fitness_map = not self.show_fitness_map
            
        begin_drawing()
        
        clear_background(self.background_color)
        
        self.add_legend()
        
        # Add functions to show "moving" things on screen
        self.draw_environment()

        # Show buttons
        self.simulation_control_panel()
        self.display_buttons()
        
        if self.show_grid:
            self.draw_grid()

        # Show buttons
        self.simulation_control_panel()
        self.display_buttons()
        
        if self.show_grid:
            self.draw_grid()
        
        if self.hasGraph and self.show_graph:
            self.draw_graph()
        if self.hasGraph and self.show_graph:
            self.draw_graph()
            
        if self.hasGraph and self.show_pheromone_track:
            self.draw_acoPheromone_heatmap()
            
        if self.algorithm == "pso" and self.show_fitness_map:
            self.draw_fitness_map()
            
        self.draw_agents()
        self.add_env_description()        
        self.add_algorithm_info()
        
        self.add_title()
        
        end_drawing()
        
    def spawn_algorithm_loading(self):
        begin_drawing()
        
        clear_background(self.background_color)
        
        font_size = 40
        draw_text(
            f"Running {self.algorithm.upper()} algorithm offline ...",
            300,
            (self.height - font_size) // 2,
            font_size,
            YELLOW
        )
        draw_text(
            "Processing data for the simulation",
            430,
            (self.height - font_size) // 2 + 60,
            int(font_size * 0.6),
            WHITE
        )
        
        end_drawing()
        
    def define_environment(self, environment):
        self.environment = environment
        
        x, y = self.environment.get_dimensions()
        self.scale_env = min( 
                            (self.width - self.right_border - self.left_border - self.padding * 2) / x,
                            (self.height - self.bottom_border - self.top_border - self.padding * 2) / y
                            )
        
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

    
    def draw_agents(self):
        agents = self.environment.get_agents()
        if agents is None:
            return
        
        for agent in agents:
            a_pos = agent.get_position()
            a_pos_screen = self.env_to_screen((a_pos[0], a_pos[1]))
            
            if self.show_target_node:
                target_pos = agent.target
                if target_pos is not None:
                    x, y = self.env_to_screen((target_pos[0], target_pos[1]))
                    x = int(x)
                    y = int(y)
                    size = 4  # half-length of the cross
                    color_full = [color[0], color[1], color[2], 255]
                    
                    # Draw two lines forming an X
                    draw_line(x - size, y - size, x + size, y + size, color_full)
                    draw_line(x - size, y + size, x + size, y - size, color_full)
            
            if agent.fail:
                color = RED
                
            if self.algorithm == "pso" and self.show_fitness_map:
                color = WHITE
            else:
                color = agent.color
            
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

            
        
    def add_title(self):
        font_size = 27
        draw_rectangle(self.padding - 5, self.padding - 5, 300, font_size + 15, [0, 0, 0, 255])
        
        draw_text("CROWD EVACUATION", self.padding, self.padding, font_size, WHITE)
        draw_text("during", self.padding, self.padding + font_size + 5, font_size, WHITE)
        draw_text("EMERGENCY", self.padding, self.padding + 2 * (font_size + 5), font_size, RED)
        
        
    def add_legend(self): 
        env_pos = self.env_to_screen((self.environment.get_width(), 0))
        lx_pos = self.padding + env_pos[0]
        ly_pos = env_pos[1] #self.top_border + self.padding
        length_symbol = 20
        padding_length_symbol_and_text = 10
        y_after_first = 0
               
        draw_text("Legend:", lx_pos, ly_pos, 20, self.text_color)
        y_after_first += 40
        
        draw_circle(lx_pos + 5, ly_pos + y_after_first + 10, 5, YELLOW)
        draw_line(lx_pos + 5, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, YELLOW)
        draw_text("Agent", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
        y_after_first += 30
        
        if self.show_target_node:
            draw_line(lx_pos, ly_pos + y_after_first + 10 - length_symbol//2, lx_pos + length_symbol, ly_pos + y_after_first + 10 + length_symbol//2, YELLOW)
            draw_line(lx_pos, ly_pos + y_after_first + 10 + length_symbol//2, lx_pos + length_symbol, ly_pos + y_after_first + 10 - length_symbol//2, YELLOW)
            draw_text("Target Node", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
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
            grid_color = [200, 200, 200, 100]
            # horizontal lines
            draw_line(lx_pos, ly_pos + y_after_first, lx_pos + length_symbol, ly_pos + y_after_first, grid_color)
            draw_line(lx_pos, ly_pos + y_after_first + 20, lx_pos + length_symbol, ly_pos + y_after_first + 20, grid_color)
            # vertical lines
            draw_line(lx_pos, ly_pos + y_after_first, lx_pos, ly_pos + y_after_first + length_symbol, grid_color)
            draw_line(lx_pos + length_symbol, ly_pos + y_after_first, lx_pos + length_symbol, ly_pos + y_after_first + length_symbol, grid_color)
            draw_text("(1x1)m cell", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
        
        if self.hasGraph and self.show_graph:
            draw_line(lx_pos, ly_pos + y_after_first + 10, lx_pos + length_symbol, ly_pos + y_after_first + 10, [255, 105, 180, 100])
            draw_text("ACO Graph", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
            
        if self.hasGraph and self.show_pheromone_track:
            cell_width = length_symbol
            cell_height = 20  # or any height you like
            num_steps = cell_width  # one color per pixel horizontally

            for i in range(num_steps):
                t = i / (num_steps - 1)  # normalized 0..1
                color = self.colormap(t)
                # Draw a vertical line for each step to simulate gradient
                draw_line(
                    lx_pos + i, ly_pos + y_after_first,
                    lx_pos + i, ly_pos + y_after_first + cell_height,
                    color
                )
            
            draw_text("ACO Pheromone", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
            
        if self.algorithm == "pso" and self.show_fitness_map:
            cell_width = length_symbol
            cell_height = 20  # or any height you like
            num_steps = cell_width  # one color per pixel horizontally

            for i in range(num_steps):
                t = i / (num_steps - 1)  # normalized 0..1
                color = self.colormap(t)
                # Draw a vertical line for each step to simulate gradient
                draw_line(
                    lx_pos + i, ly_pos + y_after_first,
                    lx_pos + i, ly_pos + y_after_first + cell_height,
                    color
                )
            
            draw_text("PSO Fitness Map", lx_pos + length_symbol + padding_length_symbol_and_text, ly_pos + y_after_first, 20, self.text_color)
            y_after_first += 30
            
        
    def add_env_description(self):
        desc_x = self.padding - 20
        desc_y = self.top_border + 150
        
        draw_text(f"# Seed: {self.config.random_seed}", desc_x, desc_y, 20, self.text_color)
        desc_y += 60
        
        draw_text("Environment Details:", desc_x, desc_y, 20, self.text_color)
        draw_text(f"> Type: {self.config.world_type}", desc_x, desc_y + 30, 20, self.text_color)
        draw_text(f"> Safety exits: {len(self.environment.get_safety_exits())}", desc_x, desc_y + 60, 20, self.text_color)
        
        agents = len(self.environment.agents)
        if agents > self.environment.initial_agent_count * 0.5:
            color = RED
        elif agents > self.environment.initial_agent_count * 0.2:
            color = ORANGE
        elif agents == 0.0:
            color = GREEN
        else: 
            color = YELLOW
        draw_text(f"> Still in danger:       / {self.environment.initial_agent_count}", desc_x, desc_y + 90, 20, self.text_color)
        draw_text(f"{agents}", desc_x + 185, desc_y + 90 - 2, 24, color)
        

    def add_algorithm_info(self):
        desc_x = self.padding - 20
        desc_y = self.top_border + 150 + 90 + 60 * 2
        
        draw_text("Algorithm:", desc_x, desc_y, 20, self.text_color)
        draw_text(f"{self.algorithm.upper()}", desc_x + 120, desc_y, 20, YELLOW)
        
        if self.algorithm == "aco":
            draw_text(f"> Ants used: {self.config.num_ants}", desc_x, desc_y + 40, 20, self.text_color)
            draw_text(f"> Iterations: {self.config.num_iterations}", desc_x, desc_y + 70, 20, self.text_color)
            draw_text(f"> Alpha: {self.config.alpha}", desc_x, desc_y + 100, 20, self.text_color)
            draw_text(f"> Beta: {self.config.beta}", desc_x, desc_y + 130, 20, self.text_color)
            draw_text(f"> Evaporation rate: {self.config.evaporation_rate}", desc_x, desc_y + 160, 20, self.text_color)
            draw_text(f"> Graph type: {self.config.graph_type} ({self.config.n}x{self.config.m})", desc_x, desc_y + 190, 20, self.text_color)
        elif self.algorithm == "pso":
            draw_text(f"> Neighborhood radius: {self.config.neighborhood_radius}", desc_x, desc_y + 40, 20, self.text_color)
            draw_text(f"> Inertia weight: {self.config.W}", desc_x, desc_y + 70, 20, self.text_color)
            draw_text(f"> Cognitive weight: {self.config.C1}", desc_x, desc_y + 100, 20, self.text_color)
            draw_text(f"> Social weight: {self.config.C2}", desc_x, desc_y + 130, 20, self.text_color)
        elif self.algorithm == "boids":
            draw_text(f"> Separation weight: {self.config.weights['separate']}", desc_x, desc_y + 40, 20, self.text_color)
            draw_text(f"> Alignment weight: {self.config.weights['align']}", desc_x, desc_y + 70, 20, self.text_color)
            draw_text(f"> Cohesion weight: {self.config.weights['cohere']}", desc_x, desc_y + 100, 20, self.text_color)
            draw_text(f"> Seek weight: {self.config.weights['seek']}", desc_x, desc_y + 130, 20, self.text_color)
            draw_text(f"> Avoid weight: {self.config.weights['avoid']}", desc_x, desc_y + 160, 20, self.text_color) 
               
    def env_to_screen(self, env_position):
        starting_pos = (  # to center the environment in the window
            self.left_border + self.padding + (self.width - self.right_border - self.left_border - self.environment.get_width() * self.scale_env - self.padding * 2) / 2,
            self.top_border + self.padding + (self.height - self.bottom_border - self.top_border - self.environment.get_height() * self.scale_env - self.padding * 2) / 2 
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
                [255, 105, 180, 50]
            )
            for neighbor_id in node.edges.keys():
                neighbor_pos = self.nodes[neighbor_id].pos
                neighbor_screen = self.env_to_screen(neighbor_pos)
                draw_line(
                    int(node_screen[0]),
                    int(node_screen[1]),
                    int(neighbor_screen[0]),
                    int(neighbor_screen[1]),
                    [255, 105, 180, 25]
                )
                
    def simulation_control_panel(self):
        env_pos = self.env_to_screen((self.environment.get_width(), 0))
        lx_pos = self.padding + env_pos[0]
        ly_pos = env_pos[1] + 350
        btn_dim = 28
        text_dim = 20
        padding_in_between = 10
        
        draw_text("Simulation Controller:", lx_pos, ly_pos, text_dim, self.text_color)
        ly_pos += 40
        
        draw_text(f">", lx_pos, ly_pos, text_dim, self.text_color)
        lx_pos_curr = lx_pos + 20
        
        ly_pos_curr = ly_pos - (btn_dim - text_dim) // 2
        self.btn_play_pos = [lx_pos_curr, ly_pos_curr, btn_dim, btn_dim]
        if self.play:
            rl.DrawRectangleLinesEx(self.btn_play_pos, 1, RED)
            bar_width = btn_dim * 0.15
            bar_height = btn_dim * 0.6
            spacing_between_bars = bar_width * 0.95
            bar_margin = (btn_dim - (bar_width * 2 + spacing_between_bars)) / 2  # spacing between bars
            x0 = lx_pos_curr + bar_margin
            y0 = ly_pos_curr + (btn_dim - bar_height) / 2
            rl.DrawRectangle(
                int(x0), int(y0), int(bar_width), int(bar_height), RED
            )
            x1 = lx_pos_curr + bar_margin + bar_width + spacing_between_bars
            y1 = y0
            rl.DrawRectangle(
                int(x1), int(y1), int(bar_width), int(bar_height), RED
            )
        else:
            rl.DrawRectangleLinesEx(self.btn_play_pos, 1, GREEN)
            triangle_margin = btn_dim * 0.25  # padding inside button
            x0 = lx_pos_curr + triangle_margin
            y0 = ly_pos_curr + triangle_margin
            x1 = lx_pos_curr + triangle_margin
            y1 = ly_pos_curr + btn_dim - triangle_margin
            x2 = lx_pos_curr + btn_dim - triangle_margin
            y2 = ly_pos_curr + btn_dim / 2
            rl.DrawTriangle(
                ffi.new("Vector2 *", [x0, y0])[0],
                ffi.new("Vector2 *", [x1, y1])[0],
                ffi.new("Vector2 *", [x2, y2])[0],
                GREEN
            )
            
        lx_pos_curr += + btn_dim + padding_in_between   
        
        draw_text(f"Sim Time: {self.environment.simulation_time:.2f} s", lx_pos_curr, ly_pos, text_dim, self.text_color)
     
                
    def display_buttons(self):
        env_pos = self.env_to_screen((self.environment.get_width(), 0))
        lx_pos = self.padding + env_pos[0]
        ly_pos = env_pos[1] + 450 #self.top_border + self.padding
        btn_height = 40
        text_height = 15
        padding_in_between = 10
        
        draw_text("Show / Hide:", lx_pos, ly_pos, 20, self.text_color)
        ly_pos += 40
        
        color = WHITE if self.show_grid else GRAY
        self.btn_showGrid_pos = [lx_pos, ly_pos, 50, btn_height]
        rl.DrawText(b'Grid', lx_pos + 10, ly_pos + (btn_height - text_height)//2, text_height, color)
        rl.DrawRectangleLinesEx(self.btn_showGrid_pos, 1, color)
        lx_pos_current = lx_pos + 50 + padding_in_between
        
        if self.algorithm == "aco":
            color = WHITE if self.show_graph else GRAY
            self.btn_showGraph_pos = [lx_pos_current, ly_pos, 100, btn_height]
            rl.DrawText(b'ACO Graph', lx_pos_current + 10, ly_pos + (btn_height - text_height)//2, text_height, color)
            rl.DrawRectangleLinesEx(self.btn_showGraph_pos, 1, color)
            ly_pos += btn_height + padding_in_between
            lx_pos_current = lx_pos
            
            color = WHITE if self.show_pheromone_track else GRAY
            self.btn_showPheromoneTrack_pos = [lx_pos_current, ly_pos, 100, btn_height]
            rl.DrawText(b'Pheromone', lx_pos_current + 10, ly_pos + (btn_height - text_height)//2, text_height, color)
            rl.DrawRectangleLinesEx(self.btn_showPheromoneTrack_pos, 1, color)
            
            color = WHITE if self.show_target_node else GRAY
            lx_pos_current += 100 + padding_in_between
            self.btn_showTargetNode_pos = [lx_pos_current, ly_pos, 85, btn_height]
            rl.DrawText(b'Targets', lx_pos_current + 10, ly_pos + (btn_height - text_height)//2, text_height, color)
            rl.DrawRectangleLinesEx(self.btn_showTargetNode_pos, 1, color)
            
        if self.algorithm == "pso":
            color = WHITE if self.show_fitness_map else GRAY
            self.btn_showFitnessMap_pos = [lx_pos_current, ly_pos, 110, btn_height]
            rl.DrawText(b'Fitness Map', lx_pos_current + 10, ly_pos + (btn_height - text_height)//2, text_height, color)
            rl.DrawRectangleLinesEx(self.btn_showFitnessMap_pos, 1, color)
        
                
    def draw_acoPheromone_heatmap(self):
        if self.nodes is None:
            print("No graph nodes to draw.")
            return
        
        # let's cut out the lower pheromone levels to improve visualization
        max_pheromone = max(self.aco_env.pheromone.values())
        min_pheromone = min(self.aco_env.pheromone.values())
        pheromone_range = max_pheromone - min_pheromone if max_pheromone != min_pheromone else 1.0
                
        for edge_key, pheromone_level in self.aco_env.pheromone.items():
            
            if pheromone_level <= 1.0:
                continue
            
            node_ids = list(edge_key)
            node1 = self.nodes[node_ids[0]]
            node2 = self.nodes[node_ids[1]]
            node1_screen = self.env_to_screen(node1.pos)
            node2_screen = self.env_to_screen(node2.pos)
            
            # Normalize pheromone level to [0, 1]
            normalized_level = (pheromone_level - min_pheromone) / pheromone_range
            
            draw_line(
                int(node1_screen[0]),
                int(node1_screen[1]),
                int(node2_screen[0]),
                int(node2_screen[1]),
                self.colormap(normalized_level)
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

        return [r, g, b, 125]
    
    def draw_fitness_map(self):
        # size of each cell on screen
        fitnessGrid = self.environment.agents[0].fitness_map
        width, height = fitnessGrid.grid.shape
        
        env_width, env_height = self.environment.get_dimensions()
        cell_width = (self.scale_env * env_width) / width
        cell_height = (self.scale_env * env_height) / height

        for i in range(width):
            for j in range(height):
                color = self.distance_to_color(fitnessGrid.distance_map[i, j], distance_map=fitnessGrid.distance_map)
                # draw rectangle for this cell
                draw_rectangle(
                    int(i * cell_width) + self.env_to_screen((0, 0))[0],
                    int(j * cell_height) + self.env_to_screen((0, 0))[1],
                    int(cell_width),
                    int(cell_height),
                    color
                )
    
    def distance_to_color(self, d, vmin=None, vmax=None, distance_map=None):
        """Map a distance value to a color using normalized colormap, ignoring walls and infinities."""
        if not np.isfinite(d):
            return [200, 200, 200, 20]  # Light gray for finite distances

        if distance_map is not None:
            # Mask out walls (<0) and infinities
            valid = distance_map[(distance_map >= 0) & np.isfinite(distance_map)]
            if vmin is None:
                vmin = valid.min() if valid.size > 0 else 0
            if vmax is None:
                vmax = valid.max() if valid.size > 0 else 1
        else:
            if vmin is None:
                vmin = 0
            if vmax is None:
                vmax = max(1.0, d)

        # Avoid division by zero
        if vmax == vmin:
            t = 1.0
        else:
            t = (d - vmin) / (vmax - vmin)
            t = max(0.0, min(1.0, t))  # clamp

        # Use your colormap (or matplotlib viridis)
        r, g, b, a = self.colormap(t)
        return [r, g, b, 50]
    




