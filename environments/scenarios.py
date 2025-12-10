# FILE: environments/scenarios.py
import numpy as np
from environments.environment import Environment


class BottleneckEnvironment(Environment):
    def __init__(self):
        width = 50
        height = 50
        walls = [
            [(25, 0), (25, 20)],
            [(25, 30), (25, 50)]
        ]
        exits = [
            [(49, 12), (49, 15)]
        ]
        super().__init__("Bottleneck_Scenario", (width, height), walls, exits)
        self.add_external_walls()

    def get_ordered_spawn_positions(self, num_agents):
        positions = []
        cols = int(np.sqrt(num_agents)) + 1
        start_x = 2
        start_y = 2
        spacing = 1.5

        count = 0
        for i in range(cols * 2):
            for j in range(cols):
                if count >= num_agents:
                    return positions
                x = start_x + (j * spacing)
                y = start_y + (i * spacing)
                if self.check_is_position_free((x, y)):
                    positions.append((x, y))
                    count += 1
        return positions


class CenterSpawnTwoDoorsEnvironment(Environment):
    def __init__(self):
        self.width = 50
        self.height = 50

        walls = []

        door_size = 10
        mid_y = self.height / 2
        door_start_y = mid_y - (door_size / 2)
        door_end_y = mid_y + (door_size / 2)

        exits = [
            [(0, door_start_y), (0, door_end_y)],  # Left Exit
            [(self.width, door_start_y), (self.width, door_end_y)]  # Right Exit
        ]

        super().__init__("Two_Doors_Scenario", (self.width, self.height), walls, exits)
        self.add_external_walls()

    def get_ordered_spawn_positions(self, num_agents):
        positions = []
        side_len = int(np.sqrt(num_agents)) + 1
        spacing = 1.5

        grid_width = side_len * spacing
        grid_height = side_len * spacing

        start_x = (self.width / 2) - (grid_width / 2)
        start_y = (self.height / 2) - (grid_height / 2)

        count = 0
        for i in range(side_len):
            for j in range(side_len):
                if count >= num_agents:
                    return positions

                x = start_x + (j * spacing)
                y = start_y + (i * spacing)

                if self.check_is_position_free((x, y)):
                    positions.append((x, y))
                    count += 1
        return positions


class SlalomEnvironment(Environment):
    """
    Scenario: Zig-Zag path.
    - Wall 1 (x=16): Gap at TOP (y=0-10).
    - Wall 2 (x=33): Gap at BOTTOM (y=40-50).
    - Exit: Middle Right.
    - Spawn: Bottom Left (forces Up -> Down -> Center movement).
    """

    def __init__(self):
        width = 50
        height = 50

        walls = [
            [(16, 10), (16, 50)],  # Wall 1: Blocks bottom, free at top (y < 10)
            [(33, 0), (33, 40)]  # Wall 2: Blocks top, free at bottom (y > 40)
        ]

        # Single final exit on the right wall
        exits = [
            [(49, 20), (49, 30)]
        ]

        super().__init__("Slalom_Scenario", (width, height), walls, exits)
        self.add_external_walls()

    def get_ordered_spawn_positions(self, num_agents):
        positions = []
        # Create a rectangular grid of agents
        cols = int(np.sqrt(num_agents)) + 1
        spacing = 1.5

        # Start at Bottom-Left (x=2, y=48) and fill UPWARDS and RIGHT
        # This ensures they start behind the first wall's blocked section
        start_x = 2
        start_y = 48

        count = 0
        # We iterate enough times to cover the agents
        for i in range(cols * 3):
            for j in range(cols):
                if count >= num_agents:
                    return positions

                x = start_x + (j * spacing)
                y = start_y - (i * spacing)  # Subtract to move UP

                # Boundary check to ensure we don't spawn outside or in walls
                if x < 15 and y > 0:
                    if self.check_is_position_free((x, y)):
                        positions.append((x, y))
                        count += 1
        return positions


class EmptyRoomEnvironment(Environment):
    """
    Scenario: Easy empty room with 1 door.
    - Spawn: Centered on Left wall.
    - Exit: Centered on Right wall.
    """

    def __init__(self):
        self.width = 50
        self.height = 50
        walls = []  # No internal obstacles

        # One large exit centered on the right
        door_size = 12
        mid_y = self.height / 2
        exits = [
            [(self.width, mid_y - door_size / 2), (self.width, mid_y + door_size / 2)]
        ]

        super().__init__("Empty_Room_Scenario", (self.width, self.height), walls, exits)
        self.add_external_walls()

    def get_ordered_spawn_positions(self, num_agents):
        positions = []
        side_len = int(np.sqrt(num_agents)) + 1
        spacing = 1.5

        grid_height = side_len * spacing

        # Center the spawn block vertically
        start_x = 3
        start_y = (self.height / 2) - (grid_height / 2)

        count = 0
        for i in range(side_len):
            for j in range(side_len):
                if count >= num_agents:
                    return positions

                x = start_x + (j * spacing)
                y = start_y + (i * spacing)

                if self.check_is_position_free((x, y)):
                    positions.append((x, y))
                    count += 1
        return positions


def get_scenario_by_name(name):
    if name == "bottleneck":
        return BottleneckEnvironment()
    elif name == "two_doors":
        return CenterSpawnTwoDoorsEnvironment()
    elif name == "slalom":
        return SlalomEnvironment()
    elif name == "empty":
        return EmptyRoomEnvironment()
    return None