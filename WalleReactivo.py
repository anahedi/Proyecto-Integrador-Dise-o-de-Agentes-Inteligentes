import json
import random
from mesa import Agent, Model
from mesa.space import MultiGrid

class Walle_reactivo(Agent):
    """Agent that moves randomly until it reaches the goal."""

    def __init__(self, model, x, y, goalx, goaly):
        super().__init__(model)
        self.x = x
        self.y = y
        self.spawn_x = x  # Store initial position
        self.spawn_y = y
        self.goalx = goalx
        self.goaly = goaly
        self.steps_taken = 0
        self.path = []  # Stores movement history

    def step(self):
        """Move randomly in x or y direction, avoiding obstacles, until reaching goal."""
        if (self.x, self.y) == (self.goalx, self.goaly):
            print(f"Walle_reactivo reached the goal at ({self.x}, {self.y}) in {self.steps_taken} steps!")
            return

        moves = []
        grid_width, grid_height = self.model.grid.width, self.model.grid.height

        if self.x < grid_width - 1:
            moves.append((self.x + 1, self.y))
        if self.x > 0:
            moves.append((self.x - 1, self.y))
        if self.y < grid_height - 1:
            moves.append((self.x, self.y + 1))
        if self.y > 0:
            moves.append((self.x, self.y - 1))

        # Filter out moves that would land on an obstacle
        valid_moves = [move for move in moves if self.model.grid.is_cell_empty(move)]

        if valid_moves:
            new_x, new_y = random.choice(valid_moves)
            self.model.grid.move_agent(self, (new_x, new_y))
            self.x, self.y = new_x, new_y
            self.steps_taken += 1
            self.path.append({"x": self.x, "y": self.y})
            print(f"Walle_reactivo moved to ({self.x}, {self.y})")


class GridModel(Model):
    """A grid model containing reactive agents."""

    def __init__(self, width, height, goalx, goaly, obstacles):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)

        # Create and place the Walle_reactivo agent
        self.agent1 = Walle_reactivo(self, 0, 0, goalx, goaly)
        self.grid.place_agent(self.agent1, (0, 0))

        # Place obstacles
        for (x, y) in obstacles:
            self.grid.place_agent(Agent(self), (x, y))

    def step(self):
        """Advance the model by one step."""
        self.agent1.step()

    def save_log(self, filename="walle_log.json"):
        """Save movement history to a JSON file in the required format."""
        log_data = {
            "robots": [
                {
                    "spawnPosition": {
                        "x": self.agent1.spawn_x,
                        "y": self.agent1.spawn_y
                    },
                    "path": self.agent1.path
                }
            ]
        }
        with open(filename, "w") as file:
            json.dump(log_data, file, indent=4)
        print(f"Simulation log saved to {filename}")


def run_simulation(scenario_file):
    with open(scenario_file, "r") as file:
        scenario = json.load(file)

    width = scenario["width"]
    height = scenario["height"]
    start_x, start_y = scenario["start"]
    goal_x, goal_y = scenario["goal"]
    obstacles = scenario["obstacles"]

    model = GridModel(width, height, goal_x, goal_y, obstacles)

    # Continue simulation until goal is reached
    while (model.agent1.x, model.agent1.y) != (goal_x, goal_y):  
        model.step()

    unique_coordinates = set([(pos["x"], pos["y"]) for pos in model.agent1.path])
    print(f"Total steps taken: {model.agent1.steps_taken}")
    print(f"Unique coordinates traversed: {len(unique_coordinates)}")  

    model.save_log(f"reactive_path_{scenario_file.split('.')[0]}.json")


# Run simulations for each scenario
scenarios = ["world_5x5.json", "world_7x7.json", "world_10x10.json"]
for scenario in scenarios:
    run_simulation(scenario)