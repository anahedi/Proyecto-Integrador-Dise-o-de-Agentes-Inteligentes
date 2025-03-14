import heapq
import json
from mesa import Agent, Model
from mesa.space import MultiGrid

# Cargar datos desde el archivo JSON
with open("me\Proyecto integrador\escena1.json") as file:
    world_data = json.load(file)

grid_size = max(world_data["width"], world_data["height"])
start_x, start_y = world_data["start"]
goal_x, goal_y = world_data["goal"]
obstacles = set(tuple(obstacle) for obstacle in world_data["obstacles"])

class AStarAgent(Agent):
    def __init__(self, model, x, y, goalx, goaly, grid_size):
        super().__init__(model)
        self.x = x
        self.y = y
        self.spawn_x = x  # Posición inicial
        self.spawn_y = y
        self.goalx = goalx
        self.goaly = goaly
        self.grid_size = grid_size
        self.path = self.a_star()
        self.path_index = 0

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self):
        start = (self.x, self.y)
        goal = (self.goalx, self.goaly)

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append({"x": current[0], "y": current[1]})
                    current = came_from[current]
                path.reverse()
                return path

            neighbors = [
                (current[0] + dx, current[1] + dy)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            ]

            neighbors = [
                (x, y)
                for x, y in neighbors
                if 0 <= x < self.grid_size
                and 0 <= y < self.grid_size
                and (x, y) not in obstacles
            ]

            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def step(self):
        if self.path_index < len(self.path):
            new_x, new_y = self.path[self.path_index]["x"], self.path[self.path_index]["y"]
            self.model.grid.move_agent(self, (new_x, new_y))
            self.x, self.y = new_x, new_y
            self.path_index += 1

class AStarModel(Model):
    def __init__(self, grid_size):
        super().__init__()
        self.grid_size = grid_size
        self.grid = MultiGrid(self.grid_size, self.grid_size, torus=False)
        self.agent = AStarAgent(self, start_x, start_y, goal_x, goal_y, self.grid_size)
        self.grid.place_agent(self.agent, (start_x, start_y))

    def step(self):
        self.agent.step()

    def save_log(self, filename="astar_log.json"):
        log_data = {
            "robots": [
                {
                    "spawnPosition": {"x": self.agent.spawn_x, "y": self.agent.spawn_y},
                    "path": self.agent.path,
                }
            ]
        }
        with open(filename, "w") as file:
            json.dump(log_data, file, indent=4)
        print(f"Simulation log saved to {filename}")

model = AStarModel(grid_size)
for _ in range(20):
    model.step()
model.save_log()

