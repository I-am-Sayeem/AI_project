import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import heapq
import math
import random
from enum import Enum
import time
import json
from collections import deque

class CellType(Enum):
    EMPTY = 0
    OBSTACLE = 1
    START = 2
    GOAL = 3
    PATH = 4
    VISITED = 5
    ROBOT = 6
    FRONTIER = 7
    CURRENT = 8

class Algorithm(Enum):
    ASTAR = "A*"
    BFS = "BFS"
    DFS = "DFS"
    UCS = "UCS"
    DLS = "DLS"

class WarehouseRobotPicker:
    def __init__(self, root):
        self.root = root
        self.root.title("Warehouse Robot Picker - Advanced Pathfinding")
        self.root.geometry("1200x800")
        
        # Grid parameters
        self.rows = 25
        self.cols = 35
        self.cell_size = 20
        self.grid = [[CellType.EMPTY for _ in range(self.cols)] for _ in range(self.rows)]
        
        # Robot parameters
        self.robot_pos = None
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.current_cell = None
        
        # Animation parameters
        self.animation_speed = 50  # ms between animation steps
        self.is_running = False
        self.show_search_process = True
        
        # Statistics
        self.stats = {
            "path_length": 0,
            "nodes_visited": 0,
            "nodes_explored": 0,
            "algorithm_time": 0,  # Pure algorithm execution time
            "total_time": 0,      # Total time including visualization
            "algorithm": ""
        }
        
        # Setup UI
        self.setup_ui()
        self.reset_grid()
        
    def setup_ui(self):
        # Main container
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel for controls
        left_panel = ttk.Frame(main_container, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_panel.pack_propagate(False)
        
        # Right panel for grid and stats
        right_panel = ttk.Frame(main_container)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # === Left Panel Controls ===
        
        # Algorithm selection
        algo_frame = ttk.LabelFrame(left_panel, text="Algorithm Selection", padding=10)
        algo_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.algorithm_var = tk.StringVar(value=Algorithm.ASTAR.value)
        algorithms = [algo.value for algo in Algorithm]
        
        for algo in algorithms:
            ttk.Radiobutton(algo_frame, text=algo, variable=self.algorithm_var, 
                           value=algo, command=self.on_algorithm_change).pack(anchor=tk.W)
        
        # DLS depth limit
        self.dls_frame = ttk.Frame(algo_frame)
        self.dls_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(self.dls_frame, text="Depth Limit:").pack(side=tk.LEFT)
        self.depth_limit_var = tk.IntVar(value=15)
        self.depth_limit_spinbox = ttk.Spinbox(self.dls_frame, from_=1, to=100, 
                                              textvariable=self.depth_limit_var, width=5)
        self.depth_limit_spinbox.pack(side=tk.LEFT, padx=(5, 0))
        
        # Grid controls
        grid_frame = ttk.LabelFrame(left_panel, text="Grid Controls", padding=10)
        grid_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(grid_frame, text="Find Path", command=self.find_path).pack(fill=tk.X, pady=2)
        ttk.Button(grid_frame, text="Reset Grid", command=self.reset_grid).pack(fill=tk.X, pady=2)
        ttk.Button(grid_frame, text="Random Obstacles", command=self.add_random_obstacles).pack(fill=tk.X, pady=2)
        ttk.Button(grid_frame, text="Clear Path", command=self.clear_path).pack(fill=tk.X, pady=2)
        ttk.Button(grid_frame, text="Random Start/Goal", command=self.random_start_goal).pack(fill=tk.X, pady=2)
        
        # Animation controls
        anim_frame = ttk.LabelFrame(left_panel, text="Animation Controls", padding=10)
        anim_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.show_search_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(anim_frame, text="Show Search Process", 
                       variable=self.show_search_var).pack(anchor=tk.W)
        
        ttk.Label(anim_frame, text="Animation Speed:").pack(anchor=tk.W, pady=(10, 0))
        self.speed_var = tk.IntVar(value=self.animation_speed)
        speed_scale = ttk.Scale(anim_frame, from_=10, to=200, variable=self.speed_var, 
                               orient=tk.HORIZONTAL, length=250)
        speed_scale.pack(fill=tk.X, pady=5)
        
        # File operations
        file_frame = ttk.LabelFrame(left_panel, text="File Operations", padding=10)
        file_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(file_frame, text="Save Layout", command=self.save_layout).pack(fill=tk.X, pady=2)
        ttk.Button(file_frame, text="Load Layout", command=self.load_layout).pack(fill=tk.X, pady=2)
        
        # Statistics
        stats_frame = ttk.LabelFrame(left_panel, text="Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.stats_text = tk.Text(stats_frame, height=10, width=30, wrap=tk.WORD)
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        
        # === Right Panel ===
        
        # Canvas for grid
        canvas_frame = ttk.Frame(right_panel)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(canvas_frame, width=self.cols * self.cell_size, 
                               height=self.rows * self.cell_size, bg="white", 
                               highlightthickness=1, highlightbackground="black")
        self.canvas.pack()
        
        # Mouse events
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        
        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        status_bar = ttk.Label(right_panel, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # Legend
        legend_frame = ttk.LabelFrame(right_panel, text="Legend", padding="5")
        legend_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        legend_items = [
            ("Empty", "white"),
            ("Obstacle", "black"),
            ("Start", "green"),
            ("Goal", "red"),
            ("Path", "blue"),
            ("Visited", "lightgray"),
            ("Frontier", "lightblue"),
            ("Current", "yellow"),
            ("Robot", "orange")
        ]
        
        for i, (text, color) in enumerate(legend_items):
            item_frame = ttk.Frame(legend_frame)
            item_frame.grid(row=0, column=i, padx=5)
            
            color_box = tk.Canvas(item_frame, width=15, height=15, bg=color, 
                                 highlightthickness=1, highlightbackground="black")
            color_box.pack(side=tk.LEFT, padx=(0, 5))
            
            ttk.Label(item_frame, text=text).pack(side=tk.LEFT)
        
        # Initialize DLS frame state
        self.on_algorithm_change()
    
    def on_algorithm_change(self):
        algorithm = self.algorithm_var.get()
        if algorithm == Algorithm.DLS.value:
            self.dls_frame.pack(fill=tk.X, pady=(10, 0))
        else:
            self.dls_frame.pack_forget()
    
    def reset_grid(self):
        self.grid = [[CellType.EMPTY for _ in range(self.cols)] for _ in range(self.rows)]
        self.start_pos = None
        self.goal_pos = None
        self.robot_pos = None
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.current_cell = None
        self.is_running = False
        
        self.draw_grid()
        self.update_stats()
        self.status_var.set("Grid reset. Click to set start position, then goal position")
    
    def add_random_obstacles(self, density=0.2):
        self.clear_path()
        
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == CellType.EMPTY and random.random() < density:
                    self.grid[i][j] = CellType.OBSTACLE
        
        self.draw_grid()
        self.status_var.set(f"Added random obstacles ({int(density*100)}% density)")
    
    def clear_path(self):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] in [CellType.PATH, CellType.VISITED, CellType.FRONTIER, CellType.CURRENT]:
                    self.grid[i][j] = CellType.EMPTY
        
        # Restore start and goal
        if self.start_pos:
            self.grid[self.start_pos[0]][self.start_pos[1]] = CellType.START
        if self.goal_pos:
            self.grid[self.goal_pos[0]][self.goal_pos[1]] = CellType.GOAL
        
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.current_cell = None
        self.robot_pos = None
        self.draw_grid()
        self.update_stats()
        self.status_var.set("Path cleared")
    
    def random_start_goal(self):
        self.clear_path()
        
        # Find empty cells
        empty_cells = [(i, j) for i in range(self.rows) for j in range(self.cols) 
                      if self.grid[i][j] == CellType.EMPTY]
        
        if len(empty_cells) < 2:
            messagebox.showwarning("Not Enough Space", "Need at least 2 empty cells for start and goal")
            return
        
        # Randomly select start and goal
        start_idx = random.randint(0, len(empty_cells) - 1)
        self.start_pos = empty_cells[start_idx]
        
        goal_idx = random.randint(0, len(empty_cells) - 1)
        while goal_idx == start_idx:
            goal_idx = random.randint(0, len(empty_cells) - 1)
        self.goal_pos = empty_cells[goal_idx]
        
        # Update grid
        self.grid[self.start_pos[0]][self.start_pos[1]] = CellType.START
        self.grid[self.goal_pos[0]][self.goal_pos[1]] = CellType.GOAL
        
        self.draw_grid()
        self.status_var.set("Random start and goal positions set")
    
    def save_layout(self):
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        # Convert grid to serializable format
        grid_data = []
        for i in range(self.rows):
            row = []
            for j in range(self.cols):
                row.append(self.grid[i][j].value)
            grid_data.append(row)
        
        layout = {
            "rows": self.rows,
            "cols": self.cols,
            "grid": grid_data,
            "start_pos": self.start_pos,
            "goal_pos": self.goal_pos
        }
        
        with open(file_path, 'w') as f:
            json.dump(layout, f)
        
        self.status_var.set(f"Layout saved to {file_path}")
    
    def load_layout(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        try:
            with open(file_path, 'r') as f:
                layout = json.load(f)
            
            self.rows = layout["rows"]
            self.cols = layout["cols"]
            
            # Load grid
            self.grid = []
            for i in range(self.rows):
                row = []
                for j in range(self.cols):
                    row.append(CellType(layout["grid"][i][j]))
                self.grid.append(row)
            
            self.start_pos = tuple(layout["start_pos"]) if layout["start_pos"] else None
            self.goal_pos = tuple(layout["goal_pos"]) if layout["goal_pos"] else None
            
            # Reset other variables
            self.robot_pos = None
            self.path = []
            self.visited_cells = set()
            self.frontier_cells = set()
            self.current_cell = None
            
            # Update canvas size
            self.canvas.config(width=self.cols * self.cell_size, height=self.rows * self.cell_size)
            
            self.draw_grid()
            self.status_var.set(f"Layout loaded from {file_path}")
        except Exception as e:
            messagebox.showerror("Load Error", f"Error loading layout: {str(e)}")
    
    def on_canvas_click(self, event):
        if self.is_running:
            return
            
        row = event.y // self.cell_size
        col = event.x // self.cell_size
        
        if 0 <= row < self.rows and 0 <= col < self.cols:
            if self.start_pos is None:
                self.start_pos = (row, col)
                self.grid[row][col] = CellType.START
                self.status_var.set("Start position set. Click to set goal position")
            elif self.goal_pos is None:
                self.goal_pos = (row, col)
                self.grid[row][col] = CellType.GOAL
                self.status_var.set("Goal position set. Click 'Find Path' to start")
            else:
                # Toggle obstacle
                if self.grid[row][col] == CellType.EMPTY:
                    self.grid[row][col] = CellType.OBSTACLE
                elif self.grid[row][col] == CellType.OBSTACLE:
                    self.grid[row][col] = CellType.EMPTY
            
            self.draw_grid()
    
    def on_canvas_drag(self, event):
        if self.is_running:
            return
            
        row = event.y // self.cell_size
        col = event.x // self.cell_size
        
        if 0 <= row < self.rows and 0 <= col < self.cols:
            if self.grid[row][col] == CellType.EMPTY:
                self.grid[row][col] = CellType.OBSTACLE
                self.draw_grid()
    
    def draw_grid(self):
        self.canvas.delete("all")
        
        for i in range(self.rows):
            for j in range(self.cols):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                color = self.get_cell_color(self.grid[i][j])
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
        
        # Draw robot if present
        if self.robot_pos:
            self.draw_robot(self.robot_pos[0], self.robot_pos[1])
    
    def get_cell_color(self, cell_type):
        colors = {
            CellType.EMPTY: "white",
            CellType.OBSTACLE: "black",
            CellType.START: "green",
            CellType.GOAL: "red",
            CellType.PATH: "blue",
            CellType.VISITED: "lightgray",
            CellType.ROBOT: "orange",
            CellType.FRONTIER: "lightblue",
            CellType.CURRENT: "yellow"
        }
        return colors.get(cell_type, "white")
    
    def draw_robot(self, row, col):
        x1 = col * self.cell_size + 2
        y1 = row * self.cell_size + 2
        x2 = x1 + self.cell_size - 4
        y2 = y1 + self.cell_size - 4
        
        self.canvas.create_oval(x1, y1, x2, y2, fill="orange", outline="darkorange", width=2)
    
    def find_path(self):
        if not self.start_pos or not self.goal_pos:
            messagebox.showwarning("Missing Positions", "Please set both start and goal positions")
            return
        
        if self.is_running:
            return
        
        self.clear_path()
        self.is_running = True
        self.status_var.set(f"Finding path using {self.algorithm_var.get()}...")
        
        # Run selected algorithm
        algorithm = self.algorithm_var.get()
        self.stats["algorithm"] = algorithm
        
        if algorithm == Algorithm.ASTAR.value:
            self.root.after(100, self.run_astar)
        elif algorithm == Algorithm.BFS.value:
            self.root.after(100, self.run_bfs)
        elif algorithm == Algorithm.DFS.value:
            self.root.after(100, self.run_dfs)
        elif algorithm == Algorithm.UCS.value:
            self.root.after(100, self.run_ucs)
        elif algorithm == Algorithm.DLS.value:
            self.root.after(100, self.run_dls)
    
    def run_astar(self):
        # Start timing
        total_start_time = time.time()
        
        # Initialize open and closed sets
        open_set = []
        closed_set = set()
        
        # Add start node
        heapq.heappush(open_set, (0, self.start_pos))
        self.frontier_cells.add(self.start_pos)
        
        # Track paths
        came_from = {}
        
        # Cost from start
        g_score = {self.start_pos: 0}
        
        # Estimated total cost
        f_score = {self.start_pos: self.heuristic(self.start_pos, self.goal_pos)}
        
        # Track visited cells for visualization
        self.visited_cells = set()
        self.stats["nodes_visited"] = 0
        self.stats["nodes_explored"] = 0
        
        # Algorithm execution (without visualization delays)
        algorithm_start_time = time.time()
        found_path = None
        
        while open_set:
            # Get node with lowest f_score
            current = heapq.heappop(open_set)[1]
            self.frontier_cells.discard(current)
            
            # Check if we reached the goal
            if current == self.goal_pos:
                found_path = self.reconstruct_path(came_from, current)
                break
            
            # Mark current as visited
            closed_set.add(current)
            self.visited_cells.add(current)
            self.stats["nodes_visited"] += 1
            
            # Check neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal_pos)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        self.frontier_cells.add(neighbor)
                        self.stats["nodes_explored"] += 1
        
        # Calculate algorithm execution time
        algorithm_end_time = time.time()
        self.stats["algorithm_time"] = algorithm_end_time - algorithm_start_time
        
        if found_path:
            # Animate the search process if enabled
            if self.show_search_var.get():
                self.animate_search_process(came_from, found_path[-1])
            else:
                # Directly show the path
                self.stats["path_length"] = len(found_path)
                self.stats["total_time"] = time.time() - total_start_time
                self.status_var.set(f"Path found! Length: {len(found_path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
                self.animate_path(came_from, found_path[-1])
        else:
            # No path found
            self.is_running = False
            self.stats["total_time"] = time.time() - total_start_time
            self.update_stats()
            self.status_var.set("No path found!")
            messagebox.showinfo("Path Not Found", "No path exists to the goal!")
    
    def run_bfs(self):
        # Start timing
        total_start_time = time.time()
        
        # Initialize queue and visited set
        queue = deque([self.start_pos])
        self.frontier_cells.add(self.start_pos)
        visited = set([self.start_pos])
        came_from = {}
        
        self.stats["nodes_visited"] = 0
        self.stats["nodes_explored"] = 1
        
        # Algorithm execution (without visualization delays)
        algorithm_start_time = time.time()
        found_path = None
        
        while queue:
            current = queue.popleft()
            self.frontier_cells.discard(current)
            self.stats["nodes_visited"] += 1
            
            # Check if we reached the goal
            if current == self.goal_pos:
                found_path = self.reconstruct_path(came_from, current)
                break
            
            # Check neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    queue.append(neighbor)
                    self.frontier_cells.add(neighbor)
                    self.stats["nodes_explored"] += 1
        
        # Calculate algorithm execution time
        algorithm_end_time = time.time()
        self.stats["algorithm_time"] = algorithm_end_time - algorithm_start_time
        
        if found_path:
            # Animate the search process if enabled
            if self.show_search_var.get():
                self.animate_search_process(came_from, found_path[-1])
            else:
                # Directly show the path
                self.stats["path_length"] = len(found_path)
                self.stats["total_time"] = time.time() - total_start_time
                self.status_var.set(f"Path found! Length: {len(found_path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
                self.animate_path(came_from, found_path[-1])
        else:
            # No path found
            self.is_running = False
            self.stats["total_time"] = time.time() - total_start_time
            self.update_stats()
            self.status_var.set("No path found!")
            messagebox.showinfo("Path Not Found", "No path exists to the goal!")
    
    def run_dfs(self):
        # Start timing
        total_start_time = time.time()
        
        # Initialize stack and visited set
        stack = [self.start_pos]
        self.frontier_cells.add(self.start_pos)
        visited = set()
        came_from = {}
        
        self.stats["nodes_visited"] = 0
        self.stats["nodes_explored"] = 1
        
        # Algorithm execution (without visualization delays)
        algorithm_start_time = time.time()
        found_path = None
        
        while stack:
            current = stack.pop()
            self.frontier_cells.discard(current)
            
            if current in visited:
                continue
                
            visited.add(current)
            self.stats["nodes_visited"] += 1
            
            # Check if we reached the goal
            if current == self.goal_pos:
                found_path = self.reconstruct_path(came_from, current)
                break
            
            # Check neighbors (in reverse order for DFS)
            neighbors = self.get_neighbors(current)
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    came_from[neighbor] = current
                    stack.append(neighbor)
                    self.frontier_cells.add(neighbor)
                    self.stats["nodes_explored"] += 1
        
        # Calculate algorithm execution time
        algorithm_end_time = time.time()
        self.stats["algorithm_time"] = algorithm_end_time - algorithm_start_time
        
        if found_path:
            # Animate the search process if enabled
            if self.show_search_var.get():
                self.animate_search_process(came_from, found_path[-1])
            else:
                # Directly show the path
                self.stats["path_length"] = len(found_path)
                self.stats["total_time"] = time.time() - total_start_time
                self.status_var.set(f"Path found! Length: {len(found_path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
                self.animate_path(came_from, found_path[-1])
        else:
            # No path found
            self.is_running = False
            self.stats["total_time"] = time.time() - total_start_time
            self.update_stats()
            self.status_var.set("No path found!")
            messagebox.showinfo("Path Not Found", "No path exists to the goal!")
    
    def run_ucs(self):
        # Start timing
        total_start_time = time.time()
        
        # Initialize priority queue and visited set
        open_set = []
        heapq.heappush(open_set, (0, self.start_pos))
        self.frontier_cells.add(self.start_pos)
        
        # Track paths
        came_from = {}
        
        # Cost from start
        g_score = {self.start_pos: 0}
        
        # Track visited cells for visualization
        self.visited_cells = set()
        self.stats["nodes_visited"] = 0
        self.stats["nodes_explored"] = 1
        
        # Algorithm execution (without visualization delays)
        algorithm_start_time = time.time()
        found_path = None
        
        while open_set:
            # Get node with lowest g_score
            current_cost, current = heapq.heappop(open_set)
            self.frontier_cells.discard(current)
            
            if current in self.visited_cells:
                continue
                
            self.visited_cells.add(current)
            self.stats["nodes_visited"] += 1
            
            # Check if we reached the goal
            if current == self.goal_pos:
                found_path = self.reconstruct_path(came_from, current)
                break
            
            # Check neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor in self.visited_cells:
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_set, (tentative_g_score, neighbor))
                    self.frontier_cells.add(neighbor)
                    self.stats["nodes_explored"] += 1
        
        # Calculate algorithm execution time
        algorithm_end_time = time.time()
        self.stats["algorithm_time"] = algorithm_end_time - algorithm_start_time
        
        if found_path:
            # Animate the search process if enabled
            if self.show_search_var.get():
                self.animate_search_process(came_from, found_path[-1])
            else:
                # Directly show the path
                self.stats["path_length"] = len(found_path)
                self.stats["total_time"] = time.time() - total_start_time
                self.status_var.set(f"Path found! Length: {len(found_path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
                self.animate_path(came_from, found_path[-1])
        else:
            # No path found
            self.is_running = False
            self.stats["total_time"] = time.time() - total_start_time
            self.update_stats()
            self.status_var.set("No path found!")
            messagebox.showinfo("Path Not Found", "No path exists to the goal!")
    
    def run_dls(self):
        depth_limit = self.depth_limit_var.get()
        # Start timing
        total_start_time = time.time()
        
        # Initialize stack and visited set
        stack = [(self.start_pos, 0)]  # (node, depth)
        self.frontier_cells.add(self.start_pos)
        visited = set()
        came_from = {}
        
        self.stats["nodes_visited"] = 0
        self.stats["nodes_explored"] = 1
        
        # Algorithm execution (without visualization delays)
        algorithm_start_time = time.time()
        found_path = None
        
        while stack:
            current, depth = stack.pop()
            self.frontier_cells.discard(current)
            
            if current in visited:
                continue
                
            visited.add(current)
            self.stats["nodes_visited"] += 1
            
            # Check if we reached the goal
            if current == self.goal_pos:
                found_path = self.reconstruct_path(came_from, current)
                break
            
            # Check depth limit
            if depth >= depth_limit:
                continue
            
            # Check neighbors (in reverse order for DFS)
            neighbors = self.get_neighbors(current)
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    came_from[neighbor] = current
                    stack.append((neighbor, depth + 1))
                    self.frontier_cells.add(neighbor)
                    self.stats["nodes_explored"] += 1
        
        # Calculate algorithm execution time
        algorithm_end_time = time.time()
        self.stats["algorithm_time"] = algorithm_end_time - algorithm_start_time
        
        if found_path:
            # Animate the search process if enabled
            if self.show_search_var.get():
                self.animate_search_process(came_from, found_path[-1])
            else:
                # Directly show the path
                self.stats["path_length"] = len(found_path)
                self.stats["total_time"] = time.time() - total_start_time
                self.status_var.set(f"Path found! Length: {len(found_path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
                self.animate_path(came_from, found_path[-1])
        else:
            # No path found
            self.is_running = False
            self.stats["total_time"] = time.time() - total_start_time
            self.update_stats()
            self.status_var.set(f"No path found within depth limit {depth_limit}!")
            messagebox.showinfo("Path Not Found", f"No path exists within depth limit {depth_limit}!")
    
    def animate_search_process(self, came_from, goal):
        # Create a list of visited cells in order for animation
        visited_order = list(self.visited_cells)
        self.animate_visited_index = 0
        
        # Start animation
        self.animate_search_step(came_from, goal, visited_order)
    
    def animate_search_step(self, came_from, goal, visited_order):
        if self.animate_visited_index < len(visited_order):
            # Show next visited cell
            cell = visited_order[self.animate_visited_index]
            if cell != self.start_pos and cell != self.goal_pos:
                self.grid[cell[0]][cell[1]] = CellType.VISITED
            
            # Show frontier cells
            for cell in self.frontier_cells:
                if cell != self.start_pos and cell != self.goal_pos:
                    self.grid[cell[0]][cell[1]] = CellType.FRONTIER
            
            self.draw_grid()
            self.animate_visited_index += 1
            
            # Continue animation
            self.root.after(max(10, self.speed_var.get() // 2), 
                          lambda: self.animate_search_step(came_from, goal, visited_order))
        else:
            # Animation complete, show the path
            path = self.reconstruct_path(came_from, goal)
            self.stats["path_length"] = len(path)
            self.stats["total_time"] = time.time() - time.time()  # Will be updated in animate_path
            self.status_var.set(f"Path found! Length: {len(path)} cells, Algorithm time: {self.stats['algorithm_time']:.6f}s")
            self.animate_path(came_from, goal)
    
    def update_search_visualization(self):
        # Clear previous visualization
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] in [CellType.VISITED, CellType.FRONTIER, CellType.CURRENT]:
                    if (i, j) == self.start_pos:
                        self.grid[i][j] = CellType.START
                    elif (i, j) == self.goal_pos:
                        self.grid[i][j] = CellType.GOAL
                    else:
                        self.grid[i][j] = CellType.EMPTY
        
        # Mark visited cells
        for cell in self.visited_cells:
            if cell != self.start_pos and cell != self.goal_pos:
                self.grid[cell[0]][cell[1]] = CellType.VISITED
        
        # Mark frontier cells
        for cell in self.frontier_cells:
            if cell != self.start_pos and cell != self.goal_pos:
                self.grid[cell[0]][cell[1]] = CellType.FRONTIER
        
        # Mark current cell
        if self.current_cell and self.current_cell != self.start_pos and self.current_cell != self.goal_pos:
            self.grid[self.current_cell[0]][self.current_cell[1]] = CellType.CURRENT
        
        self.draw_grid()
    
    def get_neighbors(self, pos):
        neighbors = []
        row, col = pos
        
        # Four directions: up, right, down, left
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            
            if (0 <= new_row < self.rows and 0 <= new_col < self.cols and 
                self.grid[new_row][new_col] != CellType.OBSTACLE):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def heuristic(self, pos1, pos2):
        # Manhattan distance
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def animate_path(self, came_from, current):
        path = self.reconstruct_path(came_from, current)
        self.path = path
        
        # Mark path cells
        for i, (row, col) in enumerate(path):
            if i > 0 and i < len(path) - 1:  # Skip start and goal
                self.grid[row][col] = CellType.PATH
        
        self.draw_grid()
        self.update_stats()
        
        # Animate robot movement
        self.robot_pos = self.start_pos
        self.draw_robot(self.robot_pos[0], self.robot_pos[1])
        
        self.move_robot_along_path(path)
    
    def move_robot_along_path(self, path):
        if not path:
            self.is_running = False
            return
        
        # Move to next position
        next_pos = path.pop(0)
        self.robot_pos = next_pos
        self.draw_robot(self.robot_pos[0], self.robot_pos[1])
        
        # Continue moving
        self.root.after(max(50, self.speed_var.get()), lambda: self.move_robot_along_path(path))

    
    def update_stats(self):
        stats_text = f"Algorithm: {self.stats['algorithm']}\n"
        stats_text += f"Path Length: {self.stats['path_length']}\n"
        stats_text += f"Nodes Visited: {self.stats['nodes_visited']}\n"
        stats_text += f"Nodes Explored: {self.stats['nodes_explored']}\n"
        stats_text += f"Algorithm Time: {self.stats['algorithm_time']:.6f}s\n"
        stats_text += f"Total Time: {self.stats['total_time']:.3f}s\n"
        
        # Add algorithm-specific info
        if self.stats['algorithm'] == Algorithm.DLS.value:
            stats_text += f"Depth Limit: {self.depth_limit_var.get()}\n"
        
        # Add optimality info
        if self.stats['path_length'] > 0:
            if self.stats['algorithm'] in [Algorithm.ASTAR.value, Algorithm.BFS.value, Algorithm.UCS.value]:
                stats_text += "Optimal: Yes\n"
            else:
                stats_text += "Optimal: No\n"
        
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(tk.END, stats_text)

def main():
    root = tk.Tk()
    app = WarehouseRobotPicker(root)
    root.mainloop()

if __name__ == "__main__":
    main()