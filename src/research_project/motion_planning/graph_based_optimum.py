"""Graph-based shortest path planning for robot motion.

This module implements a shortest path algorithm for robot motion planning using
NetworkX directed graphs. It creates a multi-layered graph where each layer represents
a target point with multiple robot configurations, and edges represent transitions
between configurations. The shortest path is found using Dijkstra's algorithm.

The module supports both file-based initialization (loading collision-free solutions
from JSON) and direct initialization from in-memory solution data.

Example:
    From file:
        >>> path_builder = PathBuilder(data_path="/path/to/data")
        >>> path_builder.find_shortest_path_and_save(iterations=100)
    
    From solutions list:
        >>> solutions = [[config1_1, config1_2], [config2_1, config2_2]]
        >>> path_builder = PathBuilder.from_solutions(solutions, lift=0.05)
        >>> path, length = path_builder.find_shortest_path(iterations=50)
"""
import json
import networkx as nx
import numpy as np
import time
import random
import os
import concurrent.futures
import research_project.utilities.utils as utils

DATA_PATH = utils.get_data_path()


class PathBuilder:
    """Builds optimal paths through robot configurations using graph-based algorithms.
    
    The PathBuilder creates a directed graph where nodes represent robot configurations
    at specific target points, and edges represent transitions between configurations.
    It uses Dijkstra's algorithm to find the shortest path (minimum joint movement)
    from start to end.
    
    Attributes:
        start_time (float): Unix timestamp when pathbuilding started.
        logger (utils.Logger): Logger instance for tracking operations.
        data_path (str): Path to data directory containing solutions/metadata.
        metadata (dict): Metadata loaded from JSON file containing lift height etc.
        lift (float): Lift height to prepend to all configurations.
        points (list): List of lists containing robot configurations per target point.
        num_points (int): Number of target points (layers in graph).
        num_solutions (int): Total number of robot configurations across all points.
        graph (nx.DiGraph): NetworkX directed graph of configurations and transitions.
        shortest_path (list): List of node IDs representing the shortest path.
        shortest_path_length (float): Total distance/cost of the shortest path.
        output_path (str): File path where shortest path JSON will be saved.
    """
    
    def __init__(self, data_path=None, logging=True):
        """Initialize PathBuilder from files in data directory.
        
        Loads collision-free solutions and metadata from JSON files, creates
        the configuration graph, and prepares for shortest path calculation.
        
        Args:
            data_path (str, optional): Path to data directory. If None, uses
                default from utils.get_data_path().
            logging (bool, optional): Enable/disable logging. Defaults to True.
        
        Note:
            The shortest_path is initialized as None and must be computed by
            calling find_shortest_path() or find_shortest_path_and_save().
        """
        print(
            "Starting to build path at:",
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
        )
        self.start_time = time.time()
        self.logger = utils.Logger(data_path)
        if not logging:
            self.logger.logging = False
        if data_path is not None:
            self.data_path = data_path
            self.metadata = self.get_metadata()
            self.lift = self.metadata.get("lift_height", 0)
            self.points = self.load_collision_free_solutions()
            self.num_points = len(self.points)
            self.num_solutions = sum([len(point) for point in self.points])
            self.graph = self.create_graph()
        self.shortest_path, self.shortest_path_length = None, None  # self.find_shortest_path_and_save(iterations=100)  #num_points, iterations=100,)
        # self.build_path_to_json()

    def get_metadata(self):
        """Load metadata from most recent metadata JSON file.
        
        Searches for the most recent file matching pattern "*metadata*.json" in
        the auto_generated/export/ directory and loads its contents.
        
        Returns:
            dict: Metadata dictionary containing configuration parameters like
                'lift_height'. Returns empty dict if no metadata file found.
        """
        file_finder = utils.FileFinder(
            os.path.join(self.data_path, r"auto_generated/export/"), ".json", "metadata"
        )

        self.metadata_path = file_finder.get_file_by_date()
        if self.metadata_path is not None:

            with open(
                self.metadata_path,
                "r",
                encoding="utf-8",
            ) as metafile:
                return json.load(metafile)
        else:
            return {}

    @classmethod
    def from_solutions(cls, solutions, lift=0, data_path=utils.get_data_path(), logging=True):
        """Create PathBuilder instance from in-memory solutions, bypassing file I/O.
        
        Alternative constructor that accepts solutions directly rather than loading
        from files. Useful for programmatic path planning without intermediate JSON
        files. Immediately builds the configuration graph.
        
        Args:
            solutions (list): List of lists of robot configurations. Each outer list
                element represents one target point, containing inner lists of
                collision-free robot configurations (joint angles).
                Example: [[[j1, j2, j3], [j1', j2', j3']], [[j1, j2, j3]]]
            lift (float, optional): Lift height value to prepend to configurations
                when exporting. Defaults to 0.
            data_path (str, optional): Path for logging. Defaults to utils.get_data_path().
            logging (bool, optional): Enable/disable logging. Defaults to True.
        
        Returns:
            PathBuilder: Initialized PathBuilder instance with graph already created.
        
        Example:
            >>> solutions = [[[0, 0, 0], [0.1, 0.1, 0.1]], [[0.2, 0.2, 0.2]]]
            >>> builder = PathBuilder.from_solutions(solutions, lift=0.05)
            >>> path, length = builder.find_shortest_path(iterations=10)
        """
        self = cls.__new__(cls)
        if logging:
            self.logger = utils.Logger(data_path)
        # optional data_path if you ever need it for logging etc.
        self.data_path = data_path
        self.metadata = None
        self.lift = lift
        self.points = solutions
        self.num_points = len(solutions)
        self.num_solutions = sum(len(point) for point in solutions)

        # set up timing exactly as __init__ does
        self.start_time = time.time()

        # build the graph before any shortest‐path calls
        self.graph = self.create_graph()

        # placeholders for later
        self.shortest_path = None
        self.shortest_path_length = None

        return self

    def load_collision_free_solutions(self):
        """Load collision-free robot configurations from most recent JSON file.
        
        Finds the most recent file matching "*collision_free_solutions*.json" in
        the auto_generated/planned_motion/ directory. Also determines the output
        path for the shortest_path.json file based on the input filename.
        
        Returns:
            list: Nested list structure of collision-free solutions. Format:
                [[point0_config0, point0_config1, ...], [point1_config0, ...], ...]
                where each config is a list of joint angles.
        
        Side Effects:
            Sets self.output_path to the path where shortest path will be saved.
        """
        file_finder = utils.FileFinder(
            os.path.join(self.data_path, r"auto_generated/planned_motion/"),
            ".json",
            "collision_free_solutions",
        )
        collision_free_solutions_path = file_finder.get_file_by_date()
        filename = collision_free_solutions_path.split("collision_free_solutions")[0] + "shortest_path.json"
        filename = filename.split("/")[-1]
        output_path = os.path.join(os.path.dirname(self.data_path), f"data/auto_generated/planned_motion/{filename}")
        self.output_path = output_path
        with open(collision_free_solutions_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def create_graph(self):
        """Create directed graph of robot configurations and transitions.
        
        Builds a layered directed graph where:
        - Nodes represent robot configurations at specific target points
        - Node attributes include 'configuration' (joint angles) and 'point' (target index)
        - Edges connect configurations from consecutive points (no backwards edges)
        - Edge weights are L1-norm distances between configurations (sum of joint movements)
        
        Uses ThreadPoolExecutor for parallel edge creation between layers to improve
        performance for large solution sets.
        
        Returns:
            nx.DiGraph: Directed graph with nodes as configurations and weighted edges
                as transition costs.
        
        Side Effects:
            Logs and prints graph construction time and statistics.
        
        Note:
            Uses L1 norm (Manhattan distance) instead of L2 (Euclidean) to better
            represent total robot joint movement.
        """
        G = nx.DiGraph()
        # Load the data from collision_free_solutions.json
        points = self.points

        # Add nodes to the graph
        node_id = 0
        for point_index, configurations in enumerate(points):
            for config in configurations:
                G.add_node(node_id, configuration=config, point=point_index)
                node_id += 1

        # Function to calculate the weight (distance) between two configurations
        def calculate_weight(config1, config2):
            # return np.linalg.norm(np.array(config1) - np.array(config2))
            return np.linalg.norm(
                np.array(config1) - np.array(config2), ord=1
            )

        # Function to add edges between nodes
        def add_edges(G, current_rung, next_rung):
            for node1 in current_rung:
                for node2 in next_rung:
                    config1 = G.nodes[node1]["configuration"]
                    config2 = G.nodes[node2]["configuration"]
                    weight = calculate_weight(config1, config2)
                    G.add_edge(node1, node2, weight=weight)

        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = []
            for point_index in range(len(points) - 1):
                current_rung = [
                    n for n, attr in G.nodes(data=True) if attr["point"] == point_index
                ]
                next_rung = [
                    n
                    for n, attr in G.nodes(data=True)
                    if attr["point"] == point_index + 1
                ]
                futures.append(executor.submit(add_edges, G, current_rung, next_rung))
            concurrent.futures.wait(futures)
        graph_time = round(time.time() - self.start_time, 3)
        self.logger.log(
            f"Graph construction took {graph_time} seconds for {self.num_points} points with a total of {self.num_solutions} solutions"
        )
        print(f"Graph construction took {graph_time} seconds for {self.num_points} points with a total of {self.num_solutions} solutions")
        return G

    def find_shortest_path_and_save(self, iterations=None):
        """Find shortest path and immediately save to JSON file.
        
        Convenience method that combines find_shortest_path() and build_path_to_json().
        
        Args:
            iterations (int, optional): Number of random start/end pairs to try.
                If None, exhaustively tries all pairs (slower but guaranteed optimal).
                Defaults to None.
        
        Side Effects:
            Updates self.shortest_path and self.shortest_path_length, then writes
            configurations to output JSON file.
        """
        self.shortest_path, self.shortest_path_length = self.find_shortest_path(iterations=iterations)
        self.build_path_to_json()

    def find_shortest_path(self, iterations=None):
        """Find shortest path from first to last point using Dijkstra's algorithm.
        
        Searches for the shortest path from any configuration at the first target
        point to any configuration at the last target point. Can operate in two modes:
        
        1. Exhaustive mode (iterations=None): Tries all start/end pairs, guaranteed
           to find global optimum but slow for large solution sets.
        2. Sampling mode (iterations=N): Randomly samples N start/end pairs, faster
           but may miss global optimum.
        
        Args:
            iterations (int, optional): Number of random start/end pairs to try.
                If None, exhaustively tries all combinations. Defaults to None.
        
        Returns:
            tuple: (shortest_path, shortest_path_length) where:
                - shortest_path (list): List of node IDs in optimal order
                - shortest_path_length (float): Total cost (L1 distance) of path
        
        Side Effects:
            Logs and prints progress whenever a shorter path is found.
            Updates self.shortest_path with the best path found so far.
        
        Raises:
            nx.NetworkXNoPath: If no path exists between a start/end pair (silently
                caught and skipped).
        
        Example:
            >>> path, length = builder.find_shortest_path(iterations=100)
            >>> print(f"Found path with cost {length}")
        """
        all_shortest_paths = []
        _start_time = time.time()
        graph = self.graph
        print("started to find shortest path")
        # Find the shortest path from any node of the first point to any node of the last point
        first_point_nodes = [
            n for n, attr in graph.nodes(data=True) if attr["point"] == 0
        ]
        last_point_nodes = [
            n for n, attr in graph.nodes(data=True) if attr["point"] == self.num_points - 1
        ]

        shortest_path = None
        shortest_path_length = float("inf")
        if iterations is None:
            for i, start_node in enumerate(first_point_nodes):
                for j, end_node in enumerate(last_point_nodes):
                    current_runtime = round(time.time() - _start_time, 3)

                    try:
                        path_length = nx.dijkstra_path_length(
                            graph, start_node, end_node
                        )
                        if path_length < shortest_path_length:
                            shortest_path_length = path_length
                            shortest_path = nx.dijkstra_path(
                                graph, start_node, end_node
                            )
                            self.shortest_path = shortest_path
                            all_shortest_paths.append(
                                [shortest_path, shortest_path_length]
                            )
                            print(
                                f"found a shorter path at {current_runtime}, length: {shortest_path_length}, iteration: {i}/{j}"
                            )
                            self.logger.log(f"found a shorter path at {current_runtime}, length: {shortest_path_length}, iteration: {i}/{j}")
                    except nx.NetworkXNoPath:
                        continue
        else:

            for i in range(iterations):
                start_node = random.choice(first_point_nodes)
                end_node = random.choice(last_point_nodes)
                # for index,end_node in enumerate(last_point_nodes):
                current_runtime = round(time.time() - _start_time, 3)
                try:
                    path_length = nx.dijkstra_path_length(graph, start_node, end_node)
                    if path_length < shortest_path_length:
                        shortest_path_length = path_length
                        shortest_path = nx.dijkstra_path(graph, start_node, end_node)
                        self.shortest_path = shortest_path
                        all_shortest_paths.append([shortest_path, shortest_path_length])
                        print(
                            f"found a shorter path at {current_runtime}, length: {shortest_path_length}, iteration: {i}"
                        )
                        self.logger.log(
                            f"found a shorter path at {current_runtime}, length: {shortest_path_length}, iteration: {i}"
                        )

                except nx.NetworkXNoPath:
                    continue
        self.logger.log(f"Shortest path length: {shortest_path_length}")
        print("Shortest path length:", shortest_path_length)

        return shortest_path, shortest_path_length

    def build_path_to_json(self):
        """Export shortest path configurations to JSON file.
        
        Converts the shortest path (list of node IDs) to a list of robot
        configurations and saves to JSON. Prepends the lift height to each
        configuration as the first element.
        
        Returns:
            list: List of configurations with lift height prepended. Format:
                [[lift, j1, j2, j3, ...], [lift, j1', j2', j3', ...], ...]
        
        Side Effects:
            Writes JSON file to self.output_path.
            Prints confirmation message with save path.
        
        Note:
            Modifies configurations in-place by inserting lift at index 0.
        """
        shortest_path_configurations = []
        for node in self.shortest_path:
            current_configuration = self.graph.nodes[node]["configuration"]
            # add_lift to config
            current_configuration.insert(0, self.lift)
            shortest_path_configurations.append(current_configuration)
        with open(self.output_path, "w", encoding="utf-8") as f:
            json.dump(shortest_path_configurations, f)
        print(f"saved at: {self.output_path}")
        return shortest_path_configurations


if __name__ == "__main__":
    path_builder = None
    try:
        path_builder = PathBuilder(DATA_PATH)
        path_builder.shortest_path, path_builder.shortest_path_length = path_builder.find_shortest_path(iterations=100)
        path_builder.build_path_to_json()
    except KeyboardInterrupt:
        if path_builder.shortest_path:
            print("shortest_path available for saving")
            path_builder.build_path_to_json()
        print("Process interrupted. Current shortest path saved.")
