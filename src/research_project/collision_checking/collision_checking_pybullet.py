"""
Collision Checking Module for Robot Configurations

Usage:
------
This module performs collision checking on robot configurations. To use this module,
you need to provide a JSON file containing robot configurations.

JSON File Structure:
--------------------
The collision checker expects a JSON file with robot configurations structured as a
3-layer nested list:

    1. Top layer: List of target frames (positions/poses the robot needs to reach)
    2. Second layer: List of all possible configurations (inverse kinematics solutions)
       that can reach each target frame
    3. Third layer: List of joint angles (typically 6 values for a 6-axis robot)
       representing a specific robot configuration

Example structure:
    [
        [  # First target frame
            [j1, j2, j3, j4, j5, j6],  # First configuration
            [j1, j2, j3, j4, j5, j6],  # Second configuration
            ...
        ],
        [  # Second target frame
            [j1, j2, j3, j4, j5, j6],  # First configuration
            ...
        ],
        ...
    ]

Example JSON file path:
    data/auto_generated/export/251117_163017_solutions.json

The JSON file path can be:
    - Automatically detected (most recent file) when creating a CollisionCheck object
    - Explicitly specified via the search_pattern_solutions parameter
    - Or provided in the __init__ method when most_recent_file=False
"""

import os
import json
import time
import research_project.collision_checking.pybullet_server as pybullet_server
import research_project.utilities.utils as utils

data_path = utils.get_data_path()
logger = utils.Logger(data_path)


class CollisionCheck:
    def __init__(self, _data_path, most_recent_file=True, search_pattern_solutions="solutions_selected", filename_metadata="metadata"):
        '''path to the data folder
        most_recent_file: if True, search for the most recent file matching the pattern provided in filename_solution and filename_metadata'''
        self.data_path = _data_path
        self.collision_free_solutions = []
        if most_recent_file:
            search_pattern = search_pattern_solutions
            path = os.path.join(_data_path, "auto_generated/export/")

            file_finder = utils.FileFinder(path, ".json", search_pattern)
            # file_finder = utils.FileFinder(path, ".json", "solutions")
            search_pattern_solutions = file_finder.get_file_by_date()
            self.solutions_path = os.path.join(path, f"{search_pattern_solutions}")

            file_finder = utils.FileFinder(path, ".json", "metadata")
            filename_metadata = file_finder.get_file_by_date()
            self.metadata_path = os.path.join(path, f"{filename_metadata}")
        else:
            self.solutions_path = os.path.join(path, f"{search_pattern_solutions}.json")
            self.metadata_path = os.path.join(path, f"{filename_metadata}.json")
        self.load_input()

    def load_input(self):
        with open(self.metadata_path, "r") as f:
            metadata = json.load(f)
        print(f"metadata read from {self.metadata_path}")
        logger.log(f"metadata read from {self.metadata_path}")
        spawnpoint = [metadata["robot_spawnpoint"][0], metadata["robot_spawnpoint"][1], metadata["robot_spawnpoint"][2]]
        self.spawnpoint = spawnpoint

        with open(self.solutions_path, "r") as f:
            self.solutions = json.load(f)
        print(f"solutions read from {self.solutions_path}")
        logger.log(f"solutions read from {self.solutions_path}")

        self.num_starting_solutions = sum([len(point) for point in self.solutions])

    @classmethod
    def from_solutions(cls, solutions):
        """
        Alternate constructor that initializes with a given list of solutions
        and a known spawnpoint, bypassing file I/O.
        """
        # 1) create instance without calling __init__
        self = cls.__new__(cls)
        self.data_path = utils.get_data_path()
        self.solutions_path = None  # No file path since we're using direct input
        # 2) set up attributes
        self.solutions = solutions
        self.spawnpoint = [0, 0, 0]  # Default spawnpoint; adjust as needed
        self.num_starting_solutions = sum(len(p) for p in solutions)
        self.collision_free_solutions = []

        return self

    def collision_check(self, write_output=False):
        server = pybullet_server.PybulletServer()  # debug="show_collision", gui=True) #debug=show_collision / show_no_collision gui=True
        urdf_path = os.path.join(self.data_path, "URDF/ur20_tool_90deg.urdf")
        # urdf_path = os.path.join(self.data_path, "URDF/ur20_tool_all_inputs_bottom_90deg.urdf")
        # _spawnpoint = [self.spawnpoint[0], self.spawnpoint[1], self.spawnpoint[2]]
        _spawnpoint = [0, 0, 0]
        print(f"Loading robot from {urdf_path} at {_spawnpoint}")
        server.load_robot(
            urdf_path,
            position=_spawnpoint,
            useFixedBase=False,
        )
        collision_path = os.path.join(self.data_path, "auto_generated/collision_temp/")
        server.load_collision_meshes(collision_path, [0, 0, 0])
        self.collision_free_solutions = server.cull_collisions(self.solutions)
        if write_output:
            filename = self.solutions_path.split("solutions")[0]
            filename = filename.split("/")[-1] + "collision_free_solutions.json"
            output_path = os.path.dirname(self.solutions_path)
            output_path = os.path.join(output_path, f"../planned_motion/{filename}")
            # output_path = self.solutions_path.split("solutions_reduced")[0] + "collision_free_solutions.json"
            with open(output_path, "w") as outfile:
                json.dump(self.collision_free_solutions, outfile)
        server.disconnect()


if __name__ == "__main__":
    starttime = time.time()
    print(
        "Starting collision check at:",
        time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
    )

    check = CollisionCheck(data_path)  # , search_pattern_solutions="solutions")
    check.collision_check(write_output=True)

    solutions = check.collision_free_solutions
    solutions_per_point = [len(point) for point in solutions]
    sum_of_all_solutions = sum(solutions_per_point)
    finish = time.time()
    logger.log("Collision Check for " + str(check.num_starting_solutions) + " Solutions in: " + str(round((finish - starttime), 2)) + " Seconds")
    print("\nFinished Collision Check for " + str(check.num_starting_solutions) + " Solutions in: " + str(round((finish - starttime), 2)) + " Seconds")
    logger.log("Starting Solutions: " + str(check.num_starting_solutions) + " reduced to " + str(sum_of_all_solutions) + " solutions")
    print(f"Starting Solutions: {str(check.num_starting_solutions)} reduced to {sum_of_all_solutions} solutions")

    empty_points_indices = [index for index, point in enumerate(solutions) if point == []]
    if empty_points_indices:
        raise ValueError(f"There are {len(empty_points_indices)} nodes without a valid configuration. Points affected: {empty_points_indices}")
    else:
        print("All points have a valid configuration")
