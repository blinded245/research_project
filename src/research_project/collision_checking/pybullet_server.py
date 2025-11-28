"""PyBullet simulation server for robot collision checking.

This module provides a wrapper around the PyBullet physics engine for robot
collision detection. It manages the PyBullet connection, loads robot URDF models
and collision meshes, and provides methods to check configurations for collisions.
"""
# afab Python 3.11.4
import pybullet as p
import os


class PybulletServer:
    """PyBullet simulation server for robot collision detection.
    
    This class manages a PyBullet physics simulation session, providing methods
    to load robot models, collision meshes, and check robot configurations for
    collisions with the environment.
    
    Attributes:
        physicsClient: PyBullet physics client instance.
        debug (bool): Enable debug visualization.
        GUI (bool): Enable GUI mode (visual window).
        collision_meshes (list): List of loaded collision mesh body IDs.
        robot: Robot body ID in PyBullet simulation.
    """
    
    def __init__(self, debug=False, gui=False):
        """Initialize the PyBullet server and connect to physics engine.
        
        Args:
            debug (bool, optional): Enable debug visualization features. Defaults to False.
            gui (bool, optional): Open PyBullet GUI window. Defaults to False.
        """
        self.physicsClient = None
        self.debug = debug
        self.GUI = gui
        self.connect()
        self.collision_meshes = []

    def load_collision_meshes(self, collision_path, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
        """Load collision meshes from OBJ files into the simulation.
        
        Scans the specified directory for OBJ files, creates collision shapes from them,
        and adds them as static bodies in the simulation environment.
        
        Args:
            collision_path (str): Path to directory containing numbered OBJ files
                (e.g., 0.obj, 1.obj, ...).
            position (list, optional): [x, y, z] position for mesh placement.
                Defaults to [0, 0, 0].
            orientation (list, optional): Quaternion [x, y, z, w] for mesh orientation.
                Defaults to [0, 0, 0, 1].
                
        Notes:
            - Only loads files ending with .obj
            - Files must be named with sequential integers (0.obj, 1.obj, etc.)
            - Meshes are loaded with unit scale [1, 1, 1]
        """
        # get the number of .obj files in the folder collision_path, do not include other filetypes like stl
        no_of_meshes = len([name for name in os.listdir(collision_path) if name.endswith(".obj")])
        self.collision_meshes = []  # Initialize the "mesh" list

        for i in range(no_of_meshes):
            self.collision_meshes.append(p.createCollisionShape(p.GEOM_MESH, fileName=collision_path + str(i) + ".obj", meshScale=[1, 1, 1]))
            self.collision_meshes[i] = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=self.collision_meshes[i], basePosition=position)
            p.resetBasePositionAndOrientation(self.collision_meshes[i], position, orientation)

    def connect(self):
        """Connect to PyBullet physics engine.
        
        Attempts to connect to PyBullet, preferring GUI mode if enabled, otherwise
        using DIRECT (headless) mode. Sets up gravity after connection.
        
        Notes:
            - Reuses existing connection if already connected
            - Falls back to DIRECT mode if GUI connection fails
            - Gravity is set to [0, 0, -9.81] m/s²
        """
        if self.physicsClient is not None:
            # Get connection information
            connection_info = p.getConnectionInfo(self.physicsClient)

            # Check if connected and if the mode is GUI
            if connection_info['isConnected'] == 1 and connection_info['connectionMethod'] == p.GUI:
                print("Server running. Connected to PyBullet with GUI.")
                return
            else:
                # If not connected or not in GUI mode, disconnect
                if connection_info['isConnected'] == 1:
                    p.disconnect(self.physicsClient)

        try:
            # Try to connect in GUI mode
            self.physicsClient = p
            if not self.debug and not self.GUI:
                self.physicsClient.connect(p.DIRECT)
            else:
                self.physicsClient.connect(p.GUI)
            print("Connected to PyBullet.")
        except p.error:
            # If GUI connection fails, fall back to DIRECT mode
            print("Failed to connect to PyBullet with GUI. Falling back to DIRECT mode.")
            if not self.debug and not self.GUI:
                self.physicsClient.connect(p.DIRECT)
            else:
                self.physicsClient.connect(p.GUI)
            print("Connected to PyBullet.")
        p.setGravity(0, 0, -9.81)

    def step(self):
        """Advance the physics simulation by one timestep."""
        p.stepSimulation()

    def load_robot(self, urdf_path, position=[0, 0, 0], orientation=[0, 0, 1, 0], useFixedBase=True):
        """Load a robot URDF model into the simulation.
        
        Args:
            urdf_path (str): Path to the robot URDF file.
            position (list, optional): [x, y, z] base position. Defaults to [0, 0, 0].
            orientation (list, optional): Quaternion [x, y, z, w] base orientation.
                Defaults to [0, 0, 1, 0].
            useFixedBase (bool, optional): If True, robot base is fixed in space.
                If False, base can move freely. Defaults to True.
                
        Returns:
            bool: True if robot loaded successfully.
        """
        self.robot = p.loadURDF(urdf_path, position, orientation, useFixedBase=useFixedBase)
        return True

    def single_collision_check(self, robot, mesh):
        """Check if robot is in collision with a single mesh.
        
        Args:
            robot: PyBullet body ID of the robot.
            mesh: PyBullet body ID of the collision mesh.
            
        Returns:
            bool: True if collision detected (distance < 0.0), False otherwise.
        """
        collision = p.getClosestPoints(robot, mesh, 0.0)
        if len(collision) > 0:
            return True

    def list_collision_check(self, robot, meshes):
        """Check if robot is in collision with any mesh in a list.
        
        Args:
            robot: PyBullet body ID of the robot.
            meshes: Single mesh body ID or list of mesh body IDs to check against.
            
        Returns:
            bool: True if collision detected with any mesh, False otherwise.
        """
        if not isinstance(meshes, list):
            return self.single_collision_check(robot, meshes)
        else:
            # if there are any collisions for mesh in meshes, return True
            if any(self.single_collision_check(robot, mesh) for mesh in meshes):
                return True

    def get_joint_names(self):
        """Get the names of all joints in the loaded robot.
        
        Returns:
            list: List of joint names as strings.
        """
        num_joints = p.getNumJoints(self.robot)
        joint_names = []
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot, joint_index)
            joint_name = joint_info[1].decode('utf-8')  # Joint name is the second element
            joint_names.append(joint_name)
        return joint_names

    def set_robot_configuration(self, joint_positions):
        """Set the robot to a specific joint configuration.
        
        Args:
            joint_positions (list): List of joint angles in radians.
                Joint indices start at 1 (skipping joint 0).
        """
        # add a zero to the beginning of the joint_positions list
        # joint_positions = [0] + joint_positions
        for i, joint_position in enumerate(joint_positions, start=1):
            self.physicsClient.resetJointState(self.robot, i, joint_position)

    def disconnect(self):
        """Disconnect from the PyBullet physics engine."""
        p.disconnect()

    def cull_collisions(self, points):
        """Filter out robot configurations that result in collisions.
        
        Tests each configuration in the provided list against loaded collision meshes,
        keeping only collision-free configurations.
        
        Args:
            points (list): List of points, where each point is a list of joint
                configurations to test.
                
        Returns:
            list: Filtered list with same structure, containing only collision-free
                configurations for each point.
                
        Notes:
            - Configurations are tested sequentially
            - Debug mode can visualize colliding/non-colliding configurations
        """
        collision_free_solutions = []
        for sel_point, solutions in enumerate(points):
            collision_free_configs = []
            for sel_config, joint_values in enumerate(solutions):
                self.set_robot_configuration(joint_values)
                colliding = self.list_collision_check(self.robot, self.collision_meshes)
                if self.debug == "show_both":
                    print(f"Point {sel_point}, Config {sel_config}: colliding? {colliding}")
                    input("Press Enter to continue...")
                if colliding:
                    if self.debug == "show_collision":
                        print(f"Point {sel_point}, Config {sel_config}: colliding? {colliding}")
                        input("Press Enter to continue...")
                    continue
                else:
                    if self.debug == "show_no_collision":
                        print(f"Point {sel_point}, Config {sel_config}: colliding? {colliding}")
                        input("Press Enter to continue...")
                    # TODO: do collision check extruder against arm
                    # TODO: do collision check arm against arm
                    collision_free_configs.append(joint_values)
            collision_free_solutions.append(collision_free_configs)

        return collision_free_solutions
