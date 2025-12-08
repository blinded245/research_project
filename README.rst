============================================================
Research Project [BLINDED]
============================================================

.. start-badges

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
    :target: https://github.com/[BLINDED]/[BLINDED]/blob/master/LICENSE
    :alt: License MIT

.. image:: https://travis-ci.org/[BLINDED]/[BLINDED].svg?branch=master
    :target: https://travis-ci.org/[BLINDED]/[BLINDED]
    :alt: Travis CI

.. end-badges

Overview
--------

**Research Project [BLINDED]** is a Python-based framework designed for advanced motion planning and collision checking in robotic systems.


For the scope of this blinded repository, the functionality is limited to motion planning and collision checking for robotic arms. Other functionality like integration into design environments or fabrication planning and hardware control has been omitted, since they could not be provided in a standalone blinded form.

Main Features
-------------

* **motion Planning**: Plan efficient and collision-free movement for robotic arms.
* **Collision Checking**: Ensure that planned paths are free of collisions with the environment or other objects.

Usage
-----

Input Data
~~~~~~~~~~

The pipeline requires several types of input data in specific locations:

* **Configuration Solutions** - Robot joint angle configurations for each target pose
* **Metadata** - Information about solutions and target frames  
* **Collision Environment Meshes** - Obstacle and environment geometry file

• Configuration Solutions (Required)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The collision checker requires a JSON file containing robot configurations structured as a 3-layer nested list:

1. **Top layer**: List of target frames (positions/poses the robot needs to reach)
2. **Second layer**: All possible configurations (inverse kinematics solutions) for each target frame
3. **Third layer**: Joint angles (typically 6 values for a 6-axis robot) representing each configuration

**Example structure**::

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

**Expected location**: ``data/auto_generated/export/<timestamp>_solutions.json``

**Example**: ``data/auto_generated/export/251117_163017_solutions.json``

• Metadata (Optional)
^^^^^^^^^^^^^^^^^^^^

A JSON file containing metadata about the solutions and target frames.

**Expected location**: ``data/auto_generated/export/<timestamp>_metadata.json``

**Example**: ``data/auto_generated/export/251117_163017_metadata.json``

• Target planes (for reference) (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

JSON file containing plane definitions or reference geometry for the task.

**Expected location**: ``data/auto_generated/export/<timestamp>_planes.json``

**Example**: ``data/auto_generated/export/251117_163017_planes.json``

• Collision Environment Meshes (Required for collision checking)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mesh files defining obstacles and environment geometry for collision detection.

**Expected location**: ``data/auto_generated/collision_temp/``

**File formats**: ``.obj`` files

**Note**: These files are loaded by the collision checker and should represent walls, obstacles, and other environment boundaries that the robot must avoid.

• Complete Directory Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    data/
    ├── URDF/
    │   ├── ur20_tool_90deg.urdf          # Robot model with tool
    │   └── ur20/
    │       ├── visual/                   # Robot link visualizations
    │       │   ├── base.dae
    │       │   ├── shoulder.dae
    │       │   ├── upperarm.dae
    │       │   ├── forearm.dae
    │       │   ├── wrist1.dae
    │       │   ├── wrist2.dae
    │       │   └── wrist3.dae
    │       └── collision/                # Robot link collision meshes
    │           ├── base.stl
    │           ├── shoulder.stl
    │           ├── upperarm.stl
    │           ├── forearm.stl
    │           ├── wrist1.stl
    │           ├── wrist2.stl
    │           └── wrist3.stl
    │
    ├── tool_geometry/                    # Tool definition meshes
    │   ├── abele/
    │   │   └── split_90deg/
    │   │       ├── 250213_attachment.stl
    │   │       ├── 250213_clamp1.stl
    │   │       ├── 250213_clamp2.stl
    │   │       ├── 250213_clamp3.stl
    │   │       ├── 250213_motor.stl
    │   │       ├── 250213_90deg.stl
    │   │       └── ... (more tool parts)
    │   └── (other tool geometries)
    │
    └── auto_generated/
        ├── export/                       # Input configuration files (from Grasshopper)
        │   ├── <timestamp>_solutions.json
        │   ├── <timestamp>_metadata.json
        │   └── <timestamp>_planes.json
        │
        ├── collision_temp/               # Environment collision meshes
        │   ├── 0.obj
        │   ├── 1.obj
        │   └── ... (obstacle meshes)
        │
        └── planned_motion/               # Output motion planning results
            ├── <timestamp>_collision_free_solutions.json
            └── <timestamp>_shortest_path.json

Running the Complete Pipeline
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If your solutions JSON file is located at the expected path (``data/auto_generated/export/``), you can run the complete processing pipeline using::

    python src/research_project/utilities/run_all.py

This automated pipeline performs three sequential steps:

1. **Configuration Filtering** (``edit.py``)
   
   - Snaps joint angles to the nearest equivalent angle relative to the current robot pose
   - Applies a tolerance check (default: 120 degrees) to remove configurations too far from the reference
   - Outputs: ``<timestamp>_solutions_selected.json``

2. **Collision Checking** (``collision_checking_pybullet.py``)
   
   - Loads the filtered configurations
   - Uses PyBullet physics simulation to detect collisions
   - Removes configurations that would result in collisions with the environment
   - Outputs: ``data/auto_generated/planned_motion/<timestamp>_collision_free_solutions.json``

3. **Motion Planning** (``graph_based_optimum.py``)
   
   - Takes collision-free configurations
   - Plans optimal motion paths between target frames
   - Generates executable robot motion sequences

Running Individual Steps
~~~~~~~~~~~~~~~~~~~~~~~~~

You can also run each step individuall if the input files are located at the expected paths:

**Filter configurations**::

    python src/research_project/utilities/preprocessing/edit.py

**Collision checking only**::

    python src/research_project/collision_checking/collision_checking_pybullet.py

**Motion planning only**::

    python src/research_project/motion_planning/graph_based_optimum.py

Installation
------------

Quick Install from GitHub
~~~~~~~~~~~~~~~~~~~~~~~~~~

Install directly from GitHub without cloning::

    pip install git+https://github.com/blindmenow/research_project

Dependencies
~~~~~~~~~~~~

**Core dependencies** (automatically installed):

* numpy - Numerical computing
* networkx - Graph-based algorithms
* pybullet - Physics simulation for collision checking
* compas>=2.0 - Computational framework for architecture
* compas_robots - Robot modeling
* compas_fab - Fabrication planning


Contributing
------------

We welcome contributions to improve the project. To contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Make your changes and commit them with descriptive messages.
4. Push your changes to your fork.
5. Create a pull request to the main repository.

For detailed guidelines, refer to the `CONTRIBUTING.rst` file.

License
-------

This project is licensed under the MIT License. See the `LICENSE` file for more details.

Credits
-------

This project is maintained by [BLINDED]. Special thanks to all contributors and the open-source community.
