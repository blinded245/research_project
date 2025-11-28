import subprocess
import sys
import os




package_path = os.path.dirname(__file__)
package_path = os.path.abspath(os.path.join(package_path, "../../.."))


sys.path.append(os.path.abspath(os.path.join(package_path, "src")))

motion_planning_path = os.path.abspath(
    os.path.join(package_path, "src/research_project/motion_planning")
)
data_path = os.path.abspath(os.path.join(package_path, "data"))
collision_free_path = os.path.join(data_path, "auto_generated/collision_free_solutions.json")

# Define the paths to the scripts
collision_checking_script = os.path.join(package_path, "src/research_project/collision_checking/collision_checking_pybullet.py")
graph_based_optimum_script = os.path.join(motion_planning_path, "graph_based_optimum.py")
editing_script = os.path.join(package_path, "src/research_project/utilities/preprocessing/edit.py")


# Function to run a script
def run_script(script_path):
    result = subprocess.run([sys.executable, script_path], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error running {script_path}:")
        print(result.stderr)
    else:
        print(f"Output of {script_path}:")
        print(result.stdout)


# Run edit.py
print("Running edit.py... \n \n")
run_script(editing_script)

# Run collision_checking_pybullet.py
print("Running collision_checking_pybullet.py... \n \n")
run_script(collision_checking_script)

# Run dumb_fitness.py
print("Running graph_based_optimum.py to build a path... \n \n")
# run_script(dumb_fitness_script)
run_script(graph_based_optimum_script)
