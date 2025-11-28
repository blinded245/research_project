import math
import json
import os
# import blinded as dc
import research_project.utilities.utils as utils

def get_pose_in_jointspace(ur_ip="192.168.0.210", ur_port=30002, logging=True):
    try:
        starting_config_rad = dc.get_current_pose_joints(
            # "192.168.0.250", 50007, ur_ip, ur_port, True
            "192.168.0.42", 50007, ur_ip, ur_port, True
        )
        if starting_config_rad == "Timed out":
            print("Connection to robot timed out")
            if logging:
                logger.log("Connection to robot timed out")
            raise TimeoutError("Connection to robot timed out")
        print(f"Starting configuration received from robot: {starting_config_rad}")
    except Exception:
        # # elbow_up
        # starting_config_rad = [3.1766, -1.57423, 1.37977, -0.685916, 2.17626, -0.38855]

        # elbow_down
        starting_config_rad = [
            0.0590356,
            -2.88721,
            1.6065,
            -3.64044,
            -1.50903,
            -0.528057,
        ]

        # # elbow_up end cap
        # starting_config_rad = [
        #     0.718421,
        #     0.178418,
        #     0.527797,
        #     -1.154098,
        #     -0.903388,
        #     0.289046,
        # ]
        if logging:
            logger.log(
                f"!!! Could not connect to the robot, using {starting_config_rad} \n"
            )
        for _ in range(3):
            print(
                f"!!! Could not connect to the robot, using {starting_config_rad} !!! \n"
            )

    return starting_config_rad


def is_within_range(value, center, tol=10):
    """
    Check if the value is within the range of center ± tolerance degrees.

    :param value: The value to check (in degrees).
    :param center: The center value (default is 300 degrees).
    :param tolerance: The tolerance value (default is 10 degrees).
    :return: True if the value is within the range, False otherwise.
    """
    lower_bound = center - tol
    upper_bound = center + tol
    return lower_bound <= value <= upper_bound


def match_robot_pose(
    nested_solutions, tolerance=math.radians(120), starting_config_rad=None, joints=5
):
    """
    Match the robot pose with the starting configuration and filter out the solutions that are not within the tolerance."
    """
    if starting_config_rad is None:
        starting_config_rad = get_pose_in_jointspace()
    # Check each set of angles
    cleaned_solutions = []
    for i, point in enumerate(nested_solutions):
        cleaned_point = []
        for _, configuration in enumerate(point):
            # zipped = zip(angle_set, starting_config_rad)
            append = True
            for k in range(joints):
                angle = configuration[k]
                center = starting_config_rad[k]
                # print(f"checking: {math.degrees(angle)}, {math.degrees(center)}, point: {i}, conf: {j}, angle: {k}")
                if not is_within_range(angle, center, tol=tolerance):
                    # print(f"Point {i}, angle set {j}, config {k} has an angle out of range")
                    append = False
                if k == 2 and append:
                    cleaned_point.append(configuration)
        cleaned_solutions.append(cleaned_point)
        if len(cleaned_point) == 0:
            print(f"Point {i} has no solutions")
    return cleaned_solutions


def snap_angle(theta, theta_ref):
    # Compute the integer multiplier k that minimizes the difference to the reference angle.
    # Convert inputs to float in case they are strings
    theta = float(theta)
    theta_ref = float(theta_ref)
    k = round((theta_ref - theta) / (2 * math.pi))
    return theta + k * 2 * math.pi


def snap_configuration(candidate_config, reference_config):
    return [
        snap_angle(theta, theta_ref)
        for theta, theta_ref in zip(candidate_config, reference_config)
    ]


def snap_list_of_configurations(configurations, reference_config=None):
    if reference_config is None:
        reference_config = get_pose_in_jointspace()
    snapped_configurations = []
    for point in configurations:
        snapped_point = []
        for config in point:
            snapped_config = snap_configuration(config, reference_config)
            snapped_point.append(snapped_config)
        snapped_configurations.append(snapped_point)
    return snapped_configurations


if __name__ == "__main__":
    data_path = utils.get_data_path()
    current_dir = os.path.join(data_path, r"auto_generated/export/")
    export_file = utils.FileFinder(current_dir, ".json", "solutions").get_file_by_date()
    logger = utils.Logger(data_path)

    # Define the path to the JSON file
    output_file_path = export_file.replace("solutions.json", "solutions_selected.json")

    # Read the JSON file
    with open(export_file, "r") as open_file:
        print(f"reading {export_file}")
        data = json.load(open_file)

    current_robot_pose = get_pose_in_jointspace()

    snapped_solutions = snap_list_of_configurations(data, current_robot_pose)
    tolerance = math.radians(120)
    print(
        f"Snapping solutions to the current robot pose with a tolerance of {math.degrees(tolerance)} degrees"
    )
    logger.log(
        f"Snapping solutions to the current robot pose with a tolerance of {math.degrees(tolerance)} degrees"
    )

    matched_solutions = match_robot_pose(
        snapped_solutions, tolerance=tolerance, starting_config_rad=current_robot_pose
    )

    # Write the modified data back to a new JSON file
    with open(output_file_path, "w") as open_file:
        print(f"writing {output_file_path}")
        json.dump(matched_solutions, open_file, indent=4)

    sum_starting_solutions = sum([len(point) for point in data])
    sum_matched_solutions = sum([len(point) for point in matched_solutions])

    logger.log(f"Modified Data has been written to {output_file_path}")
    logger.log(
        f"Number of starting solutions: {sum_starting_solutions} reduced to {sum_matched_solutions} solutions"
    )
    print(f"Modified data has been written to {output_file_path}")
    print(
        f"Number of starting solutions: {sum_starting_solutions} reduced to {sum_matched_solutions} solutions"
    )
