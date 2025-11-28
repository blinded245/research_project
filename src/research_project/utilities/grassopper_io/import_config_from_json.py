import os
import json
from compas_robots import Configuration
import research_project.utilities.utils as utils

data_path = utils.get_data_path()
logger = utils.Logger(data_path)


def import_configurations_from_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    configurations = [config for config in data]
    logger.log(f"{len(configurations)} configurations loaded from {file_path}")
    print(f"{len(configurations)} configurations loaded from {file_path}")
    return configurations


if __name__ == "__main__":
    # if not filename in locals():
    #     #list all vars in locals
    #     print("path not defined in GH")
    #     file_path = os.path.join(data_path, "auto_generated/best_individual.json")
    # else:
    #     if not filename =="" and not filename == None:
    #         file_path = os.path.join(data_path, filename)
    #     else:
    file_path = os.path.join(data_path, filename)  # noqa: F821  # "auto_generated/modified_shortest_path.json"

    configurations = import_configurations_from_json(file_path)
    if compas_configurations:  # noqa: F821
        print("compas_configurations")
        configurations = [Configuration.from_prismatic_and_revolute_values([config[0]], config[1:7]) for config in configurations]
        configuration = configurations[select_point]  # noqa: F821
    else:
        configuration = configurations[select_point]  # noqa: F821
        b = configurations[select_point][1]  # noqa: F821
        c = configurations[select_point][2]  # noqa: F821
        d = configurations[select_point][3]  # noqa: F821
        e = configurations[select_point][4]  # noqa: F821
        f = configurations[select_point][5]  # noqa: F821
        g = configurations[select_point][6]  # noqa: F821
