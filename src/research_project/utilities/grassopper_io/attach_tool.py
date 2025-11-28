'''Attach tool to robot end effector'''
import os
from compas.geometry import Frame, Vector
from compas.datastructures import Mesh
from compas_rhino.conversions import mesh_to_rhino
# from compas_ghpython.artists import MeshArtist
from compas_fab.robots import Tool

from mobile_robot_control.multitool import MultiTool
from compas_ghpython.drawing import draw_frame

filename_collision = None
multitool = False

if tool == 0:  # noqa: F821
    filename_visuals = 'measurement_tool.stl'
    ee_frame = Frame([0, 0, 0.109], [1, 0, 0], [0, 1, 0])

# elif tool == 1:
#     filename_visuals = 'abele/all_inputs_bottom/all_inputs_bottom_lq2_90deg.stl'
#     filename_collision = 'abele/all_inputs_bottom/all_inputs_bottom_lq2_90deg.stl'
#     tool_centerpoint = (0.000,-0.294,0.355) #Point at (0.000,-0.293,0.356)
#     tool_orientation_x = (0.0151,0,0)
#     tool_orientation_y = (0,-0.014,0.0056)
#     ee_frame = Frame(tool_centerpoint, tool_orientation_x, tool_orientation_y)

elif tool == 1:  # noqa: F821  # measurement-tool-tip-dc-ps
    filename_visuals = 'measurement_toolx3.stl'
    filename_collision = 'abele/split_90deg/250213_90deg_adapter.stl'

    pin_frame = Frame([0.000, 0.000, 0.29642], [1, 0, 0], [0, 1, 0])
    camera_frame = Frame([0.039977881576833754, 0.0818447722840289, 0.1611195120832291], [1, 0, 0], [0, 1, 0])
    scanner_frame = Frame([-0.0167, 0.0431, 0.3908], [1, 0, 0], [0, 1, 0])
    # camera frame is rotated 90 degrees around the y-axis and 90 degrees around its xaxis
    ros_camera_frame = Frame(camera_frame.point, [0, 1, 0], [0, 0, -1])
    ros_scanner_frame = Frame([scanner_frame.point.x, scanner_frame.point.y, scanner_frame.point.z - 0.046],
                              [0, 1, 0], [-1, 0, 0])

    tool_frames = {"pin": pin_frame, "camera": camera_frame, "scanner": scanner_frame}
    ee_frame = tool_frames.get("pin")
    multitool = True

# elif tool == 1:

# elif tool == 2:

# elif tool == 3:

# elif tool == 4:

elif tool == 5:  # noqa: F821
    filename_visuals = 'concrete_extruder_v1_10mm_0deg.stl'
    filename_collision = 'concrete_extruder_v1_10mm_0deg_collision.stl'
    tool_centerpoint = (0, -0.154355, 0.361564)
    tool_orientation_x = (0, -0.149169, 0.367692)
    tool_orientation_y = (1, 0.0, 0.0)
    vec = Vector.from_start_end(tool_centerpoint, tool_orientation_x)
    ee_frame = Frame(tool_centerpoint, tool_orientation_y, vec)

elif tool == 6:  # noqa: F821
    filename_visuals = 'earth_extruder_v1_short.stl'
    filename_collision = 'earth_extruder_v1_10mm_15deg_collision_simple_short.stl'
    tool_centerpoint = (-1.88102e-09, -0.404208, 0.133956)
    tool_orientation_x = (-1.46958e-18, -0.404208, 0.139956)
    tool_orientation_y = (1, 0, 0)
    vec = Vector.from_start_end(tool_centerpoint, tool_orientation_x)
    ee_frame = Frame(tool_centerpoint, tool_orientation_y, vec)

elif tool == 7:  # noqa: F821
    filename_visuals = 'abele/split_90deg/250515_90deg_adapter_straight_inlet.stl'
    filename_collision = 'abele/split_90deg/250515_90deg_adapter_straight_inlet.stl'
    # tool_centerpoint = (0,-0.2792,0.3515+0.043)
    tool_centerpoint = (0.00302, -0.29571, 0.40445)
    tool_orientation_x = (0.0366, 0, 0)
    tool_orientation_y = (0, 0.0051, 0.0141)
    ee_frame = Frame(tool_centerpoint, tool_orientation_x, tool_orientation_y)


elif tool == 8:  # noqa: F821  # abele
    # filename_visuals = 'abele/adjusted_length_flange_lq2_90deg.stl'
    # filename_collision = 'abele/adjusted_length_flange_lq2_90deg.stl'
    # filename_visuals = 'abele/split_90deg/250213_90deg_adapter.stl'
    # filename_collision = 'abele/split_90deg/250213_90deg_adapter.stl'

    filename_visuals = 'abele/split_90deg/250213_90deg_adapter_straight_inlet.stl'
    filename_collision = 'abele/split_90deg/250213_90deg_adapter_straight_inlet.stl'
    tool_centerpoint = (0.000, -0.29786, 0.361)  # Point at (0.000,-0.293,0.356).
    tool_orientation_x = (0.0151, 0, 0)
    tool_orientation_y = (0, -0.014, 0.0056)
    ee_frame = Frame(tool_centerpoint, tool_orientation_x, tool_orientation_y)

elif tool == 9:  # noqa: F821  # measurement-tool-abele-tip
    # filename_visuals = 'abele/adjusted_length_flange_lq2_90deg.stl'
    # filename_collision = 'abele/adjusted_length_flange_lq2_90deg.stl'
    filename_visuals = 'abele/split_90deg/250213_90deg_adapter.stl'
    filename_collision = 'abele/split_90deg/250213_90deg_adapter.stl'
    tool_centerpoint = (0, -0.30709, 0.36211)
    tool_orientation_x = (0.0781, 0, 0)
    tool_orientation_y = (0, -0.0298, 0.0127)
    ee_frame = Frame(tool_centerpoint, tool_orientation_x, tool_orientation_y)

toolnumbers = [0, "measurement_tool.stl", 1, "measurement_toolx3", 5, "concrete_extruder_v1_10mm_0deg.stl",
               6, "earth_extruder_v1_short.stl", 7, "abele/adjusted_length_flange_lq2_90deg.stl",
               8, "abele/adjusted_length_flange_lq2_90deg.stl", 9, "abele/adjusted_length_flange_lq2_90deg.stl"]

data_path = os.path.join(data_path, "tool_geometry")  # noqa: F821
fullfilename_visuals = os.path.join(data_path, filename_visuals)

print(fullfilename_visuals)

ee_mesh = Mesh.from_stl(fullfilename_visuals)

if filename_collision is not None:
    print("Collision mesh loaded")
    fullfilename_collision = os.path.join(data_path, filename_collision)
    collision_mesh = Mesh.from_stl(fullfilename_collision)
    if multitool:
        tool = MultiTool(ee_mesh, tool_frames, primary_tool_name=list(tool_frames.keys())[0], collision=collision_mesh)
    else:
        tool = Tool(ee_mesh, ee_frame, collision_mesh)
    if show:  # noqa: F821
        # collision = MeshArtist(collision_mesh).draw()
        collision = mesh_to_rhino(collision_mesh)
else:
    print("No collision mesh loaded")
    if multitool:
        tool = MultiTool(ee_mesh, tool_frames, primary_tool_name=list(tool_frames.keys())[0])
    else:
        tool = Tool(ee_mesh, ee_frame)

if show:  # noqa: F821
    # artist = MeshArtist(ee_mesh)
    # visuals = artist.draw()
    visuals = mesh_to_rhino(ee_mesh)


# scene = robot.client.get_planning_scene()
# for aco in scene.robot_state.attached_collision_objects:
#     for acm in aco.to_attached_collision_meshes():
#         frame_id = aco.object["header"]["frame_id"]
#         print(frame_id)

ee_plane = draw_frame(ee_frame)
print(ee_frame.quaternion.xyzw)
