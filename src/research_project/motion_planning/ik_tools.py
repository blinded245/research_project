"""
plan_tree.py - A module for generating and manipulating robot motion plans.
"""

import inspect
import math
import time
import Rhino.Geometry as rg
from Rhino.Geometry import Plane, Transform, Point3d

from research_project.utilities.utils import dprint

import compas_rhino.conversions as conv

from compas.geometry import Transformation
from compas.geometry import Frame
from ur_fabrication_control.kinematics.ur_kinematics import inverse_kinematics
from ur_fabrication_control.kinematics.ur_params import ur_params
from compas_robots import Configuration


inputs = ["robot", "target_planes_WCS", "start_configuration", "group", "rotation"]


class IK_TOOLS:
    """A class for generating robot motion plans."""

    def free_rotation_planes(self, target_planes_wcs):
        target_planes_wcs_out = []
        if self.free_rotation == "False":
            target_planes_wcs_out = target_planes_wcs
            return target_planes_wcs_out

        if self.free_rotation == "n_steps":  # full rotation divided into n steps
            for plane in target_planes_wcs:
                transformed = []
                self.rotation_deg = 360 / self.rotation_steps
                for i in range(self.rotation_steps):
                    new_plane = rg.Plane(plane)
                    transformation_plus = Transform.Rotation(
                        i * self.rotation_deg * math.pi / 180,
                        new_plane.Normal,
                        new_plane.Origin,
                    )
                    new_plane.Transform(transformation_plus)
                    transformed.append(new_plane)
                target_planes_wcs_out.append(transformed)
            return target_planes_wcs_out
        if (
            self.free_rotation == "step_angle"
        ):  # rotation by a fixed angle starting from -angle_ccw to angle_cw
            for plane in target_planes_wcs:
                transformed = []
                for i in range(-1 * self.angle_ccw, self.angle_cw, self.rotation_deg):
                    # for i in range(-1 * self.rotation_steps, self.rotation_steps + 1):
                    new_plane = rg.Plane(plane)
                    transformation_plus = Transform.Rotation(
                        i * math.pi / 180,
                        new_plane.Normal,
                        new_plane.Origin,
                    )
                    new_plane.Transform(transformation_plus)
                    transformed.append(new_plane)
                target_planes_wcs_out.append(transformed)
            return target_planes_wcs_out

    def __init__(
        self,
        robot,
        target_planes_wcs,
        group=None,
        free_rotation="False",
        rotation_angle=5,
        rotation_steps=35,
        angle_cw=0,
        angle_ccw=0,
    ):

        self.robot = robot
        self.lift = robot.lift_height
        self.arm_base_frame = robot.RCF.translated([0, 0, self.lift])
        print("lift", self.lift)
        self.free_rotation = free_rotation
        self.rotation_steps = rotation_steps
        if self.free_rotation == "n_steps":
            self.rotation_deg = 360 / self.rotation_steps
        else:
            self.rotation_deg = rotation_angle
        self.angle_cw = angle_cw
        self.angle_ccw = angle_ccw
        self.target_planes_wcs = self.free_rotation_planes(target_planes_wcs)
        self.group = group

    def ik_analytical_solution(self, target_plane_wcs):
        """Does the cartesian Conversions and then calls the inverse kinematics
        function for a single target plane.

        Args:
            target_plane_WCS (Plane): The target plane in world coordinate system.

        Returns:
            A list of all possible solutions for the target plane.

        """
        if not self.check_requirements(["self.robot"]):
            return None
        # print("target_plane_wcs", target_plane_wcs)
        origin_point = Point3d(
            target_plane_wcs.OriginX, target_plane_wcs.OriginY, target_plane_wcs.OriginZ
        )
        frame_wcs = Frame(origin_point, target_plane_wcs.XAxis, target_plane_wcs.YAxis)
        # frame_rcs = self.robot.from_WCF_to_RCF(frame_wcs)
        frame_bcs = self.robot.from_WCF_to_BCF(frame_wcs)
        T = Transformation.from_frame_to_frame(self.arm_base_frame, self.robot.BCF)
        frame_rcs = frame_bcs.transformed(T)
        T2 = Transformation.from_frame_to_frame(self.robot.BCF, Frame.worldXY())
        frame_rcs_ = frame_rcs.transformed(T2)
        if self.robot.attached_tool:
            tool0_rcs = self.robot.from_tcf_to_t0cf([frame_rcs_])[0]
        else:
            tool0_rcs = frame_rcs
        # plane_tool0_RCS = draw_frame(tool0_RCS)
        # plane_RCS = draw_frame(frame_RCS)

        ur_params_specific = ur_params["ur20"]
        ik = inverse_kinematics(tool0_rcs, ur_params_specific)
        ik_shoulder_positive = self.check_shoulder_positive(ik)
        return ik_shoulder_positive

    def check_duplicate_solutions(self, solutions):
        """Checks for duplicate solutions in a list of joint values.

        Args:
            solutions (list): A list of joint values.

        Returns:
            A list of unique joint values.

        """
        # TODO: Make sure this function also finds duplicates that have round-off errors
        unique_solutions = []
        for solution in solutions:
            if solution not in unique_solutions:
                unique_solutions.append(solution)
        return unique_solutions

    def check_shoulder_positive(self, joint_values):
        updated_joint_values = []
        for values in joint_values:
            updated_values = list(values)  # Create a copy of the joint values
            if updated_values[0] < 0:
                updated_values[0] += 2 * math.pi
            updated_joint_values.append(updated_values)

        unique_solutions = self.check_duplicate_solutions(updated_joint_values)
        return unique_solutions

    def ik_moveit(self, plane, start_configuration=None, group=None):
        """Does the cartesian Conversions and then calls the inverse kinematics"""
        if not self.check_requirements(
            [
                "robot",
                "robot.client",
                "robot.client.is_connected",
                "plane",
            ],
            plane=plane,
        ):
            return None

        frame = conv.plane_to_compas_frame(plane)
        dprint(
            {"frame": frame, "start_configuration": start_configuration, "group": group}
        )
        configuration = self.robot.iter_inverse_kinematics(
            frame, start_configuration, group, options={"max_results": 100}
        )
        return configuration

    def ik_analytical_list_input(self, target_planes_wcs=None):
        """wrapper function for inverse_kinematics_all_solutions that takes a list of target planes
        as input and returns a list of solutions for each target plane."""
        if target_planes_wcs is None:
            target_planes_wcs = self.target_planes_wcs
        output = []
        if self.free_rotation == "n_steps" or self.free_rotation == "step_angle":
            for i, point in enumerate(self.target_planes_wcs):
                point_solutions = []
                for j, plane in enumerate(point):
                    point_solutions.extend(self.ik_analytical_solution(plane))
                output.append(point_solutions)

        else:
            if isinstance(target_planes_wcs, list):
                for plane in target_planes_wcs:
                    output.append(self.ik_analytical_solution(plane))
            elif isinstance(target_planes_wcs, Plane):
                output.append(self.ik_analytical_solution(target_planes_wcs))
        return output

    def check_requirements(self, checks, plane=None):
        for check in checks:
            if eval(check) is None:
                print("Check failed: " + check + "in " + inspect.stack()[1][3])
                return False
        return True


if __name__ == "<module>" or __name__ == "__main__":

    inst = IK_TOOLS(
        robot,  # noqa: F821
        target_planes_WCS,  # noqa: F821
        group,  # noqa: F821
        free_rotation=rotation,  # noqa: F821
        rotation_angle=angle_per_step,  # noqa: F821
        rotation_steps=steps,  # noqa: F821
        angle_cw=angle_clockwise,  # noqa: F821
        angle_ccw=angle_countercw,  # noqa: F821
    )

    # rotated_target_planes = th.list_to_tree(inst.free_rotation_planes(target_planes_WCS))
    planes = inst.target_planes_wcs
    flattened_planes = [plane for sublist in planes for plane in sublist]
    solutions_net = inst.ik_analytical_list_input()
    solutions = [
        [[joint_value for joint_value in item] for item in solution]
        for solution in solutions_net
    ]
    flattened_solutions = [solution for sublist in solutions for solution in sublist]
    flattened_solutions = [
        Configuration.from_prismatic_and_revolute_values([0], config[0:6])
        for config in flattened_solutions
    ]
    # sol=solutions[549][1]

    shape = [len(solutions_net[i]) for i in range(len(solutions_net))]
    # solutions = th.list_to_tree(solutions)
    print(shape)
    time_out = time.strftime("%H:%M:%S", time.localtime(time.time()))

unreachable = [pl for pl, num in zip(planes, shape) if num == 0]


# sol = ik_test(target_planes_WCS[549], robot)
