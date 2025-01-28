import argparse
import pathlib
import os

import numpy as np

import mgrs
import lanelet2

from lanelet2.core import Lanelet, Point3d, LineString3d
from lanelet2.io import write
from lanelet2.projection import UtmProjector


def calculate_left_right_points(x, y, z, roll, pitch, yaw, lane_width=2.0):
    # Convert angles to radians (if they are in degrees)
    yaw_rad = yaw  # Assuming yaw is already in radians

    # Calculate the direction vectors for left and right
    # Left direction: perpendicular to the forward direction (x-axis)
    left_dx = -np.sin(yaw_rad) * lane_width
    left_dy = np.cos(yaw_rad) * lane_width

    # Right direction: opposite of left
    right_dx = -left_dx
    right_dy = -left_dy

    # Calculate the new points
    left_point = (x + left_dx, y + left_dy, z)
    right_point = (x + right_dx, y + right_dy, z)

    return left_point, right_point


def read_trajectory_file(file_path):
    pose_array = []
    with open(file_path, "r") as file:
        for line in file:
            current_pose = [float(x) for x in line.split()]
            pose_array.append(current_pose)
    return pose_array


def generate(
    input_trajectory_path, output_lanelet_path, mgrs_code, lane_width=2.0, interval=2
):
    pose_array = read_trajectory_file(input_trajectory_path)
    lanelet_map = lanelet2.core.LaneletMap()

    left_point, right_point = calculate_left_right_points(*pose_array[0], lane_width)
    prev_node_right = Point3d(
        lanelet2.core.getId(),
        right_point[0],
        right_point[1],
        right_point[2],
    )
    prev_node_left = Point3d(
        lanelet2.core.getId(),
        left_point[0],
        left_point[1],
        left_point[2],
    )

    for i in range(1, len(pose_array)):
        if i % interval != 0:
            continue
        left_point, right_point = calculate_left_right_points(
            *pose_array[i], lane_width
        )
        current_node_right = Point3d(
            lanelet2.core.getId(),
            right_point[0],
            right_point[1],
            right_point[2],
        )
        current_node_left = Point3d(
            lanelet2.core.getId(),
            left_point[0],
            left_point[1],
            left_point[2],
        )

        way_right = LineString3d(
            lanelet2.core.getId(), [prev_node_right, current_node_right]
        )
        way_left = LineString3d(
            lanelet2.core.getId(), [prev_node_left, current_node_left]
        )
        way_right.attributes["type"] = "line_thin"
        way_right.attributes["subtype"] = "solid"
        way_left.attributes["type"] = "line_thin"
        way_left.attributes["subtype"] = "solid"

        lanelet = Lanelet(lanelet2.core.getId(), way_left, way_right)
        lanelet.attributes["type"] = "lanelet"
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["speed_limit"] = "40"
        lanelet.attributes["location"] = "urban"
        lanelet.attributes["one_way"] = "yes"

        lanelet_map.add(lanelet)

        prev_node_right = current_node_right
        prev_node_left = current_node_left

    m = mgrs.MGRS()
    latlon = m.toLatLon(mgrs_code)

    projector = UtmProjector(lanelet2.io.Origin(latlon[0], latlon[1]))
    lanelet2.io.write(output_lanelet_path, lanelet_map, projector)


def lanelet_generator():
    parser = argparse.ArgumentParser(description="Create lanelet2 file from trajectory")
    parser.add_argument("input_txt", help="input txt stored trajectory")
    parser.add_argument("output_lanelet", help="output lanelet2 save path")
    parser.add_argument("-m", "--mgrs_code", help="MGRS code")
    parser.add_argument(
        "-l", "--lane_width", type=float, default=2.0, help="lane width[m]"
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=int,
        default=2,
        help="interval between trajectory points",
    )

    args = parser.parse_args()

    if not pathlib.Path(args.input_txt).exists():
        raise FileNotFoundError(f"Input txt file '{args.input_txt}' is not found.")
    if pathlib.Path(args.output_lanelet).exists():
        os.remove(args.output_lanelet)

    generate(
        args.input_txt,
        args.output_lanelet,
        args.mgrs_code,
        args.lane_width,
        args.interval,
    )


if __name__ == "__main__":
    lanelet_generator()
