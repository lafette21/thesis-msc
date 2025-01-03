#!/usr/bin/env python3

import fnmatch
import math
import os
import re

from typing import List, Tuple


class Vec2f:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Vec2f(x={self.x}, y={self.y})"

    def __add__(self, other):
        return Vec2f(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2f(self.x - other.x, self.y - other.y)

    def length(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)


def files(folder: str, pattern: str = "*") -> list:
    # List all files and directories in the specified path
    entries = os.listdir(folder)

    # Filter out directories if you only want files
    files = [
        os.path.realpath(os.path.join(folder, f))
        for f in entries
        if os.path.isfile(os.path.join(folder, f)) and fnmatch.fnmatch(f, pattern)
    ]

    return files


def pair_with_ref(ref: List[Vec2f], points: List[Vec2f], threshold: float) -> List[Tuple[Vec2f, Vec2f]]:
    """
    Pair each point with reference points if they are within the threshold distance.

    :param ref: List of reference points.
    :param points: List of points to be paired.
    :param threshold: Maximum distance for pairing.
    :return: List of tuples where each tuple contains a reference point paired with a point from 'points'.
    """
    paired = []

    for r in ref:
        for p in points:
            dist = (r - p).length()
            if dist <= threshold:
                paired.append((r, p))
                #  print(f"({r.x}, {r.y})\t({p.x}, {p.y})\tdist: {dist}")

    return paired


def points_out_of_range(ref: List[Vec2f], points: List[Vec2f], threshold: float) -> List[Tuple[Vec2f, float]]:
    ret = []

    for p in points:
        out = True
        min_dist = float(math.inf)
        closest_r = None
        for r in ref:
            dist = (r - p).length()
            if dist <= threshold:
                out = False
            elif dist <= min_dist:
                min_dist = dist
                closest_r = r
        if out:
            ret.append((closest_r, p))

    return ret


def eval_file(file_path: str, ref: List[Vec2f], origin: Vec2f):
    print(f"Processing file: {file_path}")
    with open(file_path, "r") as inf:
        data = inf.readlines()

    points = []

    for line in data:
        p = line.strip().split()
        points.append(Vec2f(float(p[0]), float(p[1])) + origin)

    paired = pair_with_ref(ref, points, threshold=0.5)
    out_of_range = points_out_of_range(ref, points, threshold=0.5)

    return paired, out_of_range


def main():
    if len(os.sys.argv) != 4:
        print(f"Usage: {os.sys.argv[0]} <REFERENCE> <PATH> <INPUT_DIR>")
        exit(1)

    ref = []

    with open(os.sys.argv[1], "r") as inf:
        data = inf.readlines()

    data = list(filter(lambda x: x.lstrip().startswith("cylinder"), data))

    for d in data:
        d = d.split()
        ref.append(Vec2f(float(d[1]), float(d[2])))

    path = []

    with open(os.sys.argv[2], "r") as inf:
        data = inf.readlines()

    for d in data:
        d = d.split()
        path.append(Vec2f(float(d[0]), float(d[1])))

    origin = path[0]

    file_path_list = files(os.sys.argv[3], "registered-*")
    file_path_list.sort(key=lambda x: int(re.sub(".*registered-", "", x).rstrip(".xyz")))

    paired_list = []
    out_of_range_list = []

    dist_sum = lambda l: sum((r - p).length() for r, p in l)

    for file_path in file_path_list:
        paired, out_of_range = eval_file(file_path, ref, origin)
        if len(paired) > 0 or len(out_of_range) > 0:
            print(f"Success ratio: {len(paired) / (len(paired) + len(out_of_range))}\tFail ratio: {len(out_of_range) / (len(paired) + len(out_of_range))}")
        if len(paired) > 0:
            print(f"Avg in range distance: {dist_sum(paired) / len(paired)}")
        if len(out_of_range) > 0:
            print(f"Avg out of range distance: {dist_sum(out_of_range) / len(out_of_range)}")
        paired_list.append(paired)
        out_of_range_list.append(out_of_range)

    flat_paired = [item for sublist in paired_list for item in sublist]
    flat_out_of_range = [item for sublist in out_of_range_list for item in sublist]

    print(f"Number of points in range: {len(flat_paired)}\tAvg in range distance: {dist_sum(flat_paired) / len(flat_paired)}")
    print(f"Number of points out of range : {len(flat_out_of_range)}\tAvg out of range distance: {dist_sum(flat_out_of_range) / len(flat_out_of_range)}")
    print(f"Number of points: {len(flat_paired + flat_out_of_range)}\tAvg distance: {dist_sum(flat_paired + flat_out_of_range) / len(flat_paired + flat_out_of_range)}")

if __name__ == "__main__":
    main()
