#!/usr/bin/env python3

import fnmatch
import math
import os

from typing import List, Tuple


class Vec2f:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Vec2f(x={self.x}, y={self.y})"

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
        for r in ref:
            dist = (r - p).length()
            if dist <= threshold:
                out = False
            elif dist <= min_dist:
                min_dist = dist
        if out:
            ret.append((p, min_dist))

    return ret


def eval_file(file_path: str, ref: List[Vec2f]):
    with open(file_path, "r") as inf:
        data = inf.readlines()

    points = []

    for line in data:
        p = line.strip().split()
        points.append(Vec2f(float(p[0]), float(p[1])))

    paired = pair_with_ref(ref, points, threshold=0.5)
    out_of_range = points_out_of_range(ref, points, threshold=0.5)
    for p, d in out_of_range:
        print(f"{p}\tdist: {d}")


def main():
    if len(os.sys.argv) != 3:
        print(f"Usage: {os.sys.argv[0]} <REFERENCE> <INPUT_DIR>")
        exit(1)

    ref = []

    with open(os.sys.argv[1], "r") as inf:
        data = inf.readlines()

    data = list(filter(lambda x: x.lstrip().startswith("cylinder"), data))

    for d in data:
        d = d.split()
        ref.append(Vec2f(float(d[1]), float(d[2])))

    for path in files(os.sys.argv[2], "registered-*"):
        eval_file(path, ref)


if __name__ == "__main__":
    main()
