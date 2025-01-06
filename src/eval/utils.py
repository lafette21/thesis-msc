import fnmatch
import itertools
import math
import os
import re
import shlex
import subprocess

from typing import List, Tuple

from vec import Vec2f


def eval_file(file_path: str, ref: List[Vec2f], origin: Vec2f, threshold: float):
    with open(file_path, "r") as inf:
        data = inf.readlines()

    points = []

    for line in data:
        p = line.strip().split()
        points.append(Vec2f(float(p[0]), float(p[1])) + origin)

    paired = pair_with_ref(ref, points, threshold)
    out_of_range = points_out_of_range(ref, points, threshold)

    return paired, out_of_range


def eval_scenario(dir, origin, ref_points, logger):
    file_path_list = files(dir, "registered-*")
    file_path_list.sort(key=lambda x: int(re.sub(".*registered-", "", x).rstrip(".xyz")))
    logger.debug(f"Files to process: {file_path_list}")

    indices = []
    paired_list = []
    out_of_range_list = []

    for file_path in file_path_list:
        logger.info(f"Processing file: {file_path}")
        paired, out_of_range = eval_file(file_path, ref_points, origin, threshold=0.5)
        paired_list.append(paired)
        out_of_range_list.append(out_of_range)

    return file_path_list, paired_list, out_of_range_list


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


def generate_configs(s: str, d: dict):
    def generate_all_combinations(d: dict) -> list:
        ret = []

        cartesian_prod = list(itertools.product(*d.values()))

        for elem in cartesian_prod:
            dd = dict(zip(d.keys(), elem))
            ret.append(dd)

        return ret

    ret = []
    cfgs = generate_all_combinations(d)

    for cfg in cfgs:
        name = ""
        ss = s
        for k, v in cfg.items():
            name += f"_{k.lower().strip('__')}_{str(v)}"
            ss = ss.replace(k, str(v), -1)
        name = name[1:]
        ret.append((name, ss))

    return ret


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


def read_ref_points(file_path: str):
    ret = []

    with open(file_path, "r") as inf:
        data = inf.readlines()

    data = list(filter(lambda x: x.lstrip().startswith("cylinder"), data))

    for d in data:
        d = d.split()
        ret.append(Vec2f(float(d[1]), float(d[2])))

    return ret


def read_path_points(file_path: str):
    ret = []

    with open(file_path, "r") as inf:
        data = inf.readlines()

    for d in data:
        d = d.split()
        ret.append(Vec2f(float(d[0]), float(d[1])))

    return ret


def run_simulator(sim_bin, work_dir, map, path, sim_cfg, sim_params, logger):
    sim_cfgs = generate_configs(sim_cfg, sim_params)
    sim_out_dirs = []

    for name, cfg in sim_cfgs:
        out_dir = f"{work_dir}/sim_out-{name}"

        if not os.path.exists(f"{out_dir}/data"):
            os.makedirs(f"{out_dir}/data")

        with open(f"{out_dir}/sim.cfg.yaml", "w") as of:
            of.write(cfg.strip())

        sim_out_dirs.append(out_dir)
        logger.info(f"[Simulator] Output directory: {out_dir}")

        result = run_command(f"{sim_bin} --config {out_dir}/sim.cfg.yaml --map {map} --path {path} --outdir {out_dir}/data")
        if result.returncode != 0:
            logger.error("[Simulator] Something went wrong!")
            logger.error(f"[Simulator] STDOUT:\n{result.stdout}")
            logger.error(f"[Simulator] STDERR:\n{result.stderr}")
        #  break   # TODO: Remove

    return sim_out_dirs


def run_perception(per_bin, work_dir, sim_out_dirs, per_cfg, per_params, logger):
    per_cfgs = generate_configs(per_cfg, per_params)
    per_out_dirs = []

    for od in sim_out_dirs:
        end = len(files(f"{od}/data", "test_fn*"))

        for name, cfg in per_cfgs:
            out_dir = f"{work_dir}/per_out-{od.lstrip(f'{work_dir}/')}-{name}"

            if not os.path.exists(f"{out_dir}/data"):
                os.makedirs(f"{out_dir}/data")

            with open(f"{out_dir}/per.cfg.yaml", "w") as of:
                of.write(cfg.strip())

            per_out_dirs.append(out_dir)
            logger.info(f"[Perception] Output directory: {out_dir}")

            result = run_command(f"{per_bin} --config {out_dir}/per.cfg.yaml --indir {od}/data --start 1 --end {end} --outdir {out_dir}/data --format xyz")
            if result.returncode != 0:
                logger.error("[Perception] Something went wrong!")
                logger.error(f"[Perception] STDOUT:\n{result.stdout}")
                logger.error(f"[Perception] STDERR:\n{result.stderr}")
            #  break   # TODO: Remove
        #  break   # TODO: Remove

    return per_out_dirs


def run_command(command: str, shell=False) -> int:
    """ Run a command

    Raises exception if the `command` is not valid.
    """

    if command in ["", None]:
        raise Exception("Command cannot be empty or `None`")

    return subprocess.run(shlex.split(command), capture_output=True, shell=shell)
