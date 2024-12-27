#!/usr/bin/env python3

import itertools
import os
import shlex
import subprocess


SIM_CONFIG = """
simulation:
  path:
    num_interp_points: 1001
  velocity:
    max: __VELOCITY__ # m/s
motion:
  sampling: 4 # Hz
lidar:
  vlp_16:
    channels: 16
    range: 100 # m
    fov_v: 30 # deg
    fov_h: 360 # deg
    ang_res_v: 2 # deg
    rotation_rate:
      min: 5 # Hz
      max: 20 # Hz
      value: 20 # Hz
    rpm:
      min: 300
      max: 1200
      partition_size: 60
      value: 1200
    firing_cycle: 5.5296e-05
    accuracy:
      value: __ACCURACY__ # m
"""

PER_CONFIG = """
random:
  seed: 1
downsampling:
  enabled: true
  leaf_size:
    x: 0.07 # m
    y: 0.07 # m
    z: 0.07 # m
plane_filtering:
  distance_threshold: 0.1 # m
  min_inliers: 500
clustering:
  k_search: 50
  cluster_size:
    max: 1000000
    min: 50
  num_of_neighbours: 30
  smoothness_threshold: 0.314159265 # rad (18.f / 180.f * pi)
  curvature_threshold: 1
circle_extraction:
  ransac:
    distance_threshold: 0.07 # m
    iter: 10000
    min_samples: 20
    r_max: 0.32 # m
    r_min: 0.28 # m
pairing:
  distance_threshold: 0.5 # m
"""


SIM_PARAMS = {
    "__VELOCITY__": [1, 2, 4],
    "__ACCURACY__": [0.01, 0.05, 0.1],
}

PER_PARAMS = {

}


def _gen_all_combinations(d: dict) -> list:
    ret = []

    cartesian_prod = list(itertools.product(*d.values()))

    for elem in cartesian_prod:
        dd = dict(zip(d.keys(), elem))
        ret.append(dd)

    return ret


def replace_all(s: str, d: dict):
    ret = []
    cfgs = _gen_all_combinations(d)

    for cfg in cfgs:
        ss = s
        for k, v in cfg.items():
            ss = ss.replace(k, str(v), -1)
        ret.append(ss)

    return ret



def run_command(command: str, shell=False) -> int:
    """ Run a command

    Raises exception if the `command` is not valid.
    """

    if command in ["", None]:
        raise Exception("Command cannot be empty or `None`")

    return subprocess.run(shlex.split(command), capture_output=True, shell=shell)


def main():
    if len(os.sys.argv) < 3:
        print(f"Usage: {os.sys.argv[0]} <SIMULATOR_BIN> <PERCEPTION_BIN>")
        exit(1)

    sim_bin = os.sys.argv[1]
    per_bin = os.sys.argv[2]

    print(f"Simulator: {sim_bin}")
    print(f"Perception: {per_bin}\n")

    print(f"Using config for simulator:\n```yaml{SIM_CONFIG}```\n")
    print(f"Using config for perception:\n```yaml{PER_CONFIG}```\n")

    #  result = run_command(f"cat <(echo \"{SIM_CONFIG}\") | yq", shell=True)
    #  print(result.stdout)

    sim_cfgs = replace_all(SIM_CONFIG, SIM_PARAMS)
    per_cfgs = replace_all(PER_CONFIG, PER_PARAMS)


if __name__ == "__main__":
    main()
