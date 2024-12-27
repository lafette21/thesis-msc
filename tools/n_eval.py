#!/usr/bin/env python3

import fnmatch
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
    random:
      seed: 1
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


def generate_configs(s: str, d: dict):
    ret = []
    cfgs = _gen_all_combinations(d)

    for cfg in cfgs:
        name = ""
        ss = s
        for k, v in cfg.items():
            name += f"_{k.lower().strip('__')}_{str(v)}"
            ss = ss.replace(k, str(v), -1)
        name = name[1:]
        ret.append((name, ss))

    return ret


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


def run_command(command: str, shell=False) -> int:
    """ Run a command

    Raises exception if the `command` is not valid.
    """

    if command in ["", None]:
        raise Exception("Command cannot be empty or `None`")

    return subprocess.run(shlex.split(command), capture_output=True, shell=shell)


def main():
    if len(os.sys.argv) < 4:
        print(f"Usage: {os.sys.argv[0]} <SIMULATOR_BIN> <PERCEPTION_BIN> <WORK_DIR>")
        exit(1)

    sim_bin = os.sys.argv[1]
    per_bin = os.sys.argv[2]
    work_dir = os.sys.argv[3]
    run_command(f"mkdir -p {work_dir}")

    print(f"Simulator: {sim_bin}")
    print(f"Perception: {per_bin}")
    print(f"Working directory: {work_dir}\n")

    sim_cfgs = generate_configs(SIM_CONFIG, SIM_PARAMS)
    per_cfgs = generate_configs(PER_CONFIG, PER_PARAMS)

    sim_out_dirs = []

    for name, cfg in sim_cfgs:
        out_dir = f"{work_dir}/sim_out-{name}"
        run_command(f"mkdir -p {out_dir}/data")

        with open(f"{out_dir}/sim.cfg.yaml", "w") as of:
            of.write(cfg.strip())

        sim_out_dirs.append(out_dir)
        print(f"Output directory: {out_dir}")

        result = run_command(f"{sim_bin} --config {out_dir}/sim.cfg.yaml --map data/maps/garage.map --path data/paths/garage.path --outdir {out_dir}/data")
        if result.returncode != 0:
            print("[Simulator] Something went wrong!")
        break

    for od in sim_out_dirs:
        end = len(files(f"{od}/data", "test_fn*"))

        for name, cfg in per_cfgs:
            out_dir = f"{work_dir}/per_out-{od.lstrip(f'{work_dir}/')}-{name}"
            run_command(f"mkdir -p {out_dir}/data")

            with open(f"{out_dir}/per.cfg.yaml", "w") as of:
                of.write(cfg.strip())

            print(f"Output directory: {out_dir}")

            result = run_command(f"{per_bin} --config {out_dir}/per.cfg.yaml --indir {od}/data --start 1 --end {end} --outdir {out_dir}/data --format xyz")
            if result.returncode != 0:
                print("[Perception] Something went wrong!")
            break
        break


if __name__ == "__main__":
    main()
