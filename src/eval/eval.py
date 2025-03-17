#!/usr/bin/env python3

import argparse
import json
import logging
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import re
import traceback

import utils

from enum import Enum


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
    iter: __RANSAC_ITER__
    min_samples: 20
    r_max: 0.32 # m
    r_min: 0.28 # m
"""

SLAM_CONFIG = """
pairing:
  distance_threshold: 0.5 # m
spatial_consistency:
  buff_capacity: 3
  min_occurrence: 2
  distance_threshold: 0.5
input_sampling_rate: 4 # Hz
initial_velocity: __INITIAL_VELOCITY__ # m/s
"""


SIM_PARAMS = {
    "__VELOCITY__": [1, 2],
    "__ACCURACY__": [0.01, 0.05, 0.1],
}

PER_PARAMS = {
    "__RANSAC_ITER__": [10000, 100000]
}

SLAM_PARAMS = {
    "__INITIAL_VELOCITY__": [1, 2],
}


class AnsiColors(Enum):
    DARK_GREY = "\x1b[30;1m"
    RED       = "\x1b[31;20m"
    BOLD_RED  = "\x1b[31;1m"
    GREEN     = "\x1b[32;20m"
    YELLOW    = "\x1b[33;20m"
    BLUE      = "\x1b[34;20m"
    PURPLE    = "\x1b[35;20m"
    CYAN      = "\x1b[36;20m"
    WHITE     = "\x1b[37;1m"
    GREY      = "\x1b[38;20m"
    DEFAULT   = "\x1b[0m"


class ColorLogFormatter(logging.Formatter):
    formats = {
        logging.DEBUG:    "%(asctime)s [%(name)s] {}%(levelname)s:{} %(message)s".format(AnsiColors.BLUE.value, AnsiColors.DEFAULT.value),
        logging.INFO:     "%(asctime)s [%(name)s] {}%(levelname)s:{} %(message)s".format(AnsiColors.GREEN.value, AnsiColors.DEFAULT.value),
        logging.WARNING:  "%(asctime)s [%(name)s] {}%(levelname)s:{} %(message)s".format(AnsiColors.YELLOW.value, AnsiColors.DEFAULT.value),
        logging.ERROR:    "%(asctime)s [%(name)s] {}%(levelname)s:{} %(message)s".format(AnsiColors.RED.value, AnsiColors.DEFAULT.value),
        logging.CRITICAL: "%(asctime)s [%(name)s] {}%(levelname)s:{} %(message)s".format(AnsiColors.BOLD_RED.value, AnsiColors.DEFAULT.value)
    }

    def format(self, record):
        log_fmt = self.formats.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


log_handler = logging.StreamHandler()
log_handler.setFormatter(ColorLogFormatter())

logger = logging.getLogger("Eval")
logger.addHandler(log_handler)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="Evaluation script",
    )

    parser.add_argument("--simulator", metavar="SIMULATOR_BIN", type=str, required=True, help="path of the simulator binary executable")
    parser.add_argument("--perception", metavar="PERCEPTION_BIN", type=str, required=True, help="path of the perception binary executable")
    parser.add_argument("--slam", metavar="SLAM_BIN", type=str, required=True, help="path of the SLAM binary executable")
    parser.add_argument("-w", "--work-dir", type=str, required=True, help="path of the working directory (will be created if not present)")
    parser.add_argument("--map", type=str, required=True, help="path of the map file")
    parser.add_argument("--path", type=str, required=True, help="path of the path file")
    parser.add_argument("-v", "--verbose", action="store_true")

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.verbose:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    if os.path.exists(args.work_dir) and not os.path.isdir(args.work_dir):
        logger.error(f"The path `{args.work_dir}` already exists and it is not a directory!")
        exit(1)

    if not os.path.exists(args.work_dir):
        os.makedirs(args.work_dir)
        logger.info(f"The directory `{args.work_dir}` has been created!")

    logger.info(f"Simulator: {args.simulator}")
    logger.info(f"Perception: {args.perception}")
    logger.info(f"SLAM: {args.slam}")
    logger.info(f"Working directory: {args.work_dir}\n")

    sim_out_dirs = utils.run_simulator(args.simulator, args.work_dir, args.map, args.path, SIM_CONFIG, SIM_PARAMS, logger)
    logger.debug(f"Simulator output directories: {sim_out_dirs}")

    per_out_dirs = utils.run_perception(args.perception, args.work_dir, sim_out_dirs, PER_CONFIG, PER_PARAMS, logger)
    logger.debug(f"Perception output directories: {per_out_dirs}")

    slam_out_dirs = utils.run_slam(args.slam, args.work_dir, per_out_dirs, SLAM_CONFIG, SLAM_PARAMS, logger)
    logger.debug(f"SLAM output directories: {slam_out_dirs}")

    ref_points = utils.read_ref_points(args.map)
    logger.debug(f"Reference points: {ref_points}")

    path_points = utils.read_path_points(args.path)
    logger.debug(f"Path points: {path_points}")

    origin = path_points[0]

    dist_sum = lambda l: sum((r - p).length() for r, p in l)

    evaluated_scenarios = {}

    for dir in slam_out_dirs:
        file_path_list, paired_list, out_of_range_list = utils.eval_scenario(f"{dir}/data", origin, ref_points, logger)
        evaluated_scenarios[dir] = (file_path_list, paired_list, out_of_range_list)

    for k, v in evaluated_scenarios.items():
        data = {
            "file": [],
            "success_ratio": [],
            "fail_ratio": [],
            "gt_ratio": [],
            "avg_in_range_dist": [],
            "avg_out_of_range_dist": [],
            "processing_time_us": [],
        }

        with open(f"{k}/data/stats.json", "r") as inf:
            stats = json.load(inf)

        for file, paired, out_of_range in zip(*v):
            data["file"].append(file)
            idx = re.sub(".*registered-", '', file).rstrip(".xyz")
            if len(paired) > 0 or len(out_of_range) > 0:
                data["success_ratio"].append(len(paired) / (len(paired) + len(out_of_range)))
                data["fail_ratio"].append(len(out_of_range) / (len(paired) + len(out_of_range)))
                data["gt_ratio"].append(len(paired) / len(ref_points))
                # TODO: There is a case where during clustering one cylinder is split into two and hence recognized as two cylinders
                #  if len(paired) / len(ref_points) > 1:
                    #  for r, p in paired:
                        #  print(p)
                    #  break
            else:
                data["success_ratio"].append(np.nan)
                data["fail_ratio"].append(np.nan)
                data["gt_ratio"].append(np.nan)
            if len(paired) > 0:
                data["avg_in_range_dist"].append(dist_sum(paired) / len(paired))
            else:
                data["avg_in_range_dist"].append(np.nan)
            if len(out_of_range) > 0:
                data["avg_out_of_range_dist"].append(dist_sum(out_of_range) / len(out_of_range))
            else:
                data["avg_out_of_range_dist"].append(np.nan)
            data["processing_time_us"].append(stats["processingTimesUs"][idx])

        fields = {
            "success_ratio": "Success ratio",
            "fail_ratio": "Fail ratio",
            "gt_ratio": "Ground truth ratio",
            "avg_in_range_dist": "Avg. in-range distance (m)",
            "avg_out_of_range_dist": "Avg. out-of-range distance (m)",
            "processing_time_us": "Processing time (μs)"
        }

        for fk, fv in fields.items():
            field = data[fk]
            label = fv

            print(field)
            print(label)

            #  plt.scatter([i for i, elem in enumerate(field) if elem != "NaN"], [elem for elem in field if elem != "NaN"], color='#18789F', s=15)
            plt.scatter([i for i in range(len(field))], field, color='#18789F', s=15)

            plt.xlabel('Frame')
            plt.ylabel(label)

            plt.savefig(f'{args.work_dir}/{fk}-{os.path.basename(k)}.pdf')
            plt.clf()

        df = pd.DataFrame(data)
        #  df.to_excel(f"{k}.xlsx", index=False)
        df.to_csv(f"{k}.csv", index=False)

if __name__ == "__main__":
    try:
        main()
    except Exception as ex:
        logger.error(f"Exception: {ex}")
        if logger.getEffectiveLevel() == logging.DEBUG:
            traceback.print_exception(ex)
