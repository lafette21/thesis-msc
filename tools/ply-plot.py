#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def read_file(path, colored=True):
    with open(path, 'r') as file:
        lines = file.readlines()

    # Extract coordinates
    x = []
    y = []
    z = []
    colors = []
    flag = False
    for line in lines[:-1]:
        if "end_header\n" in line:
            flag = True
        elif flag and line.strip():  # Skip empty lines
            coordinates = line.split()
            x.append(float(coordinates[0]))
            y.append(float(coordinates[1]))
            z.append(float(coordinates[2]))
            if colored:
                # Append color values, normalized between 0 and 1
                r = int(coordinates[3]) / 255.0
                g = int(coordinates[4]) / 255.0
                b = int(coordinates[5]) / 255.0
                colors.append([r, g, b])
            else:
                colors.append([164 / 255.0, 152 / 255.0, 16 / 255.0])

    return x, y, z, colors


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="PLY Plot Script",
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument("-f", "--file", required=True)
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("-c", "--colored", action="store_true", default=False)
    parser.add_argument(      "--show", action="store_true", default=False)

    return parser.parse_args()


def main():
    args = parse_arguments()

    xs, ys, zs, colors = read_file(args.file, colored=args.colored)

    fig = plt.figure(figsize=(12, 6))

    # --- First subplot: Bird's-eye view
    ax1 = fig.add_subplot(121, projection='3d')

    ax1.scatter(xs, ys, zs, color=colors, s=5, label='LiDAR Point')

    # Set axis labels
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')

    # Set axis limits
    ax1.set_xlim([-0.5, 2])
    ax1.set_ylim([-0.5, 2])
    ax1.set_zlim([-0.5, 2])
    ax1.axis('equal')

    # Remove grid, ticks, and background for a clean look
    ax1.grid(False)
    ax1.set_xticks([])
    ax1.set_yticks([])
    ax1.set_zticks([])
    ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    # Add a title for the first subplot
    ax1.set_title("Bird's-Eye View")

    # Add a legend
    ax1.legend()

    ax1.view_init(elev=90, azim=-90)

    # --- First subplot: Bird's-eye view
    ax2 = fig.add_subplot(122, projection='3d')

    ax2.scatter(xs, ys, zs, color=colors, s=5, label='LiDAR Point')

    # Set axis labels
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')

    # Set axis limits
    ax2.set_xlim([-0.5, 2])
    ax2.set_ylim([-0.5, 2])
    ax2.set_zlim([-0.5, 2])
    ax2.axis('equal')

    # Remove grid, ticks, and background for a clean look
    ax2.grid(False)
    ax2.set_xticks([])
    ax2.set_yticks([])
    ax2.set_zticks([])
    ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    # Add a title for the first subplot
    ax2.set_title("Isometric View")

    # Add a legend
    ax2.legend()

    ax2.view_init(elev=35.264, azim=-45)

    plt.savefig(args.output)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
