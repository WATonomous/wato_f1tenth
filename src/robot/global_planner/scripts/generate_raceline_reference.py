#!/usr/bin/env python3

"""Add raceline-relative right/left track widths to a raceline CSV."""

import argparse
import csv
import math


def read_rows(path, skip_header=False):
    with open(path, newline="") as csv_file:
        rows = list(csv.reader(csv_file))
    if skip_header:
        rows = rows[1:]
    return [[float(value) for value in row] for row in rows]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raceline", help="headerless x,y,v raceline CSV")
    parser.add_argument("centerline", help="x,y,right,left centerline CSV with a header")
    parser.add_argument("output", help="output x,y,v,dr,dl CSV")
    args = parser.parse_args()

    raceline = read_rows(args.raceline)
    centerline = read_rows(args.centerline, skip_header=True)

    with open(args.output, "w", newline="") as csv_file:
        writer = csv.writer(csv_file, lineterminator="\n")
        writer.writerow(["x_m", "y_m", "v_mps", "dr_m", "dl_m"])

        for x, y, velocity in raceline:
            closest = min(
                range(len(centerline)),
                key=lambda index: (centerline[index][0] - x) ** 2
                + (centerline[index][1] - y) ** 2,
            )
            previous = centerline[(closest - 1) % len(centerline)]
            following = centerline[(closest + 1) % len(centerline)]
            tangent_x = following[0] - previous[0]
            tangent_y = following[1] - previous[1]
            tangent_length = math.hypot(tangent_x, tangent_y)
            normal_x = -tangent_y / tangent_length
            normal_y = tangent_x / tangent_length

            center_x, center_y, center_right, center_left = centerline[closest]
            raceline_d = (x - center_x) * normal_x + (y - center_y) * normal_y
            right = center_right + raceline_d
            left = center_left - raceline_d
            writer.writerow([x, y, velocity, right, left])


if __name__ == "__main__":
    main()
