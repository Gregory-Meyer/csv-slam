#!/usr/bin/env python

import os
from argparse import ArgumentParser


def main():
    parser = ArgumentParser(
        description="run Cartographer on Velodyne hits and IMU data"
    )
    parser.add_argument(
        "cartographer_config",
        metavar="CONFIG",
        help="Lua configuration file for Cartographer",
    )
    parser.add_argument(
        "velodyne_hits",
        metavar="VEL",
        help="binary file containing Velodyne hits",
    )
    parser.add_argument(
        "ms25_data", metavar="IMU", help="CSV file containing MS25 IMU data",
    )

    args = parser.parse_args()

    os.execv(
        "./csv-slam",
        [
            "./csv-slam",
            args.cartographer_config,
            args.velodyne_hits,
            args.ms25_data,
        ],
    )


if __name__ == "__main__":
    main()
