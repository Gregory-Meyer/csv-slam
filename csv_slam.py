#!/usr/bin/env python

# Copyright (C) 2020 Chao Chen, Yutian Han, Gregory Meyer, and Sumukha Udupa
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
    parser.add_argument(
        "odometry_data",
        nargs="?",
        metavar="ODOMETRY",
        help="CSV file containing odometry data",
    )
    parser.add_argument(
        "output_filename",
        metavar="OUTPUT",
        help="file to write CSV-formatted output trajectory",
    )

    args = parser.parse_args()

    if args.odometry_data is not None:
        argv = [
            "./csv-slam",
            args.cartographer_config,
            args.velodyne_hits,
            args.ms25_data,
            args.odometry_data,
            args.output_filename,
        ]
    else:
        argv = (
            [
                "./csv-slam",
                args.cartographer_config,
                args.velodyne_hits,
                args.ms25_data,
                args.output_filename,
            ],
        )

    os.execv(
        "./csv-slam", argv,
    )


if __name__ == "__main__":
    main()
