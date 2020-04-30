#!/usr/bin/env python

# Copyright (C) 2020 Chao Chen, Kevin Han, Gregory Meyer, and Sumukha Udupa
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

import multiprocessing
import os
import shlex
import shutil
import subprocess
import sys
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from pathlib import Path


def main():
    parser = ArgumentParser(
        description="run Cartographer on NCLT Velodyne hits, IMU data, and (optionally) odometry data",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )

    build_run_common_parser = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter, add_help=False
    )

    build_type_group = build_run_common_parser.add_mutually_exclusive_group()
    build_type_group.add_argument(
        "-d",
        "--debug",
        action="store_const",
        const="Debug",
        default="Debug",
        dest="build_type",
        help="pass -DCMAKE_BUILD_TYPE=Debug to cmake",
    )
    build_type_group.add_argument(
        "-r",
        "--release",
        action="store_const",
        const="Release",
        default="Debug",
        dest="build_type",
        help="pass -DCMAKE_BUILD_TYPE=Release to cmake",
    )
    build_type_group.add_argument(
        "-b",
        "--build-type",
        default="Debug",
        dest="build_type",
        help='passed to cmake as "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"',
    )

    generator_group = build_run_common_parser.add_mutually_exclusive_group()
    generator_group.add_argument(
        "-m",
        "--unix-makefiles",
        action="store_const",
        const="Unix Makefiles",
        dest="generator",
        help="pass '-G Unix Makefiles' to cmake",
    )
    generator_group.add_argument(
        "-n",
        "--ninja",
        action="store_const",
        const="Ninja",
        dest="generator",
        help="pass '-G Ninja' to cmake",
    )
    generator_group.add_argument(
        "-g", "--generator", help='passed to cmake as "-G $GENERATOR"'
    )

    build_run_common_parser.add_argument(
        "-c",
        "--cmake-path",
        default="cmake",
        help="path to or name of the cmake executable to run",
    )

    build_run_common_parser.add_argument(
        "-a",
        "--cmake-args",
        type=shlex.split,
        default=[],
        help="additional arguments passed directly to cmake. "
        "split by shlex.split",
    )

    build_run_common_parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        default=False,
        help="print executed commands and pass stdout and stderr through",
    )

    subparsers = parser.add_subparsers(dest="subparser", required=True)

    subparsers.add_parser(
        "clean",
        help="remove artifacts built by this script",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )

    subparsers.add_parser(
        "build",
        help="compile csv-slam without running it",
        formatter_class=ArgumentDefaultsHelpFormatter,
        parents=[build_run_common_parser],
    )

    run_subparser = subparsers.add_parser(
        "run",
        help="compile (if necessary) and run csv-slam",
        formatter_class=ArgumentDefaultsHelpFormatter,
        parents=[build_run_common_parser],
    )

    run_subparser.add_argument(
        "cartographer_config",
        metavar="CONFIG",
        help="Lua configuration file for Cartographer",
    )
    run_subparser.add_argument(
        "velodyne_hits",
        metavar="VEL",
        help="binary file containing Velodyne hits",
    )
    run_subparser.add_argument(
        "ms25_data", metavar="IMU", help="CSV file containing MS25 IMU data",
    )
    run_subparser.add_argument(
        "odometry_data",
        nargs="?",
        metavar="ODOMETRY",
        help="CSV file containing odometry data",
    )
    run_subparser.add_argument(
        "output_filename",
        metavar="OUTPUT",
        help="file to write CSV-formatted output trajectory",
    )

    args = parser.parse_args()

    if args.subparser == "clean":
        target_dir = Path("./target/")

        if target_dir.exists():
            shutil.rmtree(target_dir)

        return

    if args.subparser in {"build", "run"}:
        source_directory = Path(".")
        build_directory = Path(f"./target/{args.build_type}")
        build_directory.mkdir(parents=True, exist_ok=True)

        cmake_args = [
            args.cmake_path,
            "-S",
            str(source_directory),
            "-B",
            str(build_directory),
            f"-DCMAKE_BUILD_TYPE={args.build_type}",
        ]

        if args.generator is not None:
            cmake_args += ["-G", args.generator]

        cmake_args += args.cmake_args

        _run(cmake_args)

        cmake_build_args = [
            args.cmake_path,
            "--build",
            str(build_directory),
            "-j",
            str(multiprocessing.cpu_count()),
        ]

        _run(cmake_build_args, verbose=args.verbose)

        if args.subparser == "run":
            csv_slam_executable = build_directory / "csv-slam"

            if args.odometry_data is not None:
                argv = [
                    csv_slam_executable,
                    args.cartographer_config,
                    args.velodyne_hits,
                    args.ms25_data,
                    args.odometry_data,
                    args.output_filename,
                ]
            else:
                argv = [
                    csv_slam_executable,
                    args.cartographer_config,
                    args.velodyne_hits,
                    args.ms25_data,
                    args.output_filename,
                ]

            os.execv(csv_slam_executable, argv)


def _run(args, *, verbose=False):
    if verbose:
        print(_shlex_join(args))
        result = subprocess.run(
            args, text=True, stdout=sys.stdout, stderr=sys.stderr
        )
    else:
        result = subprocess.run(args, capture_output=True, text=True)

    if result.returncode != 0:
        print(
            f"error: {_shlex_join(args)} returned {result.returncode}",
            file=sys.stderr,
        )

        if not verbose:
            print("stdout:", file=sys.stderr)
            print(result.stdout, file=sys.stderr)
            print("stderr:", file=sys.stderr)
            print(result.stderr, file=sys.stderr)

        sys.exit(result.returncode)


def _shlex_join(args):
    return " ".join(shlex.quote(this_arg) for this_arg in args)


if __name__ == "__main__":
    main()
