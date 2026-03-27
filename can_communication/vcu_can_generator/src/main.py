# https://github.com/cantools/cantools/blob/master/src/cantools/subparsers/generate_c_source.py

import argparse
import os
import os.path
import shutil
import sys
from typing import List, Tuple

from cantools import database
from cantools.database import Database
from generators.decoding import gen_c_decoding
from generators.packing import generate_c_packing
from generators.ros_msg import (
    generate_ros_cmake_list,
    generate_ros_message,
    generate_ros_package_xml,
    generate_ros_readme,
)
from generators.unpacking import gen_c_unpacking
from git import Repo

SEASON = os.environ.get("SEASON")
if SEASON is None:
    raise ValueError("Please set the SEASON environment variable (ex. 'efr16')")
SEASON = f"yourcar{SEASON}"


def _do_generate_unpacking(args, database_name: str, dbase: Database):
    filename_h = database_name + ".hpp"
    filename_c = database_name + "Unpacking.cpp"

    header, source = gen_c_unpacking(
        dbase,
        database_name,
        filename_h,
        args.commit_hash,
        args.bit_fields,
        args.use_float,
    )

    generated_src_dir = os.path.join(
        args.output_directory, "mailman", "src", "generated"
    )
    os.makedirs(generated_src_dir, exist_ok=True)

    generated_include_dir = os.path.join(
        args.output_directory, "mailman", "include", "generated"
    )
    os.makedirs(generated_include_dir, exist_ok=True)

    path_h = os.path.join(generated_include_dir, filename_h)

    with open(path_h, "w") as fout:
        fout.write(header)

    path_c = os.path.join(generated_src_dir, filename_c)

    # with open(path_c, 'w') as fout:
    #    fout.write(source)

    print(f"{filename_h},{filename_c}", end="")


def _do_generate_decoding(args, database_name: str, dbase: Database):
    filename_h = database_name + ".hpp"
    filename_c = database_name + "Decoding.cpp"

    source = gen_c_decoding(dbase, database_name, filename_h, args.commit_hash)

    generated_src_dir = os.path.join(
        args.output_directory, "mailman", "src", "generated"
    )
    os.makedirs(generated_src_dir, exist_ok=True)

    path_c = os.path.join(generated_src_dir, filename_c)

    with open(path_c, "w") as fout:
        fout.write(source)

    print(f",{filename_c}", end="")


def _do_generate_packing(args, database_name: str, dbase: Database):
    filename_h = database_name + ".hpp"
    filename_c = database_name + "Packing.cpp"

    source = generate_c_packing(dbase, database_name, filename_h, args.commit_hash)

    generated_src_dir = os.path.join(
        args.output_directory, "mailman", "src", "generated"
    )
    os.makedirs(generated_src_dir, exist_ok=True)

    path_c = os.path.join(generated_src_dir, filename_c)

    with open(path_c, "w") as fout:
        fout.write(source)

    print(f",{filename_c}", end="")


def _do_generate_ros_structure(
    out_dir: str,
    ros_addons: List[str],
    dbs: List[Tuple[str, Database]],
    commit_hash: str,
):
    can_msgs_dir = os.path.join(out_dir, "vcu_can_msgs")

    msg_dir = os.path.join(can_msgs_dir, "msg")
    os.makedirs(msg_dir, exist_ok=False)

    # Copy the ROS message addon files
    for ros_addon in ros_addons:
        shutil.copyfile(
            os.path.join(ros_addon), os.path.join(msg_dir, os.path.basename(ros_addon))
        )

    # Package XML file, this shouldn't change,
    # maybe changing the version might be an option
    xml_content = generate_ros_package_xml()
    xml_path = os.path.join(can_msgs_dir, "package.xml")
    with open(xml_path, "w") as fout:
        fout.write(xml_content)

    cmake_list_content = generate_ros_cmake_list(dbs, ros_addons)
    cmake_list_path = os.path.join(can_msgs_dir, "CMakeLists.txt")
    with open(cmake_list_path, "w") as fout:
        fout.write(cmake_list_content)

    readme_content = generate_ros_readme(commit_hash)
    readme_path = os.path.join(can_msgs_dir, "README.md")
    with open(readme_path, "w") as fout:
        fout.write(readme_content)


def _do_generate_ros_messages(args, database_name: str, dbase: Database):
    msg_dir = os.path.join(args.output_directory, "vcu_can_msgs", "msg")

    generated_msgs = []

    for msg in dbase.messages:
        msg_name, msg_def = generate_ros_message(database_name, msg, args.commit_hash)
        msg_path = os.path.join(msg_dir, msg_name)

        with open(msg_path, "w") as fout:
            fout.write(msg_def)
            generated_msgs.append(msg_name)

    _message_list = ", ".join(generated_msgs)


def _main():
    parser = argparse.ArgumentParser(
        prog="cantool_generator",
        description="Generate file from .dbc CAN definition",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--database-name",
        help=(
            "The database name.  Uses the stem of the input file name if not specified."
        ),
    )
    parser.add_argument(
        "--no-floating-point-numbers",
        action="store_true",
        default=False,
        help="No floating point numbers in the generated code.",
    )
    parser.add_argument(
        "--bit-fields",
        action="store_true",
        help="Use bit fields to minimize struct sizes.",
    )
    parser.add_argument("-e", "--encoding", help="File encoding.")
    parser.add_argument(
        "--prune",
        action="store_true",
        help="Try to shorten the names of named signal choices.",
    )
    parser.add_argument(
        "--no-strict", action="store_true", help="Skip database consistency checks."
    )
    parser.add_argument(
        "-f",
        "--generate-fuzzer",
        action="store_true",
        help="Also generate fuzzer source code.",
    )
    parser.add_argument(
        "-o",
        "--output-directory",
        default="./dist",
        help="Directory in which to write output files.",
    )
    parser.add_argument(
        "--addon-directory",
        default=os.path.join(os.path.dirname(__file__), "..", "addons", SEASON),
        help="Directory in which additional files are hosted, for example the InverterData definitions",
    )
    parser.add_argument(
        "--use-float",
        action="store_true",
        default=False,
        help="Use float instead of double for floating point generation.",
    )
    parser.add_argument("indir", help="Folder with database files.")
    parser.set_defaults(func=_execute_all)

    args = parser.parse_args()

    try:
        args.func(args)
    except BaseException as e:
        sys.exit("error: " + str(e))


def _execute_all(args):
    indir = os.path.abspath(args.indir)

    commit_hash = Repo(indir).head.commit.hexsha
    setattr(args, "commit_hash", commit_hash)

    filenames: List[str] = sorted(
        filter(lambda x: x.endswith(".dbc") and (not "alt" in x), os.listdir(indir))
    )
    print(f"Generating for {filenames}")

    dbs: List[Tuple[str, Database]] = []
    for filename in filenames:
        database_name = filename.split("_")[0].lower().capitalize()
        dbase = database.load_file(
            os.path.join(indir, filename),
            encoding=args.encoding,
            prune_choices=args.prune,
            strict=not args.no_strict,
        )
        dbs.append((database_name, dbase))

    # Make path absolute
    args.output_directory = os.path.abspath(args.output_directory)

    # List of absolute paths to ROS msg addon files
    ros_addons: List[str] = []
    for addon_filename in sorted(os.listdir(args.addon_directory)):
        abs_path = os.path.abspath(os.path.join(args.addon_directory, addon_filename))
        if addon_filename.endswith(".msg"):
            ros_addons.append(abs_path)

    if os.path.isdir(args.output_directory):
        import shutil

        # print("Deleting previous dist folder")
        shutil.rmtree(args.output_directory)

    _do_generate_ros_structure(args.output_directory, ros_addons, dbs, commit_hash)

    for database_name, db in dbs:
        print(
            f" > Generating {database_name} ({len(db.messages)} messages):  \t", end=""
        )
        _do_generate_unpacking(args, database_name, db)
        _do_generate_packing(args, database_name, db)
        _do_generate_decoding(args, database_name, db)
        _do_generate_ros_messages(args, database_name, db)


if __name__ == "__main__":
    _main()
