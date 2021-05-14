"""
This Python scripts is used for shift odometry timestamp and generate new time shifted odometry data.
Params: 
    shift_value: represent for delta value we need to shift
    num_shifts: represent number of shift data we will generate, shifts in shift_data will be shift_value * 0, shift_value * 1, ..., shift_value * (num_shifts - 1)
"""

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default="formatted_odometry.txt", help="original formatted odometry file")
parser.add_argument("--output", type=str, default="./output", help="directory where to store the time shifted odom file")
parser.add_argument("--shift_value", type=float, default=1e-5, help="represent for delta value we need to shift")
parser.add_argument("--num_shifts", type=int, default=100, help=" represent number of shift data we will generate, shifts in shift_data will be shift_value * 0, shift_value * 1, ..., shift_value * (num_shifts - 1)")
args = parser.parse_args()

os.makedirs(args.output, exist_ok=True)

num_shifts = args.num_shifts
for i in range(num_shifts):
    output_filenames = os.path.join(args.output, "odom_formatted_{}.txt".format(i))
    f_out = open(output_filenames, "w")
    with open(args.filename, "r") as f:
        for idx, element in enumerate(f):
            element = element.strip()
            if idx == 0:
                f_out.write(element + "\n")
                continue
            data = element.split(",")
            data[0] = str(float(data[0]) + args.shift_value * i)
            new_data = ",".join(data) + "\n"
            f_out.write(new_data)
    f_out.close()
