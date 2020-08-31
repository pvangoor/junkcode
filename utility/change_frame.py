#!/usr/bin/env python3
import csv
import argparse
from pylie import SE3

parser = argparse.ArgumentParser(description="Compute the poses of a frame that is rigidly attached to the frame of the original poses.")
parser.add_argument("original", metavar='f', type=str, help="The file of original poses.")
parser.add_argument("--fspec", type=str, default='xw', help="The format of the poses in the file. Default xw.")
parser.add_argument("--col", type=int, default=1, help="The column where the poses start in the file. Default 1.")
parser.add_argument("--header", type=int, default=1, help="The number of header rows in the file. Default 1.")
args = parser.parse_args()

P_list = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949]

frame_offset = SE3.from_list(P_list, 'P')

output_fname = args.original[:-4] + "_offset.csv"
with open(args.original, 'r') as original_file, open(output_fname, 'w') as output_file:
    reader = csv.reader(original_file)
    writer = csv.writer(output_file)

    # Copy the header
    for _ in range(args.header):
        line = next(reader)
        writer.writerow(line)
    
    # Offset and write the poses
    for row in reader:
        original_pose = SE3.from_list(row[args.col:], args.fspec)
        new_pose = original_pose * frame_offset
        new_row = new_pose.to_list(args.fspec)
        new_row = row[:args.col] + new_row
        writer.writerow(new_row)
    

