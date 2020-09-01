#!/usr/bin/env python3
import csv
import argparse
import progressbar

parser = argparse.ArgumentParser(description="Delete the first N columns from a csv file.")
parser.add_argument("fname", metavar='f', type=str, help="The file name.")
parser.add_argument("--ncol", metavar='n', type=int, default=1, help="The number of columns to delete. Default 1.")
args = parser.parse_args()

def count_lines(fname):
    total = 0
    with open(fname,'r') as f:
        for _ in f:
            total += 1
    return total

ofname = args.fname[:-4] + "_truncated.csv"

num_lines = count_lines(args.fname)

with open(args.fname,'r') as f, open(ofname,'w') as of:
    reader = csv.reader(f)
    writer = csv.writer(of)
    for row in progressbar.progressbar(reader, max_value=num_lines):
        writer.writerow(row[args.ncol:])
