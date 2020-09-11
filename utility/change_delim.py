#!/usr/bin/env python3
import csv
import argparse

parser = argparse.ArgumentParser(description="Change the delimiter in a csv.")
parser.add_argument("fname", metavar='f', type=str, help="The csv file to edit.")
parser.add_argument("--old_delim", metavar='o', type=str, default=" ", help="The current delimiter. Default \" \"")
parser.add_argument("--new_delim", metavar='n', type=str, default=",", help="The desired delimiter. Default \",\"")

args = parser.parse_args()

fname = args.fname
ofname = fname[:-4] + "_delimchanged.csv"
with open(fname, 'r') as f, open(ofname, 'w') as of:
    reader = csv.reader(f, delimiter=args.old_delim)
    writer = csv.writer(of, delimiter=args.new_delim)
    for line in reader:
        writer.writerow(line)

