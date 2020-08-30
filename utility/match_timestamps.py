#!/usr/bin/env python3
import csv
import argparse

parser = argparse.ArgumentParser(description="Finds the closest timestamp in the template csv to the input csv and stores the corresponding row in a new file.")
parser.add_argument("template", metavar='t', type=str, help="The template csv file to get data from.")
parser.add_argument("input", metavar='i', type=str, help="The input csv file to match timestamps from.")
parser.add_argument("--t_start", type=int, default=1, help="The row where the template data starts. Default 1.")
parser.add_argument("--i_start", type=int, default=1, help="The row where the input data starts. Default 1.")
parser.add_argument("--t_col", type=int, default=0, help="The column where the template timestamps are stored. Default 0.")
parser.add_argument("--i_col", type=int, default=0, help="The column where the input timestamps are stored. Default 0.")

args = parser.parse_args()

template_fname = args.template
input_fname = args.input

def read_times(fname, start_row, times_col):
    with open(fname, 'r') as f:
        reader = csv.reader(f)
        # Skip the first rows
        for _ in range(start_row):
            next(reader)
        
        # Read the timestamps
        times = []
        for line in reader:
            time = float(line[times_col])
            times.append(time)
    
    return times

def find_nearest(itimes, ttimes):
    # For each time in itimes, find the nearest ttimes
    # Assume both lists are sorted
    nearest_indices = []
    tt_i = 0
    for it in itimes:
        while tt_i < len(ttimes) and ttimes[tt_i] < it:
            tt_i += 1
        if tt_i == len(ttimes):
            nearest_indices.append(tt_i-1)
        elif tt_i == 0 or abs(ttimes[tt_i] - it) < abs(ttimes[tt_i-1] - it):
            nearest_indices.append(tt_i)
        else:
            nearest_indices.append(tt_i-1)
    return nearest_indices

def copy_template_lines(fname, start_row, indices):
    ofname = fname[:-4] + "_matched" + ".csv"
    with open(fname, 'r') as template_file, open(ofname,'w') as of:
        # Copy the first rows
        for _ in range(start_row):
            line = next(template_file)
            of.write(line)
        
        # Write the relevant indices
        line_ind = 0
        line = next(template_file)
        for ind in indices:
            while line_ind < ind:
                line_ind += 1
                line = next(template_file)
            of.write(line)



# Read the timestamps
input_times = read_times(args.input, args.i_start, args.i_col)
template_times = read_times(args.template, args.t_start, args.t_col)

template_indices = find_nearest(input_times, template_times)
print(len(template_indices))

copy_template_lines(args.template, args.t_start, template_indices)
