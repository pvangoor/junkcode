#!/usr/bin/env python3
import csv
import numpy as np
import progressbar

import argparse

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

def write_csv_msg(writer, msg_dict, timestamp, record_list):
    row = [timestamp]
    for record in record_list:
        if record in msg_dict:
            row += msg_dict[record]
        else:
            break
    else:
        writer.writerow(row)

parser = argparse.ArgumentParser(description="Process a sensor log file.")
parser.add_argument("--raw", action='store_false', help="Output raw sensor data.")
parser.add_argument("fileName", metavar="f", type=str, nargs=1, help="The name of the txt file to be processed.")
args = parser.parse_args()


imu_fname_txt = args.fileName[0]
if not imu_fname_txt.endswith(".txt"):
    imu_fname_txt = imu_fname_txt + ".txt"

try:
    imu_file_txt = open(imu_fname_txt, 'r')
except IOError:
    print("Could not find the IMU text file.")
    raise(IOError)

header_row = ["TIME"]
if args.raw:
    record_list = ['GYR', 'ACC', 'MAG']
    for record in record_list:
        header_row += [record+pos for pos in ("X","Y","Z")]
    imu_fname_csv = imu_fname_txt[:-4] + ".csv"
else:
    record_list = ['RAWGYR', 'RAWMAG']
    for record in record_list:
        header_row += [record+pos for pos in ("X","Y","Z","BX","BY","BZ")]
    imu_fname_csv = imu_fname_txt[:-4] + "_RAW" + ".csv"

progBar = progressbar.ProgressBar(max_value=file_len(imu_fname_txt))


imu_file_csv = open(imu_fname_csv, 'w')
imu_csv_writer = csv.writer(imu_file_csv)


imu_csv_writer.writerow(header_row)


# Read through the file
current_time = -1
msg_dict = {}
line_count = 0
for txt_row in imu_file_txt:
    txt_row = txt_row.strip('\n')
    txt_row = txt_row.split("\t")

    timestamp = int(txt_row[0]) * 1e-3
    if (current_time < 0):
        current_time = timestamp
    
    msg_type = txt_row[1]
    msg_val = [float(val) for val in txt_row[2:]]
    
    if abs(timestamp - current_time) < 1e-6:
        # Collect messages with the same timestamps
        msg_dict[msg_type] = msg_val
    
    else:
        # Write when a new timestamp appears
        write_csv_msg(imu_csv_writer, msg_dict, timestamp, record_list)
        current_time = timestamp
        msg_dict = {msg_type:msg_val}

    line_count += 1
    progBar.update(line_count)





imu_file_txt.close()
imu_file_csv.close()