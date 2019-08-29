'''
Phone Sensor Data Converter: txt to csv
Usage: phone_imu_txt2csv.py -f"
Arguments:
 -f, --file         The file containing IMU data in the sensor log format.

Flags:
 -h                 help
'''     

import csv
import numpy as np
import progressbar

import getopt
import sys

def usage():
    print sys.exit(__doc__)

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

def write_csv_msg(writer, msg_dict, timestamp):
    record_list = ['GYR', 'ACC']
    row = [timestamp]
    for record in record_list:
        if record in msg_dict:
            row += msg_dict[record]
        else:
            break
    else:
        writer.writerow(row)


try:
    opts, args = getopt.getopt(sys.argv[1:], "f:", ["file="])
except getopt.GetoptError as err:
    print(str(err))
    usage()

for opt, arg in opts:
    if opt == '-h':
        usage()

    elif opt in ("-f", "--file"):
        imu_fname_txt = arg
        if not imu_fname_txt.endswith(".txt"):
            imu_fname_txt = imu_fname_txt + ".txt"
        
        try:
            imu_file_txt = open(imu_fname_txt, 'r')
        except IOError:
            print "Could not find the IMU text file."
            sys.exit(1)

progBar = progressbar.ProgressBar(max_value=file_len(imu_fname_txt))

imu_fname_csv = imu_fname_txt[:-4] + ".csv"
imu_file_csv = open(imu_fname_csv, 'w')
imu_csv_writer = csv.writer(imu_file_csv)

header_row = ["TIME", "GYRX", "GYRY", "GYRZ", "ACCX", "ACCY", "ACCZ", "MAGX", "MAGY", "MAGZ"]
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
        write_csv_msg(imu_csv_writer, msg_dict, timestamp)
        current_time = timestamp
        msg_dict = {msg_type:msg_val}

    line_count += 1
    progBar.update(line_count)





imu_file_txt.close()
imu_file_csv.close()