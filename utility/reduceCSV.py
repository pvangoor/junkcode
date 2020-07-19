import csv
import argparse

parser = argparse.ArgumentParser(description="Reduce the number of rows in a CSV file.")
parser.add_argument('fname', metavar='f', type=str, help="The name of the CSV file.")
parser.add_argument('steps', type=int, help="The number of steps between rows kept from the original file.")
parser.add_argument('--noHeader', type=bool, default=False, help="Set this flag if there is no header in the file, i.e. the first row is data.")
parser.add_argument('--skipRows', type=int, default=-1, help="The number of rows to be skipped at the start of the file.")
args = parser.parse_args()

posesFname = args.fname
newPosesFname = posesFname[:-4] + "_reduced.csv"

with open(posesFname, 'r') as posesFile, open(newPosesFname, 'w') as newPosesFile:
    posesCSV = csv.reader(posesFile)
    newPosesCSV = csv.writer(newPosesFile)

    if not args.noHeader:
        newPosesCSV.writerow(next(posesCSV))

    i=0
    for line in posesCSV:
        if i % args.steps == 0 and i > args.skipRows:
            newPosesCSV.writerow(line)
        i+=1
        
