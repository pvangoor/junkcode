import csv
import argparse
import progressbar

parser = argparse.ArgumentParser(description="Offset the timestamp of every entry in a csv file.")
parser.add_argument("inputFile", metavar="f", type=str, nargs=1, help="The name of the csv file.")
parser.add_argument("zeroTime", metavar="t", type=float, nargs=1, help="The time by which to offset all entries in the file. t_new = t_old - t0")
args = parser.parse_args()

t0 = args.zeroTime[0]
inputFile = open(args.inputFile[0], 'r')
outputFile = open(args.inputFile[0][:-4]+"_offset.csv", 'w')
reader = csv.reader(inputFile)
writer = csv.writer(outputFile)

row = next(reader) # header
writer.writerow(row)
if row.count("time"):
    timeCol = row.index("time")
elif row.count("Time"):
    timeCol = row.index("Time")
elif row.count("TIME"):
    timeCol = row.index("TIME")

for row in reader:
    row[timeCol] = float(row[timeCol]) - t0
    writer.writerow(row)

inputFile.close()
outputFile.close()


