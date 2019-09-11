import numpy as np
import csv
import argparse
import progressbar

def skew(v):
    s = np.array([[0., -v[2,0], v[1,0]], [v[2,0], 0., -v[0,0]], [-v[1,0], v[0,0], 0.]])
    return s

def rod2rotm(rod):
    R = 1.0/(1.0 + rod.T.dot(rod)) * ( (1-rod.T.dot(rod))*np.eye(3) + 2*rod.dot(rod.T) + 2*skew(rod) )
    return R

def rod2quat(rod):
    theta = np.arctan(np.linalg.norm(rod))
    q = np.block([[np.cos(theta)*rod],[np.cos(theta)]])
    return q


parser = argparse.ArgumentParser(description="Compute hand-eye calibration using aruco markers and vicon poses")
parser.add_argument("viconFile", metavar="v", type=str, nargs=1, help="The name of the stamped vicon poses file.")
parser.add_argument("arucoFile", metavar="a", type=str, nargs=1, help="The name of the stamped aruco poses file.")
args = parser.parse_args()

print "Collecting vicon pose data..."
with open(args.viconFile[0], 'r') as viconFile:
    viconReader = csv.reader(viconFile)
    next(viconReader)
    for row in viconReader:
        time = float(row[0])
        t = np.array([[float(x)] for x in row[1:4]])
        q = rod2quat(np.array([[float(x)] for x in row[4:7]]))
        print q
        






print "Determining marker ids present in video..."
idDict = {}
with open(args.arucoFile[0], 'r') as arucoFile:
    arucoReader = csv.reader(arucoFile)
    header_row = next(arucoReader) # header row
    idColumn = header_row.index("id")

    for row in arucoReader:
        idNumber = row[idColumn]
        if idNumber in idDict:
            idDict[idNumber] += 1
        else:
            idDict[idNumber] = 1

print "Found (markers:instances):"
print idDict



