import argparse

parser = argparse.ArgumentParser(description="Delete all comments from a latex file.")
parser.add_argument('fname', metavar='f', type=str, help="The file name of the latex file.")
args = parser.parse_args()

latexFname = args.fname
with open(latexFname, 'r') as fin, open("nocomment_"+latexFname, 'w') as fout:
    for line in fin:
        i = line.find("%")
        if i >= 0 and (i == 0 or line[i-1] != "\\"):
            line = line[:i]+'\n'
        fout.write(line)
