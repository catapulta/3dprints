# Makelangelo Output Editor for Ender 3
# Based on https://www.reddit.com/r/3Dprinting/comments/gp5vrp/comment/frk5gil/

import argparse
import re


Y_OFFSET = 0#45
X_OFFSET = 0#25

X_BED = 235
Y_BED = 235-57

PEN_UP = 0.5

G28pattern = re.compile("^[^;]*G28.*")
Mpattern = re.compile("^[^;]*M280.*")
Xpattern = re.compile("^([^;]*G[0|1].*X)([+-]?[0-9]+\.?[0-9]*|\.[0-9]+)(.*)?$")
Ypattern = re.compile("^([^;]*G[0|1].*Y)([+-]?[0-9]+\.?[0-9]*|\.[0-9]+)(.*)?")

parser = argparse.ArgumentParser(description='Randomize layer temperatures for wood PLA.')

parser.add_argument('input', type=argparse.FileType('r'), help='g-code file to be processed')
parser.add_argument('output', type=argparse.FileType('w+'), help='g-code file output')

args = vars(parser.parse_args())

lines = args['input'].read().splitlines()

for line in lines:
    if G28pattern.match(line):
        args['output'].write("G28 ; Auto-home\nM107 ; Fan off\nG0 Z2.0 ; Pen up\nT1 ; Plotter tool\n")
    elif Mpattern.match(line) and "S30" in line:
        args['output'].write(f"G0 Z0.0 ; Pen down\n")
    elif Mpattern.match(line) and "S90" in line:
        args['output'].write(f"G0 Z{PEN_UP:.3f} ; Pen up\n")
    elif Xpattern.match(line) or Ypattern.match(line):
        matched = Xpattern.match(line)
        x_coord = float(matched.group(2)) + X_BED/2 + X_OFFSET
        # trust there's no ~~~ (used to avoid \1x_coord be read as eg \123)
        line = re.sub(Xpattern, r"\1~~~" + f"{x_coord:.3f}" + r"\3", line).replace("~~~", "")
        
        matched = Ypattern.match(line)
        y_coord = float(matched.group(2)) + Y_BED/2 + Y_OFFSET
        # trust there's no ~~~ (used to avoid \1y_coord be read as eg \123)
        line = re.sub(Ypattern, r"\1~~~" + f"{y_coord:.3f}" + r"\3", line).replace("~~~", "")

        args['output'].write(line + "\n")
    else:
        args['output'].write("%s\n" % line)

args['output'].write("G28 ; Auto-home\n")

args['output'].close()
