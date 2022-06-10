# Wood PLA layer temp randomizer
# Taken from https://www.reddit.com/r/3Dprinting/comments/gp5vrp/comment/frk5gil/

import argparse
import re
import random

Zpattern = re.compile("^[^;]*(G(0|1)).*Z.*$")
Tpattern = re.compile("^[^;]*(M104|M109).*")

parser = argparse.ArgumentParser(description='Randomize layer temperatures for wood PLA.')

parser.add_argument('input', type=argparse.FileType('r'), help='g-code file to be processed')
parser.add_argument('output', type=argparse.FileType('w+'), help='g-code file output')
parser.add_argument('--tmin', default=200, help='lower temperature bound' )
parser.add_argument('--tmax', default=220, help='upper temperature bound' )

args = vars(parser.parse_args())

layerchange = False
lines = args['input'].read().splitlines()

for line in lines:
    if layerchange == True:
        layerchange = False;
        if not Tpattern.match (line):
            args['output'].write("M104 S%d\n" % random.randint(args['tmin'], args['tmax']))
    if layerchange == False and Zpattern.match ( line ):
        layerchange = True
    args['output'].write("%s\n" % line)

args['output'].close()
