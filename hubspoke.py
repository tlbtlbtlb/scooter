#!/usr/local/bin/python
from __future__ import division
from utils import *
from math import *

n_hub=28
n_rim=28

hub_pos = [(1.25*cos(i*pi*2/n_hub),
            1.25*sin(i*pi*2/n_hub),
            (i%2)*2-1) for i in range(n_hub)]
           
rim_pos = [(7*cos(i*pi*2/n_rim),
            7*sin(i*pi*2/n_rim),
            0) for i in range(n_rim)]

#print "hub",hub_pos
#print "rim",rim_pos

print "%!PS-Adobe 2.0"
print "300 300 translate 36 36 scale"
print "0.062 setlinewidth"

def conv_3d(x, y, z):
    return (x+z*0, y+z*0)

def draw_line(a, b):
    ax,ay=conv_3d(*a)
    bx,by=conv_3d(*b)
    if b[2]<0:
        print "1 0 0 setrgbcolor"
    else:
        print "0 0 1 setrgbcolor"
    print "%g %g moveto %g %g lineto stroke" % (ax, ay, bx, by)
    

print "0 0 0 setrgbcolor"
for i in range(len(rim_pos)):
    draw_line(rim_pos[i], rim_pos[(i+1)%len(rim_pos)])
for i in range(len(hub_pos)):
    draw_line(hub_pos[i], hub_pos[(i+2)%len(hub_pos)])
for i in range(0,len(hub_pos),2):
    draw_line(hub_pos[i], hub_pos[(i+1)%len(hub_pos)])

for i in range(len(rim_pos)):
    draw_line(rim_pos[i], hub_pos[(n_hub*i//n_rim + [-5, 5][(i//2)%2]) % n_hub])

print "showpage"
