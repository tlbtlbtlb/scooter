#!/usr/local/bin/python
from __future__ import division
import sys, os, re, string, time, math, signal
from utils import *

def scan_log(f):
    trace=[]
    allkeys={}
    allkeys_ordered=[]
    while 1:
        l=f.readline()
        if l=='': break
        
        m=re.search(r'c packet {(.*)}', l)
        if m:
            pieces=m.group(1).split(' ')
            d={}
            for kvs in pieces:
                k,v = kvs.split('=')
                d[k]=v
                if k not in allkeys:
                    allkeys[k]=1
                    allkeys_ordered.append(k)
            trace.append(d)
            
    for ki,name in enumerate(allkeys_ordered):
        print 'plot "gp" using 1:%d title "%s"' % (ki+1, name)

    gpf=open('gp', 'w')

    for t in trace:
        for name in allkeys_ordered:
            print>>gpf, t[name],
        print>>gpf
            
            

scan_log(open('balctl.log1'))
