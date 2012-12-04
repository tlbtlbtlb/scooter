#!/usr/local/bin/python

import sys, os, re, string, time, math, socket
import tty, termios, fcntl, select, struct
from math import *
from types import *
from random import *
from utils import *

def ts():
    return time.time()-init_time

avrtty=open('/dev/scooterlog','r+',0)
fcntl.fcntl(avrtty, os.O_NONBLOCK, 1)
tty.setraw(avrtty)
attrs=tty.tcgetattr(avrtty)
attrs[4]=attrs[5]=115200 # baud rate
attrs[2]=termios.CS8|termios.CREAD
tty.tcsetattr(avrtty, termios.TCSANOW, attrs)

# resets it
#os.system("uisp -v -dprog=stk500 -dpart=ATmega32 -dserial=/dev/cuaa0 -dspeed=115200  --rd_vtg >/dev/null")

init_time=time.time()

gporder=('realtime','tcnt','interval','lpf_angle','lpf_angrate',
         'lpf_angintegral', 'softstart',
         'in_rate','kalman_angle','kalman_angrate',
         'in_y','in_angle','rate_bias',
         'mot_level', 'mot_leveli',
         'adc0','adc1','adc2',
         'cmd','balance_torque','cur_speed_est', 'batt_voltage')

tcnt_mult = 1024.0 / 16.0e6
adc_of=None
of=None
tsvalues={}

def startup():
    global init_time, ofname, of, adc_ofname, adc_of
    global last_tcnt, realtime, tsvalues
    init_time = time.time()

    ofname="cap%04d" % (int(init_time)%10000)
    of=open(ofname,'w')
    adc_ofname="adc%04d" % (int(init_time)%10000)
    adc_of=open(adc_ofname,'w')

    for i,name in indexed(gporder):
        print>>sys.stderr, "plot \"%s\" using 1:%d title \"%s\"" % (ofname, i+1, name)
    print>>sys.stderr, "plot \"%s\" using 0:2 title \"ADC 1\"" % (adc_ofname)
    print>>sys.stderr, "plot \"%s\" using 1:2 title \"ADC 1\"" % (adc_ofname)

    last_tcnt=0
    realtime=0.0
    tsvalues={}

def main():
    global init_time, ofname, of, adc_ofname, adc_of
    global last_tcnt, realtime, tsvalues

    last_good=''
    buf=''
    while 1:
        buf += os.read(avrtty.fileno(), 128)
        #print>>sys.stderr, len(buf)

        while len(buf):
            hdr=buf[0]
            if hdr=='b':
                fmt=('<ffffffff',('interval','lpf_angle','lpf_angrate', 'lpf_angintegral', 'softstart', 'cmd', 'balance_torque',
                                   'cur_speed_est'))
            elif hdr=='&':
                fmt=('<B', ('tcnt',))
            elif hdr=='r':
                fmt=('<fff',('in_rate','kalman_angle','kalman_angrate'))
            elif hdr=='c':
                fmt=('<fff',('adc0','adc1','adc2'))
            elif hdr=='a':
                fmt=('<fff',('in_y','in_angle','rate_bias'))
            elif hdr=='m':
                fmt=('<fh',('mot_level','mot_leveli'))
            elif hdr=='l':
                if len(buf)<2: break
                n_adc_log,=struct.unpack('B',buf[1:2])
                bufend=2+ 2*n_adc_log
                if len(buf)<bufend: break
                adc_log=struct.unpack('%dH'%n_adc_log, buf[2:bufend])
                if adc_of:
                    for x in adc_log:
                        print>>adc_of, realtime, x
                buf=buf[bufend:]
                continue
            elif hdr=='!':
                lfpos=buf.find("\n");
                if lfpos>=0:
                    comment=buf[1:lfpos]
                    if re.match(r'unicycle ', comment):
                        print>>sys.stderr, "Got header"
                        startup()
                    print>>sys.stderr, comment
                    buf=buf[lfpos+1:]
                    continue
                else:
                    break
            elif hdr==';':
                if 'tcnt' in tsvalues:
                    ticks=(tsvalues['tcnt'] - last_tcnt) % 256
                    realtime += ticks * tcnt_mult
                    last_tcnt = tsvalues['tcnt']

                    tsvalues['hosttime']=time.time() - init_time
                    tsvalues['realtime']=realtime
                    for k in gporder:
                        if k in tsvalues:
                            of.write(" %g"%(tsvalues[k]))
                        else:
                            of.write(" 0.0")
                    of.write("\n")
                    of.flush()
                    sys.stderr.write('.')

                    #print tsvalues
                    #tsvalues={}
                buf=buf[1:]
                continue
            else:
                print>>sys.stderr, "Garbage character `%s' after %s" % (charname(hdr), last_good)
                buf=buf[1:]
                continue

            minlen = struct.calcsize(fmt[0])
            if len(buf)<minlen+1:
                break

            #print hdr,fmt[0],minlen

            if 1:
                argnames=fmt[1]
                values=struct.unpack(fmt[0],buf[1:minlen+1])
                #print>>sys.stderr, fmt[0], "<", minlen+1
                for k,v in zip(argnames,values):
                    #print "  ",k,"=",v
                    tsvalues[k]=v

                last_good=fmt[0]+'('+','.join(argnames)+')'

            buf=buf[minlen+1:]


main()
