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
         'lmot_level', 'lmot_leveli',
         'rmot_level', 'rmot_leveli',
         'adc_batt_voltage','adc_pitch_rate','adc_fwd_accel', 'adc_steering', 'adc_acceltemp',
         'cmd','balance_torque','cur_speed_est', 'cur_cg_speed_est')

tcnt_mult = 1024.0 / 16.0e6
adc_of=None
of=None
tsvalues={}
last_tcnt=0

def startup():
    global init_time, ofname, of, adc_ofname, adc_of
    global last_tcnt, realtime, tsvalues, last_adc_realtime
    init_time = time.time()

    ofname="cap%04d" % (int(init_time)%10000)
    of=open(ofname,'w')
    adc_ofname="adc%04d" % (int(init_time)%10000)
    adc_of=open(adc_ofname,'w')

    for i,name in indexed(gporder):
        print "plot \"%s\" using 1:%d title \"%s\"" % (ofname, i+1, name)
    print "plot \"%s\" using 0:2 title \"ADC 1\"" % (adc_ofname)
    print "plot \"%s\" using 1:2 title \"ADC 1\"" % (adc_ofname)
    print "plot \"%s\" using 1:3 title \"TCNT\"" % (adc_ofname)
    sys.stdout.flush()

    last_tcnt=0
    last_adc_realtime=0.0
    realtime=0.0
    tsvalues={}

def main():
    global init_time, ofname, of, adc_ofname, adc_of
    global last_tcnt, realtime, tsvalues, last_adc_realtime

    last_good=''
    buf=''
    bufi = 0
    while 1:
        buf += os.read(avrtty.fileno(), 8192)
        #print>>sys.stderr, len(buf)

        while bufi < len(buf):
            hdr=buf[bufi]
            if hdr=='b':
                fmt=('<fffffffff',('interval','lpf_angle','lpf_angrate', 'lpf_angintegral', 'softstart', 'cmd', 'balance_torque',
                                   'cur_speed_est', 'cur_cg_speed_est'))
            elif hdr=='&':
                fmt=('<B', ('tcnt',))
            elif hdr=='r':
                fmt=('<fff',('in_rate','kalman_angle','kalman_angrate'))
            elif hdr=='c':
                fmt=('<fffff',('adc_batt_voltage','adc_pitch_rate','adc_fwd_accel', 'adc_steering', 'adc_acceltemp'))
            elif hdr=='a':
                fmt=('<fff',('in_y','in_angle','rate_bias'))
            elif hdr=='m':
                fmt=('<fhfh',('lmot_level','lmot_leveli', 'rmot_level','rmot_leveli'))
            elif hdr=='l':
                if bufi+2 >= len(buf): break
                n_adc_log,=struct.unpack('B',buf[bufi+1:bufi+2])
                bufend=bufi + 2+ 4*n_adc_log
                if len(buf)<bufend: break

                bi=bufi+2
                dt = (realtime-last_adc_realtime) / n_adc_log
                for i in range(n_adc_log):
                    adc_sample, = struct.unpack('H', buf[bi : bi+2])
                    adc_tcnt, = struct.unpack('H', buf[bi+2 : bi+4])
                    print>>adc_of, last_adc_realtime + i*dt, adc_sample, adc_tcnt
                    bi+=4
                last_adc_realtime=realtime
                bufi=bufend
                continue


                adc_log=struct.unpack('%dH'%n_adc_log, buf[bufi+2:bufend])
            
            elif hdr=='!':
                lfpos=buf.find("\n", bufi);
                if lfpos>=0:
                    comment=buf[bufi+1:lfpos]
                    if re.match(r'bal ', comment):
                        print>>sys.stderr, "Got header"
                        startup()
                    print>>sys.stderr, comment
                    bufi = lfpos+1
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
                bufi = bufi+1
                continue
            else:
                print>>sys.stderr, "Garbage character `%s' after %s" % (charname(hdr), last_good)
                bufi = bufi+1
                continue

            minlen = struct.calcsize(fmt[0])
            if len(buf)<bufi+minlen+1:
                break

            #print hdr,fmt[0],minlen

            if 1:
                argnames=fmt[1]
                values=struct.unpack(fmt[0],buf[bufi+1:bufi+minlen+1])
                #print>>sys.stderr, fmt[0], "<", minlen+1
                for k,v in zip(argnames,values):
                    #print "  ",k,"=",v
                    tsvalues[k]=v

                last_good=fmt[0]+'('+','.join(argnames)+')'

            bufi += minlen+1

        buf=buf[bufi:]
        bufi=0

main()
