#!/usr/local/bin/python
# COPYRIGHT
from __future__ import division
import sys, os, re, string, time, math, signal, traceback, optparse
import tty, termios, fcntl, select, struct, copy
import gtktlb
import builtlib
from boost_packet import *
from math import *
from types import *
from utils import *
from gtktlb import *
from cPickle import *
from serial import *

#gdbme()

verbose=0
impotent=0

#debugentity.incr_verbose('/dev/scooterlog')
#debugentity.incr_verbose('/dev/scooterlog.pkt')
#debugentity.incr_verbose('/dev/scooterlog.pkt')

protolog=file('balctl.log1', 'a')
print>>protolog, "Startup %d" % os.getpid()

def header_reader(f):
    def getl():
        while 1:
            l=f.readline()
            if l=='': return None
            l=re.sub(r'//.*$', '', l)
            if re.match('\s*$', l): continue
            if re.match('#', l): continue
            return l
    
    structs={}
    while 1:
        l=getl()
        if l is None: break
        m=re.match(r'\s*struct (\w+) {', l)
        if m:
            structname=m.group(1)
            structdef=[]
            while 1:
                l=getl()
                m=re.match(r'\s*struct\s+(\w+)\s+(\w+)\s*;\s*$', l)
                if m:
                    type=m.group(1)
                    member=m.group(2)
                    for m1,t1 in structs[type]:
                        structdef.append((member + '.' + m1,t1))
                else:
                    m=re.match(r'\s*(\w+)\s+(\w+)\s*;\s*$', l)
                    if m:
                        type=m.group(1)
                        member=m.group(2)
                        structdef.append((member,type))
                    else:
                        m=re.match(r'\s*};\s*$', l)
                        if m:
                            break
                        else:
                            raise ValueError, "Bad Line %s" % l
                        
            #print structname, structdef
            structs[structname]=structdef
        else:
            raise ValueError, "Bad Line %s" % l
    return structs

def make_cstruct(l, name):
    
    fmtstr=''
    elems=[]
        
    for member, type in l:
        if type=='float':
            fmtstr += 'f'
        elif type=='uint8_t':
            fmtstr += 'B'
        elif type=='uint16_t':
            fmtstr += 'H'
        elif type=='uint32_t':
            fmtstr += 'L'
        else:
            raise ValueError, "Unknown type %s" % type
        elems.append(member)

    print name,"size:",struct.calcsize(fmtstr)

    return fmtstr, tuple(elems)

bal_structs = header_reader(os.popen('cd ../balfw && gmake dump-balconf', 'r'))
bal_config = make_cstruct(bal_structs['bal_config'], 'bal_config')
bal_drive = make_cstruct(bal_structs['bal_drive'], 'bal_drive')
bal_state = make_cstruct(bal_structs['bal_state'], 'bal_state')
bal_hwadj = make_cstruct(bal_structs['bal_hwadj'], 'bal_hwadj')
print bal_state[1]

#print bal_state

class cstruct:
    def __init__(self, typeinfo):
        self.fmt, self.elems = typeinfo

        self.from_str( '\000' * self.sizeof())

    def sizeof(self):
        return struct.calcsize(self.fmt)

    def from_str(self, str):

        if len(str) != self.sizeof():
            raise ValueError, "Expected %d bytes, got %d" % (self.sizeof(), len(str))

        values=struct.unpack(self.fmt, str)
        for name,value in zip(self.elems, values):
            setattr(self, name, value)

    def as_str(self):

        values=[]
        for name in self.elems:
            values.append(getattr(self, name))
        #print values
        return struct.pack(self.fmt, *values)

    def __str__(self):
        return '{' + ' '.join([
            "%s=%g" % (name, getattr(self, name)) for name in self.elems
            ]) + '}'

class bal_c_response:
    def __init__(self, state):
        self.timestamp=time.time()
        self.state=state

class bal_r_response:
    def __init__(self, conf):
        self.timestamp=time.time()
        self.conf=conf

class bal_w_response:
    def __init__(self, ok):
        self.timestamp=time.time()
        self.ok=ok

class bal_d_response:
    def __init__(self, ok):
        self.timestamp=time.time()
        self.ok=ok

    
class BalApp(BasicApp):
    def __init__(self, conn):
        self.conn=conn
        self.grfp=None

        parmbox=gtk.HBox()

        self.adjs={}
        
        def add_parm(key, name, lb, ub, incr=None, page=None):
            
            if incr is None: incr=(ub-lb)/100
            if page is None: page=incr*10
            
            adj=gtk.Adjustment(0.0, lb, ub, incr, page, 0.0)
            self.adjs[key]=adj
            scale=gtk.VScale(adj)
            scale.set_digits(2)
            scale.set_size_request(25,170)
            adj.connect('value_changed', self.parm_changed, key)

            parmbox.add(add_gtk_frame(name, scale))

        add_parm('p_gain', "P Gain", 0.0, 10.0)
        add_parm('d_gain', "D Gain", 0.0, 1.0)
        add_parm('i_gain', "I Gain", 0.0, 10.0)
        add_parm('yaw_steer_gain', "Yaw Steer Gain", 0.0, 0.3)
        add_parm('hard_speed_lim', "Hard Speed Lim", 0.0, 1.0)
        add_parm('fwd_speed_lim', "Fwd Speed Lim", 0.01, 1.2)
        add_parm('rev_speed_lim', "Rev Speed Lim", 0.01, 1.2)
        if 0: add_parm('fwd_accel_bias', "Fwd Accel Bias", -0.1, 0.1)
        add_parm('crossover_boost', "Crossover Boost", 0.0, 0.05)
        add_parm('fwd_accel_coupling', "Fwd Accel Coupling", -2.5, 2.5)
        add_parm('motor_torque_factor', "Motor torque factor", 0.0, 1.5)

        drivebox=gtk.HBox()

        def add_drive(key, name, lb, ub, incr=None, page=None):
            
            if incr is None: incr=(ub-lb)/100
            if page is None: page=incr*10
            
            adj=gtk.Adjustment(0.0, lb, ub, incr, page, 0.0)
            self.adjs[key]=adj
            scale=gtk.VScale(adj)
            scale.set_digits(2)
            scale.set_size_request(25,170)
            adj.connect('value_changed', self.drive_changed, key)

            drivebox.add(add_gtk_frame(name, scale))

        add_drive('speed_targ', "Speed", -0.25, 0.25)
        add_drive('steering_targ', "Steering", -0.4, 0.4)
        
        self.read_parms=None
        self.bal_active=0
        self.adcdumpfiles=None
        
        gobject.timeout_add(30, self.periodic)

        BasicApp.__init__(self, gtkvb(parmbox, drivebox))

    def drive_changed(self, adj, key):

        self.send_drive()

    def send_drive(self):

        if impotent:
            print "Not writing drive, impotent"
            return
        print "Writing drive"

        conf=cstruct(bal_drive)
        for key, adj in self.adjs.items():
            if hasattr(conf, key):
                setattr(conf, key, adj.value)
        
        print>>protolog, "< D", str(conf)

        self.conn.wr_pkt('D' + conf.as_str())

    def parm_changed(self, adj, key):
        if 1 or verbose: print "parm changed", key, "=", adj.value

        if self.read_parms is None:
            print "Not writing config, since parms not valid"
            return

        if impotent:
            print "Not writing config, impotent"
            return
        
        if getattr(self.read_parms, key) == adj.value:
            #print "(Not changed)"
            return

        print "Writing config because %s changed %g to %g" % (key, getattr(self.read_parms, key), adj.value)

        conf = copy.copy(self.read_parms)
        for key, adj in self.adjs.items():
            if hasattr(conf, key):
                setattr(conf, key, adj.value)

        print>>protolog, "< W", str(conf)

        self.conn.wr_pkt('W' + conf.as_str())
        self.conn.wr_pkt('R')

    def set_parms(self, conf):
        self.read_parms=conf
        for key, adj in self.adjs.items():
            if hasattr(conf, key):
                adj.set_value(getattr(conf, key))

    def set_state(self, state):
        if self.grfp is None:
            init_time = time.time()
            ofname="cap%04d" % (int(init_time)%10000)
            self.grfp=open(ofname, 'w')
            savefp=open(ofname+".gnuplot", 'w+')
            
            for i,name in indexed(state.elems):
                print>>savefp, "plot \"%s\" using %d:%d title \"%s\"" % (ofname, (i==0 and [0] or [1])[0], i+1, name)
            print "Output in %s" % ofname+".gnuplot"
        
        print>>self.grfp, ' '.join(["%g" % getattr(state, name) for name in state.elems])
            

    def periodic(self):

        print>>protolog, "Periodic ",time.time()

        if self.bal_active:
            if 0:
                self.send_drive()
            if 1:
                print>>protolog, "< C";
                self.conn.wr_pkt('C')
            if 1:
                print>>protolog, "< H";
                self.conn.wr_pkt('H')
            if self.read_parms is None:
                print>>protolog, "< R";
                self.conn.wr_pkt('R')

        while 1:
            p=self.conn.rd_pkt()
            if p is None: break
            self.handle_pkt(p)

        return 1

    def handle_pkt(self, p):
        cmd=p[0]
        if cmd=='r':
            conf=cstruct(bal_config)
            conf.from_str(p[1:])
            print>>protolog, '>', charname(p), "r packet", conf
            self.set_parms(conf)

        elif cmd=='c':
            st=cstruct(bal_state)
            st.from_str(p[1:])
            print>>protolog, '>', charname(p), "c packet", st
            self.set_state(st)

        elif cmd=='h':
            st=cstruct(bal_hwadj)
            st.from_str(p[1:])
            print>>protolog, '>', charname(p), "h packet", st
            if 0: print st

        elif cmd=='!':
            print>>protolog, '>', charname(p)
            if re.match(r'!bal \d+\.\d+', p):
                self.bal_active=1

        elif cmd=='w':
            if len(p) != 2: return
            ok = ord(p[1])
            print>>protolog, '>', charname(p), "w packet", ok

        elif cmd=='d':
            if len(p) != 2: return
            ok = ord(p[1])
            print>>protolog, '>', charname(p), "d packet", ok

        elif cmd=='l':
            if not self.adcdumpfiles:
                self.adcdumpfiles=[file('adc%d.dump' % i, 'w') for i in range(0x40)]

            raw = struct.unpack('H16H', p[1:])
            starti=raw[0]
            data=raw[1:]

            timei=starti
            for datum in data:
                adci = (datum>>10)&0x3f
                adcval = (datum>>0)&0x3ff
                self.adcdumpfiles[adci].write('%d %d\n' % (timei, adcval))
                timei+=1

        else:
            print>>protolog, '>', charname(p), "Unknown packet"

def main():
    scooterlog = tty_serial('/dev/scooterlog')
    scooterlog.set_bitrate(115200)

    conn=packet_conn(scooterlog)

    BalApp(conn)
    gtk.main()

if __name__ == '__main__': main()
    
