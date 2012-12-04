#!/usr/local/bin/python
from __future__ import division
import sys, os, re, string, time, math, signal, gtk, gtktlb, traceback, optparse
import tty, termios, fcntl, select, struct
from math import *
from types import *
from utils import *
from gtk import gdk
from gtktlb import *
from cPickle import *
from serial import *

verbose=0
impotent=1

protolog=file('balctl.log1', 'a')
print>>protolog, "Startup %d" % os.getpid()

def header_reader(f):
    def getl():
        while 1:
            l=f.readline()
            if l=='': return None
            l=re.sub(r'//.*$', '', l)
            if re.match('\s*$', l): continue
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

def make_cstruct(l):
    
    fmtstr=''
    elems=[]
        
    for member, type in l:
        if type=='float':
            fmtstr += 'f'
        elif type=='uint8_t':
            fmtstr += 'B'
        else:
            raise ValueError, "Unknown type %s" % type
        elems.append(member)

    return fmtstr, tuple(elems)

bal_structs = header_reader(file('balconf.h'))
bal_config = make_cstruct(bal_structs['bal_config'])
bal_drive = make_cstruct(bal_structs['bal_drive'])
bal_state = make_cstruct(bal_structs['bal_state'])

#print bal_state

class cstruct:
    def __init__(self, typeinfo):
        self.fmt, self.elems = typeinfo

        self.from_str( '\000' * self.sizeof())

    def sizeof(self):
        return struct.calcsize(self.fmt)

    def from_str(self, str):

        if len(str) != self.sizeof(): raise ValueError

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

class bal_conn:
    def __init__(self, f):
        self.f=f
        self.rdbuf=''
        self.responses={}
        self.bal_active=0

    def read_pkt(self):

        rable, wable, xable = select.select([self.f.fileno()], [], [], 0.0)
        if not rable: return None
        
        self.rdbuf += os.read(self.f.fileno(), 8192)
        
        while len(self.rdbuf)>0:
            cmd=self.rdbuf[0]
            if cmd=='r':
                conf=cstruct(bal_config)
                end=1+conf.sizeof()
                if len(self.rdbuf) < end: return
                conf.from_str(self.rdbuf[1:end])
                print>>protolog, '>', charname(self.rdbuf[0:end]), "r packet", conf
                self.rdbuf=self.rdbuf[end:]
                return bal_r_response(conf)

            elif cmd=='c':
                st=cstruct(bal_state)
                end=1+st.sizeof()
                if len(self.rdbuf) < end: return
                st.from_str(self.rdbuf[1:end])
                print>>protolog, '>', charname(self.rdbuf[0:end]), "c packet", st
                self.rdbuf=self.rdbuf[end:]
                return bal_c_response(st)

            elif cmd=='!':
                end=self.rdbuf.find('\n')
                if end<0: return
                print>>protolog, '>', charname(self.rdbuf[0:end])
                end+=1
                self.rdbuf=self.rdbuf[end:]

            elif cmd=='w':
                end=2
                if len(self.rdbuf) < end: return
                ok = self.rdbuf[1]
                print>>protolog, '>', charname(self.rdbuf[0:end]), "w packet", ok
                self.rdbuf=self.rdbuf[end:]
                return bal_w_response(ok=='1' and 1 or 0)

            elif cmd=='d':
                end=2
                if len(self.rdbuf) < end: return
                ok = self.rdbuf[1]
                print>>protolog, '>', charname(self.rdbuf[0:end]), "d packet", ok
                self.rdbuf=self.rdbuf[end:]
                return bal_d_response(ok=='1' and 1 or 0)

            elif cmd=='*':
                end=5
                if len(self.rdbuf) < end: return
                if self.rdbuf[0:end] == '*a7sC':
                    self.bal_active=1
                print>>protolog, '>', charname(self.rdbuf[0:end]), "active", self.bal_active
                self.rdbuf=self.rdbuf[end:]

            else:
                print>>protolog, '>', charname(self.rdbuf[0:1]),"Bad packet"
                self.rdbuf=self.rdbuf[1:]
    
    def send_pkt(self, data):
        if self.bal_active:
            print>>protolog, '<', charname(data)
            self.f.write(data)
    
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

        add_parm('p_gain', "P Gain", 0.0, 20.0)
        add_parm('d_gain', "D Gain", 0.0, 10.0)
        add_parm('i_gain', "I Gain", 0.0, 20.0)
        add_parm('hard_speed_lim', "Hard Speed Lim", 0.0, 1.0)
        add_parm('fwd_speed_lim', "Fwd Speed Lim", 0.01, 1.2)
        add_parm('rev_speed_lim', "Rev Speed Lim", 0.01, 1.2)
        add_parm('pitch_bias', "Pitch Bias", -0.4, 0.4)

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

        add_drive('speed_targ', "Speed", -0.2, 0.2)
        add_drive('steering_targ', "Steering", -0.8, 0.8)

        self.read_parms=None

        gtk.timeout_add(50, self.periodic)

        BasicApp.__init__(self, gtkvb(parmbox, drivebox))

    def drive_changed(self, adj, key):

        self.send_drive()

    def send_drive(self):

        if impotent:
            print "Not writing drive, impotent"
            return

        conf=cstruct(bal_drive)
        for key, adj in self.adjs.items():
            if hasattr(conf, key):
                setattr(conf, key, adj.value)
        
        print>>protolog, "<", str(conf)

        self.conn.send_pkt('D' + conf.as_str())

    def parm_changed(self, adj, key):
        if verbose: print key,"=",adj.value

        if self.read_parms is None:
            print "Not writing config, since parms not valid"
            return

        if impotent:
            print "Not writing config, impotent"
            return
        
        if getattr(self.read_parms, key) == adj.value:
            print "(Not changed)"
            return

        conf=cstruct(bal_config)
        for key, adj in self.adjs.items():
            if hasattr(conf, key):
                setattr(conf, key, adj.value)

        print>>protolog, "<", str(conf)

        self.conn.send_pkt('W' + conf.as_str())
        self.conn.send_pkt('R')

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

        if self.read_parms is None:
            self.conn.send_pkt('R')
        else:
            if 0:
                self.send_drive()
            if 1:
                self.conn.send_pkt('C')

        while 1:
            p=self.conn.read_pkt()
            if p is None: break
            if isinstance(p, bal_w_response):
                print>>protolog, "W response: ", p.ok
            elif isinstance(p, bal_d_response):
                print>>protolog, "D response: ", p.ok
            elif isinstance(p, bal_r_response):
                self.set_parms(p.conf)
            elif isinstance(p, bal_c_response):
                self.set_state(p.state)
            else:
                raise ValueError, repr(p)

        return 1


def main():
    scooterlog=open_serial('/dev/scooterlog', baud=115200, nonblock=1)
    conn=bal_conn(scooterlog)
    BalApp(conn)
    gtk.mainloop()

if __name__ == '__main__': main()
    
