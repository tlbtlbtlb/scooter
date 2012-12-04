#!/usr/local/bin/python
from __future__ import division
import os,sys
os.system('ln -sf ../../robot/build.%s .' % sys.platform)
sys.path += ['../../robot']
from anycore.full import *
from anygui.full import *
from anygui.gl_extras import *
from math import hypot,atan2,atan
from gtk import gtkgl, gdkgl
from OpenGL import GL, GLU
import PIL.Image

gl_use_blend=1

def rotate_2d(x, y, angle):
    return (x*cos(angle) - y*sin(angle), y*cos(angle) + x*sin(angle))

def speed_str(meters_per_sec):
    return "%0.2f m/S (%0.2f mph)" % (meters_per_sec, meters_per_sec * 3600.0 / 1607.0)

def distance_str(meters):
    return "%0.2f m (%0.2f feet)" % (meters, meters / 0.0254 / 12)

def signum(x):
    if x>0: return 1
    if x<0: return -1
    return 0

gravity = 9.8  # N / kg

# Physics model of scooter. Call run(dt) where dt is the time step, a few milliseconds
class scooter(model):
    def __init__(self):
        model.__init__(self)

        self.paused=0
        self.measuremode=0
        self.fallen=0
        
        self.rider_height=1.8 # cg height
        self.handle_len=1.2
        self.wheel_rad=10.0/39.0
        self.arm_len=1.2

        self.rider_mass = 100.0
        self.base_mass = 30.0
        self.base_moi = 2.0   # base moment of inertia in kg-m^2. Not be be confused with baisse_moi

        self.rider_springrate=1000.0 # N/m
        self.rider_damping=200.0 # N/m/S

        self.max_motor_power=4000
        self.max_motor_regen=6000
        self.max_motor_torque=300
        self.rider_goal_speed=0.0
        self.speed_lim=4.5
        self.bus_speed_lim=7.0 # m/s max
        self.bus_torque_constant=100 # Nm per m/s

        self.kp=600
        self.kd=100

        self.rider_eff_area = 1.0 # m^2 * drag coeff
        self.wind_speed=0.0

        self.ground_incline=0.0

        r=random.Random(124)
        self.terrain_ampl=0.00
        tx=-500.0
        self.terrain=[(-1e6,0.0)]
        while tx < 500:
            xspan=max(0.20,min(1.5,r.expovariate(0.2)))
            lasty=self.terrain[-1][1]
            tx += xspan
            self.terrain.append((tx,min(lasty+3*xspan, max(lasty-3*xspan,
                                                             r.normalvariate(0.0, 1.0)))))
        self.terrain.append([1e6,0.0])
        self.terrain=array(self.terrain, 'd')

        self.restart()
        
        self.trace_motor_torque=tracemonitor(self,'motor_torque')
        self.trace_motor_power=tracemonitor(self,'motor_power')
        self.trace_tilt_angle=tracemonitor(self,'tilt_angle')


    def restart(self):
        self.fallen=0
        self.simtime=0.0

        self.motor_torque=0.0

        self.base_xpos=0.10
        self.base_dxpos=0.0

        self.base_ypos=0.0
        self.base_dypos=0.0
        self.curslope=0.0

        self.rider_xpos=0.1
        self.rider_dxpos=0.0

        self.rider_dypos=0.0

        self.target_arm_ext=0.0

        self.tilt_angle=0.0
        self.dtilt_angle=0.0

        self.tiltback_angle=0.0
        self.tiltback_integral=0.0

        self.run_mech(0.0)
        self.run_balance(0.0)
        self.run_rider(0.0)
        self.run_measure(0.0)

    # Create a GtkAdjustment on a particular attribute
    def adjuster(self, key, lb, ub, step_increment=0.01, page_increment=0.05, page_size=0.0):
        val=getattr(self,key)
        adj=gtk.Adjustment(val, lb, ub, step_increment, page_increment, page_size)
        adj.connect('value_changed', self.adjuster_value_changed, key)
        return adj

    def adjuster_value_changed(self, adj, key):
        setattr(self,key,adj.value)
        self.changed(key)

    def run(self, dt):
        if self.fallen or self.paused: return
        self.run_mech(dt)
        self.run_balance(dt)
        self.run_rider(dt)
        self.run_measure(dt)
        self.simtime += dt
        if self.tilt_angle > 1.55 or self.tilt_angle <-1.55:
            self.fallen=1
        self.changed('run')

    def run_mech(self, dt):
        ti=bsearch_le(self.terrain[:,0], self.base_xpos)
        slope=self.terrain_ampl*(self.terrain[ti+1][1] - self.terrain[ti][1]) / (self.terrain[ti+1][0] - self.terrain[ti][0])
        if slope != self.curslope:
            vel=sqrt(self.base_dypos**2 + self.base_dxpos**2)
            if self.base_dxpos<0: vel=-vel
            angle=atan(slope)
            self.base_dxpos = cos(angle)*vel
            self.base_dypos = sin(angle)*vel
            self.curslope=slope
        ground_ypos = (self.terrain_ampl*self.terrain[ti][1] + slope * (self.base_xpos - self.terrain[ti][0]))
        self.base_ypos = self.wheel_rad + ground_ypos
        self.rider_ypos = self.base_ypos + sqrt(self.rider_height**2 - (self.rider_xpos-self.base_xpos)**2)
        
        self.handle_xpos=self.base_xpos + sin(self.tilt_angle+0.18)*self.handle_len
        self.handle_ypos=self.base_ypos + cos(self.tilt_angle+0.18)*self.handle_len

        self.handle_dxpos=self.base_dxpos + self.dtilt_angle * cos(self.tilt_angle+0.18)*self.handle_len
        self.handle_dypos=self.base_dypos + self.dtilt_angle * sin(self.tilt_angle+0.18)*self.handle_len

        arm_ext = hypot(self.handle_xpos - self.rider_xpos, self.handle_ypos - self.rider_ypos)
        arm_xext = self.handle_xpos - self.rider_xpos
        arm_dxext = self.handle_dxpos - self.rider_dxpos

        rider_hand_xforce = ((arm_xext - self.target_arm_ext) * self.rider_springrate +
                             arm_dxext * self.rider_damping +
                             max(0, self.rider_springrate*10*(arm_ext - 0.98*self.arm_len)))
        
        rider_feet_xforce = (self.rider_xpos - self.base_xpos) / (self.rider_ypos) * self.rider_mass * gravity

        # drag is rho * v^2 * A
        rs=self.rider_dxpos-self.wind_speed
        if rs!=0.0:
            rider_drag_xforce = -signum(rs) * rs**2 * self.rider_eff_area * 1.3 # 1.3 kg/m^3 air density
        else:
            rider_drag_xforce = 0.0


        rider_xforce = rider_hand_xforce + rider_feet_xforce + rider_drag_xforce
        self.base_xforce = (-rider_hand_xforce - rider_feet_xforce +
                            self.motor_torque / self.wheel_rad -
                            slope * gravity * (self.base_mass + self.rider_mass))

        base_torque = -self.motor_torque - (rider_hand_xforce / (self.rider_ypos-self.base_ypos))

        self.motor_speed = self.base_dxpos / self.wheel_rad

        self.motor_power = self.motor_torque/self.wheel_rad * self.base_dxpos

        self.dtilt_angle += dt * base_torque / self.base_moi
        self.tilt_angle += dt*self.dtilt_angle

        self.rider_xpos += dt*self.rider_dxpos
        self.base_xpos += dt*self.base_dxpos

        self.rider_dxpos += dt*rider_xforce / self.rider_mass
        self.base_dxpos += dt*self.base_xforce / self.base_mass
        self.base_dypos = slope * self.base_dxpos

    def run_balance(self, dt):
        
        os=max(-1.0, min(1.0, self.base_dxpos - self.speed_lim))
        self.tiltback_integral = max(0.0, self.tiltback_integral+
                                     max(-3.0, min(0.3, self.base_dxpos - self.speed_lim))
                                     *0.2*dt)
        self.tiltback_angle=max(0.0, os*0.2)+self.tiltback_integral
        
        mt = self.kp * (self.tilt_angle+self.tiltback_angle) + self.kd * self.dtilt_angle

        maxt,mint=self.motor_torque_limits(self.base_dxpos)
        mt=min(maxt,max(mint,mt))
        self.motor_torque=mt

    def run_rider(self, dt):
        self.target_arm_ext = 0.20 + min(0.45, max(-0.30, (self.rider_dxpos +
                                                            self.motor_torque/self.wheel_rad * 0.0005 -
                                                            self.rider_goal_speed)/3.0))

    def run_measure(self, dt):
        if self.rider_goal_speed!=0.0:
            self.measuremode=0
        if self.measuremode==1:
            if self.base_dxpos * self.measure_from_dxpos <= 0:
                self.measuremode=2
        if self.measuremode==1:
            self.measure_to_xpos = self.base_xpos
            self.measure_to_dxpos = self.base_dxpos

    def start_measure(self):
        self.measuremode=1
        self.measure_from_xpos = self.base_xpos
        self.measure_from_dxpos = self.base_dxpos
        self.run_measure(0.0)

    def motor_torque_limits(self, speed):
        return (min(self.max_motor_torque,
                    self.max_motor_power/max(0.1,speed) * self.wheel_rad,
                    -self.max_motor_regen/min(-0.1,speed) * self.wheel_rad,
                    (self.bus_speed_lim - speed) * self.bus_torque_constant),
                max(-self.max_motor_torque,
                    self.max_motor_power/min(-0.1,speed) * self.wheel_rad,
                    -self.max_motor_regen/max(0.1,speed) * self.wheel_rad,
                    -(self.bus_speed_lim + speed) * self.bus_torque_constant))
                

# Record a history of values. Adapter pattern
class tracemonitor(model):
    def __init__(self, m, key, duration=5, interval=0.020):
        model.__init__(self)
        self.m=m
        self.simtime=self.m.simtime
        self.key=key
        self.duration=duration
        self.interval=interval
        self.trace=[]
        self.m.add_dependent(self.model_changed)

    def model_changed(self, what):
        if self.trace and self.trace[-1][0] > self.m.simtime:
            self.trace=[]
        self.simtime=self.m.simtime
        if self.trace and self.m.simtime - self.trace[-1][0] < 0.020:
            return
        self.trace.append((self.m.simtime, getattr(self.m, self.key)))
        self.changed()

class motor_envelope_view(BufferedDrawingArea):
    def __init__(self, m):
        BufferedDrawingArea.__init__(self)
        self.m=m
        self.m.add_dependent(self.model_changed)
        self.tm_torque = tracemonitor(m, 'motor_torque')
        self.tm_speed = tracemonitor(m, 'base_dxpos')
        self.tm_speed.add_dependent(self.model_changed)
        self.set_size_request(200,200)
        st=self.get_style().copy()
        st.bg[0]=named_color(self,'999999')
        self.set_style(st)
        self.decay_time=5.0

    def create_gcs(self):
        BufferedDrawingArea.create_gcs(self)
        self.env_gc=self.mkgc('ffffff')
        self.axis_gc=self.mkgc('cccccc')
        self.pts_gc=self.mkgc('cc0000')
        self.pts_gc.line_width=3
        self.pts_gc.cap_style=gdk.CAP_PROJECTING

    def drawit(self, pixmap, style, width, height):

        lmarg=20
        rmarg=0
        tmarg=0
        bmarg=20

        def ptx(speed):
            return int(lmarg + (width-lmarg-rmarg)/2 * (speed/8.5 + 1.0))

        def pty(torque):
            return max(-50,min(height*2,int(tmarg + (height-tmarg-bmarg)/2 * (1.0 - torque/350.0))))

        fwdl=[]
        revl=[]
        for speed in arange(-9.0, 9.0, 0.1):
            if speed==0: continue
            maxt,mint=self.m.motor_torque_limits(speed)
            if maxt < mint: continue
            fwdl.append((ptx(speed), pty(maxt)))
            revl.append((ptx(speed), pty(mint)))
        revl.reverse()

        pixmap.draw_polygon(self.env_gc, 1,
                     fwdl + revl)

        pixmap.draw_line(self.axis_gc, ptx(-15),pty(0), ptx(15),pty(0))
        pixmap.draw_line(self.axis_gc, ptx(0),pty(-300), ptx(0),pty(300))

        for ylabel in (-300, 0, 300):
            str="%+.0f Nm" % ylabel
            draw_lstring(pixmap, self, self.text_gc, 5, pty(ylabel), str, hcenter=0.0, vcenter=0.5)

        for xlabel in (-5, 0, 5):
            draw_lstring(pixmap, self, self.text_gc, ptx(xlabel), height, "%+.0f m/s" % xlabel, hcenter=0.5, vcenter=1.0)

        end_index=min(len(self.tm_speed.trace),len(self.tm_torque.trace))
        start_index=max(0, end_index-int(self.decay_time / 0.020))
        simtime=self.m.simtime
        lastcolor_tmi=-999
        pts=[]
        for tmi in range(start_index, end_index):
            t,speed=self.tm_speed.trace[tmi]
            t,torque=self.tm_torque.trace[tmi]
            age=(simtime-t)/self.decay_time
            if age>1.0: continue

            pts.append((ptx(speed), pty(torque)))

            if len(pts)>5 or tmi==end_index-1:
                r=int(65535 * (0.0+age*1.0))
                g=int(65535 * (0.5+age*0.5))
                b=int(65535 * (0.0+age*1.0))
                self.pts_gc.foreground=self.get_colormap().alloc_color(r,g,b)
                pixmap.draw_lines(self.pts_gc, pts)
                pts=[pts[-1]]

# Draw a scrolling graph
class stripchartview(BufferedDrawingArea):
    lmarg=40
    def __init__(self, m, lb, ub, tics):
        BufferedDrawingArea.__init__(self)
        self.m=m
        self.m.add_dependent(self.model_changed)
        self.lb=lb
        self.ub=ub
        self.tics=tics
        self.trace=[]
        self.set_size_request(250,80)
        for value,label in self.tics:
            w,h = size_lstring(self, label)
            stripchartview.lmarg=max(stripchartview.lmarg, 5+w)

    def dup(self):
        n=self.__class__(self.m,self.lb,self.ub,self.tics)
        return n

    def create_gcs(self):
        BufferedDrawingArea.create_gcs(self)
        self.line_gc=self.mkgc('ff0000')
        self.axis_gc=self.mkgc('cccccc')

    def drawit(self, pixmap, style, width, height):
        m=self.m
        tr=m.trace
        xscale=(width-self.lmarg) / 5.0
        tmarg=bmarg=8

        def ptx(x):
            return self.lmarg + int((x - begin_time) * xscale)
        def pty(y):
            return height - bmarg - int((y-self.lb) / (self.ub-self.lb)*(height-tmarg-bmarg))
        
        begin_time=m.simtime - (width-self.lmarg)/xscale
        while tr and tr[0][0]<begin_time:
            tr.pop(0)

        yax=height - (0-self.lb) / (self.ub-self.lb)*height
        pixmap.draw_line(self.axis_gc, self.lmarg, pty(0), width, pty(0))
        pixmap.draw_line(self.axis_gc, ptx(begin_time), tmarg, ptx(begin_time), height-bmarg)
        pixmap.draw_line(self.axis_gc, ptx(begin_time), tmarg, ptx(begin_time)-3, tmarg)
        pixmap.draw_line(self.axis_gc, ptx(begin_time), height-bmarg, ptx(begin_time)-3, height-bmarg)

        for value,label in self.tics:
            draw_lstring(pixmap, self, self.text_gc,
                         self.lmarg, pty(value), label, vcenter=0.5, hcenter=1.0)

        ti=0
        lastx=lasty=None
        while ti < len(tr):

            x=ptx(tr[ti][0])
            y=pty(tr[ti][1])
            if lasty:
                pixmap.draw_line(self.line_gc, lastx, lasty, x, y)
            lastx=x
            lasty=y
            ti+=1

# Load an image file into a GL texture map.
class texbm:
    def __init__(self, name, sx=None, sy=None, swrap=GL.GL_CLAMP, twrap=GL.GL_CLAMP):
        self.name=name
        image = PIL.Image.open(name)
        if sx or sy:
            if not sx: sx=image.width
            if not sy: sy=image.width
            image = image.resize((sx,sy), PIL.Image.BILINEAR)
        
        ix = image.size[0]
        iy = image.size[1]
        imdata = image.tostring("raw", "RGBA", 0, -1)

        self.bbox=array(image.getbbox(),'d') / array([ix,iy,ix,iy],'d')
        print name,self.bbox
        
        self.texid = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.texid)   # 2d texture (x and y size)
        
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, 4, ix, iy, 0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, imdata)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, swrap)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, twrap)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_REPLACE)
        GL.glEnable(GL.GL_TEXTURE_2D)

    #  Draw it with some scale and rotation, defined by mapping tex1 [0-1] to pos1 [gl screen coords]
    def gl_draw(self, zpos, tex1, pos1, tex2, pos2):

        tex_angle=atan2(tex2[1]-tex1[1], tex2[0]-tex1[0])
        tex_len=hypot(tex2[1]-tex1[1], tex2[0]-tex1[0])

        pos_angle=atan2(pos2[1]-pos1[1], pos2[0]-pos1[0])
        pos_len=hypot(pos2[1]-pos1[1], pos2[0]-pos1[0])

        scale=pos_len/tex_len
        rot=pos_angle-tex_angle

        rotm=array([[cos(rot),-sin(rot)], [sin(rot), cos(rot)]],'d')
        rotscalem = rotm * scale

        GL.glEnable(GL.GL_TEXTURE_2D)
        if gl_use_blend:
            GL.glEnable(GL.GL_BLEND);
            GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        else:
            GL.glEnable(GL.GL_ALPHA_TEST);
            GL.glAlphaFunc(GL.GL_GREATER, 0.05)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.texid)

        GL.glBegin(GL.GL_QUADS);
        for texx,texy in ((self.bbox[0], 1.0-self.bbox[1]),
                          (self.bbox[0], 1.0-self.bbox[3]),
                          (self.bbox[2], 1.0-self.bbox[3]),
                          (self.bbox[2], 1.0-self.bbox[1])) :
            GL.glTexCoord2f(texx, texy);
            relpos = numpy.dot(rotscalem, (array([texx,texy]) - tex1)) + pos1
            GL.glVertex3f(relpos[0], relpos[1], zpos)
        GL.glEnd()
        GL.glDisable(GL.GL_TEXTURE_2D)
        if gl_use_blend:
            GL.glDisable(GL.GL_BLEND)
        else:
            GL.glDisable(GL.GL_ALPHA_TEST)
          

class scooterviewcartoon(gtkgl.widget.DrawingArea):
    def __init__(self, m):
        conf=gdkgl.Config((gdkgl.RGBA,
                           gdkgl.DOUBLEBUFFER,
                           gdkgl.DEPTH_SIZE, 1,
                          ))
        gtkgl.widget.DrawingArea.__init__(self, glconfig=conf)
        self.m=m
        self.m.add_dependent(self.model_changed)
        self.set_size_request(500,550)
        self.basex=0.0
        self.connect('expose_event',self.expose_cb)

    def model_changed(self,what=None):
        self.queue_draw()

    def expose_cb(self,w,ev):
        style=self.get_style()
        gc=style.bg_gc[gtk.STATE_NORMAL]

        left,top,width,height=self.get_allocation()

        gldrawable = self.get_gl_drawable()
        glcontext = self.get_gl_context()

        gldrawable.gl_begin(glcontext)
        
        GL.glClearColor(1, 1, 1, 0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glViewport(0,0,width,height)

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(23, width/height, 5, 20)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        
        self.drawit()

        if gldrawable.is_double_buffered():
            gldrawable.swap_buffers()
        else:
            glFlush()

        gldrawable.gl_end()

    def drawit(self):
        m=self.m
        if not hasattr(self,'rider_texbm'):
            def texc(x,y):
                return array([x, 511-y],'d') / 512.0
            self.rider_texbm = texbm("scooter-cartoon-rider.png",256,256)
            self.frame_texbm = texbm("scooter-cartoon-frame.png",256,256)
            self.wheel_texbm = texbm("scooter-cartoon-wheel.png",256,256)
            self.upperarm_texbm = texbm("scooter-cartoon-upperarm.png",256,256)
            self.lowerarm_texbm = texbm("scooter-cartoon-lowerarm.png",256,256)
            self.ouch_texbm = texbm("scooter-cartoon-ouch.png",256,256)
            self.squiggles_texbm = texbm("squiggles.png", swrap=GL.GL_REPEAT)
            self.texpos_rim=texc(123,433)
            self.texpos_base=texc(82,435)
            self.texpos_shoulder=texc(85,144)
            self.texpos_elbow=texc(124,215)
            self.texpos_handle=texc(167,200)

        # pan to follow scooter
        self.basex=max(self.basex, m.base_xpos-0.5)
        self.basex=min(self.basex, m.base_xpos+0.5)
        self.basex=max(self.basex, m.rider_xpos-0.5)
        self.basex=min(self.basex, m.rider_xpos+0.5)

        GLU.gluLookAt(self.basex, 1.1, 10,
                      self.basex, 1.1, 0,
                      0, 1, 0)

        tplot=[]
        ti=bsearch_le(m.terrain[:,0], self.basex-5)
        GL.glColor(0,0,0,0)
        GL.glLineWidth(5)
        GL.glBegin(GL.GL_LINE_STRIP)
        while ti+1 < len(m.terrain):
            x1=m.terrain[ti,0]
            y1=m.terrain[ti,1]*m.terrain_ampl
            GL.glVertex3f(x1, y1, 0.0)
            if x1>self.basex+10: break
            ti+=1
        GL.glEnd()


        if 0: 
            for sb in range((self.basex-2)//1, (self.basex+3)//1):
                self.squiggles_texbm.gl_draw(0.0,
                                             array([0,0]), array([sb, -1]),
                                             array([1,0]), array([sb+1, -1]))
        # wind arrow
        if m.wind_speed!=0.0:
            xmul=m.wind_speed/10
            if m.wind_speed<0.0:
                xbase=m.rider_xpos+0.5
            else:
                xbase=m.rider_xpos-0.5
            GL.glBegin(GL.GL_LINE_STRIP)
            GL.glColor(0.5,0.5,0.5,0)
            for ptx,pty in ((0.00, -0.03), (0.05, -0.03),
                            (0.05, -0.05), (0.10, 0.00), (0.05, 0.05),
                            (0.05, 0.03), (0.00, 0.03),
                            (0.00, -0.03)):
                GL.glVertex3f(xbase+xmul*(ptx-0.10)*4, m.rider_ypos*0.8+pty*4, 0.0)
            GL.glEnd()
            if 0:
                str=speed_str(m.wind_speed)
                textx=xbase
                draw_lstring(pixmap, self, self.text_gc, textx, height*0.4, str, hcenter=(xmul<0 and 1.0 or 0.0))

        self.rider_texbm.gl_draw(0.0,
                                 self.texpos_base, array([m.base_xpos, m.base_ypos]),
                                 self.texpos_shoulder, array([m.rider_xpos, m.rider_ypos]))
        self.frame_texbm.gl_draw(0.0,
                                 self.texpos_base, array([m.base_xpos, m.base_ypos]),
                                 self.texpos_handle, array([m.handle_xpos, m.handle_ypos]))

        if m.fallen:
            self.ouch_texbm.gl_draw(0.0,
                                    self.texpos_base, array([m.base_xpos, m.base_ypos]),
                                    self.texpos_shoulder, array([m.rider_xpos, m.rider_ypos]))

        wr= m.wheel_rad
        wa= -m.base_xpos / wr
        self.wheel_texbm.gl_draw(0.0,
                                 self.texpos_base, array([m.base_xpos, m.base_ypos]),
                                 self.texpos_rim, array([m.base_xpos + wr*cos(wa), m.base_ypos + wr*sin(wa)]))

        # arms
        shoulder = array([m.rider_xpos, m.rider_ypos])
        hand = array([m.handle_xpos, m.handle_ypos])
        handrelx=hand[0]-shoulder[0]
        handrely=hand[1]-shoulder[1]
        
        offlinesq=0.5*m.arm_len**2 - handrelx**2 - handrely**2
        if offlinesq>0:
            offline = sqrt(offlinesq)/2
        else:
            offline=0.0

        elbow=array([shoulder[0] + handrelx*0.6 + handrely / hypot(handrely, handrelx)*offline,
                     shoulder[1] + handrely*0.6 - handrelx / hypot(handrely, handrelx)*offline])
        
        self.upperarm_texbm.gl_draw(0.0,
                                    self.texpos_shoulder, shoulder,
                                    self.texpos_elbow, elbow)
        self.lowerarm_texbm.gl_draw(0.0,
                                    self.texpos_elbow, elbow,
                                    self.texpos_handle, hand)


class scooterview(BufferedDrawingArea):
    def __init__(self, m):
        BufferedDrawingArea.__init__(self)
        self.m=m
        self.m.add_dependent(self.model_changed)
        self.size(500,550)
        self.basex=-1.0
        self.pig_time=0.0

    def create_gcs(self):
        self.wheel_tire_gc=self.mkgc('666666')
        self.wheel_outer_gc=self.mkgc('cccccc')
        self.wheel_hub_gc=self.mkgc('999999')
        self.spoke_gc=self.mkgc('444444')
        
        self.rider_gc=self.mkgc('cc8888')
        self.rider_head_gc=self.mkgc('bb7777')
        self.rider_arm_gc=self.mkgc('664444')
        self.ground_gc=self.mkgc('886666')
        self.handle_gc=self.mkgc('000000')
        self.measure_gc=self.mkgc('000000')
        self.measure_tape_gc=self.mkgc('ffff00')
        self.wind_gc=self.mkgc('999999')
        self.clouds=create_pixmap_from_xpm(self.get_window(), None, "clouds.xpm")[0]
        self.clouds_gc=self.mkgc('000000')
        self.clouds_gc=self.mkgc('000000')
        self.clouds_gc.tile=self.clouds
        self.clouds_gc.fill=GDK.TILED
        self.flying_pig=create_pixmap_from_xpm(self.get_window(), None, "flying_pig.xpm")
        self.flying_pig_gc=self.mkgc('000000')
        self.pig_pos=[]
        for iter in range(0,5):
            pigx=random.uniform(10, 200.0)
            self.pig_pos.append([pigx, random.gauss(2.2, 0.4), random.gauss(0.0, 1.0), random.gauss(0.0, 1.0), 0.0])

    def drawit(self, pixmap, style, width, height):
        m=self.m
        scale=min(width/3.0, height/2.6)
        maxx=width/scale

        # pan to follow scooter
        self.basex=max(self.basex, m.base_xpos-maxx*0.6)
        self.basex=min(self.basex, m.base_xpos-maxx*0.2)
        self.basex=max(self.basex, m.rider_xpos-maxx*0.6)
        self.basex=min(self.basex, m.rider_xpos-maxx*0.2)

        # coordinate transforms
        def xsize(x): return x*scale
        def ysize(y): return -y*scale
        def ptx(x): return xsize(x-self.basex)
        def pty(y): return height + ysize(y+0.3)

        # we integerize here to avoid appearance of wiggling between parts of rider
        rider_xcoord=int(ptx(m.rider_xpos))
        rider_ycoord=int(pty(m.rider_ypos))
        base_xcoord=int(ptx(m.base_xpos))
        base_ycoord=int(pty(m.base_ypos))
        handle_xcoord=int(ptx(m.handle_xpos))
        handle_ycoord=int(pty(m.handle_ypos))

        self.clouds_gc.ts_x_origin = int(ptx(0)) % self.clouds.width
        draw_rectangle(pixmap, self.clouds_gc, 1,
                       0, 0, width, height)

        if 1:
            curt=time.time()
            dt=min(curt-self.pig_time, 0.1)
            self.pig_time=curt
            for pig in self.pig_pos:
                pig[2] = pig[2]*(1.0-dt/5) + random.gauss(m.wind_speed, 1.0)*dt/5
                pig[3] = pig[3]*(1.0-dt/5) + random.gauss(-(pig[1]-3)/2, 1.0)*dt/5
                pig[0] += dt*pig[2]
                pig[1] += dt*pig[3]
                x=int(ptx(pig[0]))
                y=int(pty(pig[1]))
                if x>-100 and x<width+100:
                    self.flying_pig_gc.clip_x_origin=x
                    self.flying_pig_gc.clip_y_origin=y
                    self.flying_pig_gc.clip_mask = self.flying_pig[1]
                    draw_pixmap(pixmap, self.flying_pig_gc, self.flying_pig[0],
                                0, 0, x, y,
                                self.flying_pig[0].width, self.flying_pig[0].height)

        # wind arrow
        if m.wind_speed!=0.0:
            if m.wind_speed<0.0:
                xbase=width
                xmul=-width
            else:
                xbase=0
                xmul=width
            draw_polygon(pixmap, self.wind_gc, 1,
                         map(lambda pt: (xbase+xmul*pt[0], height*pt[1]),
                             ((0.00, 0.30), (0.05, 0.30), (0.05, 0.28),
                              (0.10, 0.33), (0.05, 0.38), (0.05, 0.36),
                              (0.00, 0.36), (0.00, 0.30))))
            str=speed_str(m.wind_speed)
            textx=xbase
            draw_lstring(pixmap, self, self.text_gc, textx, height*0.4, str, hcenter=(xmul<0 and 1.0 or 0.0))

        tplot=[]
        til=bsearch_le(m.terrain[:,0], self.basex)
        for tx,ty in m.terrain[til:]:
            tplot.append((ptx(tx),pty(ty*m.terrain_ampl)))
            if tx>self.basex+maxx*1.5: break
        if tplot:
            tplot.append((tplot[-1][0],pty(-0.5)))
            tplot.append((tplot[0][0],pty(-0.5)))
            draw_polygon(pixmap, self.ground_gc, 1, tplot)

        if 0:
            for xg in range(int(self.basex-2), int(self.basex+6)):
                draw_rectangle(pixmap, self.bg_gc, 1, ptx(xg),pty(0.5), xsize(0.05),height)

        # body + legs
        self.rider_gc.line_width=int(xsize(0.15))
        draw_line(pixmap, self.rider_gc, rider_xcoord, rider_ycoord, base_xcoord+xsize(-0.08), base_ycoord)

        # handle
        self.handle_gc.line_width=int(xsize(0.05))
        draw_line(pixmap, self.handle_gc, ptx(m.base_xpos + 0.14), pty(m.base_ypos), ptx(m.handle_xpos), pty(m.handle_ypos))

        # wheel
        draw_arc(pixmap, self.wheel_tire_gc, 1,
                 base_xcoord + xsize(-m.wheel_rad), base_ycoord+ysize(m.wheel_rad),
                 xsize(m.wheel_rad*2), ysize(-m.wheel_rad*2), 
                 0, 360*64)
        
        draw_arc(pixmap, self.wheel_outer_gc, 1,
                 base_xcoord + xsize(-m.wheel_rad*0.8), base_ycoord+ysize(m.wheel_rad*0.8),
                 xsize(m.wheel_rad*1.6), ysize(-m.wheel_rad*1.6), 
                 0, 360*64)
        
        draw_arc(pixmap, self.wheel_hub_gc, 1,
                 base_xcoord + xsize(-m.wheel_rad*0.3), base_ycoord+ysize(m.wheel_rad*0.3),
                 xsize(m.wheel_rad*0.6), ysize(-m.wheel_rad*0.6), 
                 0, 360*64)

        # spokes
        for ai in range(0,3):
            a=ai*2*pi/3
            self.spoke_gc.line_width=int(xsize(0.03))
            wr=m.wheel_rad
            sina=sin(a - m.base_xpos / wr)
            cosa=cos(a - m.base_xpos / wr)
            draw_line(pixmap, self.spoke_gc,
                      base_xcoord+xsize(wr*0.3*cosa), base_ycoord+ysize(wr*0.3*sina),
                      base_xcoord+xsize(wr*0.81*cosa), base_ycoord+ysize(wr*0.81*sina))

        # head
        draw_arc(pixmap, self.rider_head_gc, 1,
                 rider_xcoord+xsize(-0.1), rider_ycoord+ysize(0.1),
                 xsize(0.25), ysize(-0.25),
                 0, 360*64)

        # arms
        shoulder = (m.rider_xpos*0.85 + m.base_xpos*0.15,
                    m.rider_ypos*0.85 + m.base_ypos*0.15)
        hand = (m.handle_xpos, m.handle_ypos)

        handrelx=hand[0]-shoulder[0]
        handrely=hand[1]-shoulder[1]

        offlinesq=0.5*m.arm_len**2 - handrelx**2 - handrely**2
        if offlinesq>0:
            offline = sqrt(offlinesq)/2
        else:
            offline=0.0

        elbow=(shoulder[0] + handrelx/2 + handrely / hypot(handrely, handrelx)*offline,
               shoulder[1] + handrely/2 - handrelx / hypot(handrely, handrelx)*offline)

        self.rider_arm_gc.line_width=int(xsize(0.08))
        self.rider_arm_gc.join_style=GDK.JOIN_ROUND
        draw_lines(pixmap, self.rider_arm_gc,
                     ((ptx(shoulder[0]),pty(shoulder[1])),
                      (ptx(elbow[0]),pty(elbow[1])),
                      (ptx(hand[0]),pty(hand[1]))))

        # Ouch
        if m.fallen:
            draw_lstring(pixmap, self, self.text_gc, ptx(m.rider_xpos-0.1), pty(m.rider_ypos+0.25), "OUCH!")

        # Stoppping distance measure
        if m.measuremode:
            self.measure_gc.line_width=int(xsize(0.02))
            draw_line(pixmap, self.measure_gc, ptx(m.measure_from_xpos), pty(-0.15), ptx(m.measure_to_xpos), pty(-0.15))
            draw_rectangle(pixmap, self.measure_tape_gc, 1, ptx(m.measure_to_xpos), pty(-0.05), xsize(0.10), ysize(-0.10))

    
class scooterstats(BufferedDrawingArea):
    def __init__(self, m):
        BufferedDrawingArea.__init__(self)
        self.m=m
        m.add_dependent(self.model_changed)
        self.set_size_request(200,90)

    def drawit(self, pixmap, style, width, height):
        m=self.m

        content=StringIO()
        print>>content, "speed="+speed_str(m.base_dxpos)
        print>>content, "power=%0.3f kW" % (m.motor_power/1000)

        if m.measuremode:
            print>>content, "stop in " + distance_str(m.measure_to_xpos - m.measure_from_xpos)
            if m.measuremode==2:
                stopdist=abs(m.measure_from_xpos - m.measure_to_xpos)
                origspeed=abs(m.measure_from_dxpos)
                # v**2 = 2*a*d
                # a = v**2 / d / 2
                accel=origspeed**2 / stopdist / 2.0
                print>>content, "   avg decel = %0.1f m/S/S (%0.2f g)" % (accel, accel/9.8)

        draw_lstring(pixmap, self, self.text_gc, 5, 15, content.getvalue())
                      

# Quick popup window around the GtkWidget content. Type 'q' to close
class popup:
    def __init__(self, content):
        self.content=content
        self.win=gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.win.connect('destroy',self.destroy_cb)
        self.win.connect('key_press_event',self.toplevel_key_press_cb)
        self.win.add(content)
        self.win.show_all()

    def destroy_cb(self,*args):
        self.win.hide()
    def toplevel_key_press_cb(self,w,ev):
        key=gdk.keyval_name(ev.keyval)
        if key=='q':
            self.win.destroy()
            self.content.destroy()

class scooterapp:
    def __init__(self, style=scooterviewcartoon):
        self.s=scooter()
        self.sv=style(self.s)
        self.runtimes=[]
        self.start_time=time.time()
        self.win=gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.win.connect('destroy',self.destroy_cb)
        self.win.connect('key_press_event',self.toplevel_key_press_cb)
        self.win.set_default_size(1024,768)

        self.main_hb=gtk.HBox()
        self.win.add(self.main_hb)

        self.main_hb.pack_start(self.sv, 1, 1, 0)

        self.rvb=gtk.VBox()
        self.main_hb.pack_start(self.rvb, 0, 0, 0)

        self.adjusters={}
        self.stripcharts={}
        self.stripcharts_inorder=[]
        
        nb=gtk.Notebook()
        nb_run=gtk.VBox()
        nb.append_page(nb_run, gtk.Label("Run"))
        nb_conf_machine=gtk.VBox()
        nb.append_page(nb_conf_machine, gtk.Label("Machine"))
        nb_conf_rider=gtk.VBox()
        nb.append_page(nb_conf_rider, gtk.Label("Rider"))
        self.rvb.add(nb)

        self.add_adj(self.s, 'rider_goal_speed', -8.0, 8.0, page_increment=1.0, digits=1, where=nb_run)
        self.add_adj(self.s, 'speed_lim', 1.0, 11.0, page_increment=2.0, digits=1, where=nb_run)
        self.add_adj(self.s, 'terrain_ampl', 0.0, 0.5, page_increment=0.1, digits=2, where=nb_run)
        self.add_adj(self.s, 'wind_speed', -20, 20, page_increment=5, digits=0, where=nb_run)
        
        self.add_adj(self.s, 'max_motor_power', 0.0, 10000.0, page_increment=1000.0, digits=1, where=nb_conf_machine)
        self.add_adj(self.s, 'max_motor_regen', 0.0, 10000.0, page_increment=1000.0, digits=1, where=nb_conf_machine)
        self.add_adj(self.s, 'max_motor_torque', 0.0, 300.0, page_increment=50.0, digits=1, where=nb_conf_machine)
        self.add_adj(self.s, 'handle_len', 0.5, 1.5, page_increment=0.5, digits=1, where=nb_conf_machine)
        self.add_adj(self.s, 'wheel_rad', 0.05, 0.5, page_increment=0.1, digits=2, where=nb_conf_machine)
        self.add_adj(self.s, 'base_mass', 5, 100, page_increment=5.0, digits=0, where=nb_conf_machine)
        self.add_adj(self.s, 'base_moi', 0.1, 10.0, page_increment=1.0, digits=1, where=nb_conf_machine)
        self.add_adj(self.s, 'kp', 0, 1000, page_increment=100, digits=0, where=nb_conf_machine)
        self.add_adj(self.s, 'kd', 0, 1000, page_increment=100, digits=0, where=nb_conf_machine)
        self.add_adj(self.s, 'bus_speed_lim', 1, 10, page_increment=1, digits=0, where=nb_conf_machine)
        self.add_adj(self.s, 'bus_torque_constant', 1, 300, page_increment=1, digits=0, where=nb_conf_machine)
        
        self.add_adj(self.s, 'rider_mass', 20, 300, page_increment=10.0, digits=0, where=nb_conf_rider)
        self.add_adj(self.s, 'rider_height', 1.0, 2.5, page_increment=0.5, digits=1, where=nb_conf_rider)
        self.add_adj(self.s, 'rider_springrate', 0, 2000, page_increment=100.0, digits=0, where=nb_conf_rider)
        self.add_adj(self.s, 'rider_damping', 0, 10000, page_increment=10, digits=0, where=nb_conf_rider)
        self.add_adj(self.s, 'rider_eff_area', 0.0, 2.5, page_increment=0.5, digits=1, where=nb_conf_rider)


        butbox=gtk.HBox()
        nb_run.pack_start(butbox,0,1,5)
        but=gtk.Button("Panic Stop")
        but.connect('clicked',self.set_to_zero, self.adjusters['rider_goal_speed'])
        butbox.pack_start(but,0,0,5)
        
        but=gtk.Button("Restart")
        but.connect('clicked', self.restart_cb)
        butbox.pack_start(but,0,0,5)
        
        nb_run.pack_start(self.add_scv(self.s.trace_motor_torque,'Motor Torque',  -300, 300, ((-300, "-300 Nm"), (300,"+300 Nm"))),
                          0,0,2)
        nb_run.pack_start(self.add_scv(self.s.trace_motor_power, 'Motor Power', -5000, 5000, ((-5000, "-5 kw"), (5000,"+5 kw"))),
                          0,0,2)
        nb_run.pack_start(self.add_scv(self.s.trace_tilt_angle, 'Tilt Angle', -pi/12, pi/12, ((-pi/12, "-15"), (pi/12,"15"))),
                          0,0,2)
        nb_run.pack_start(add_gtk_frame("Motor Envelope",motor_envelope_view(self.s)), 0,0,2)

        nb_run.pack_start(scooterstats(self.s), 0,0,2)
        nb_conf_machine.pack_start(add_gtk_frame("Motor Envelope",motor_envelope_view(self.s)), 0,0,2)

        self.win.show_all()

    def add_adj(self, model, key, lb, ub, page_increment=1.0, digits=2, name=None, where=None):
        adj=model.adjuster(key, lb, ub, page_increment=page_increment)
        self.adjusters[key]=adj
        scale=gtk.HScale(adj)
        scale.set_digits(digits)
        scale.set_size_request(150,40)
        if not name:
            name=re.sub(r'_',' ',key)
        where.pack_start(add_gtk_frame(name,scale),0,1,0)

    def add_but(self, name, action, parms, where=None):
        but=gtk.Button(name)
        but.connect('clicked',action,*parms)
        where.pack_start(but,0,1,0)

    def add_sep(self):
        sep=gtk.HSeparator()
        self.rvb.pack_start(sep,0,1,10)

    def add_scv(self, model, name, lb, ub, tics):
        sc=stripchartview(model, lb, ub, tics)
        self.stripcharts_inorder.append((name,sc))
        if not name:
            name=re.sub(r'_',' ',key)
        return add_gtk_frame(name,sc)

    def set_to_zero(self, but, adj):
        adj.set_value(0.0)
        self.s.start_measure()

    def restart_cb(self, but):
        self.s.restart()
        self.start_time=time.time()

    def destroy_cb(self,*args):
        gtk.main_quit()
    def toplevel_key_press_cb(self,w,ev):
        key=gdk.keyval_name(ev.keyval)
        if key=='q':
            gtk.mainquit()
        elif key=='space':
            self.s.paused=not self.s.paused
        elif key=='t':
            self.adjusters['terrain_ampl'].set_value(0.1)
        elif key=='T':
            self.adjusters['terrain_ampl'].set_value(0.2)
        elif key=='w':
            self.adjusters['wind_speed'].set_value(-10.0)
        elif key=='f':
            self.adjusters['rider_goal_speed'].set_value(1.0)
        elif key=='F':
            self.adjusters['rider_goal_speed'].set_value(5.0)
        elif key=='b':
            self.adjusters['rider_goal_speed'].set_value(-1.0)
        elif key=='B':
            self.adjusters['rider_goal_speed'].set_value(-4.0)
        elif key=='p':
            self.adjusters['rider_goal_speed'].set_value(0.0)
            self.s.start_measure()
        elif key=='g':
            self.open_big_graphs()
        else:
            print "Unknown key %s" % key

    def open_big_graphs(self):
        vb=gtk.VBox()
        for name,sc in self.stripcharts_inorder:
            n=sc.dup()
            n.set_size_request(600,200)
            vb.pack_start(add_gtk_frame(name,n), 1, 1, 5)
        popup(vb)
            
    def periodic(self):
        ts=time.time()
        dt=0.005
        if self.s.paused:
            self.start_time = ts-self.s.simtime
        while not self.s.fallen and not self.s.paused and self.s.simtime < (ts-self.start_time):
            self.s.run(dt)
        return 1


if __name__=='__main__':
    style=scooterviewcartoon
    for arg in sys.argv[1:]:
        if arg=='-s': style=scooterview
        if arg=='-nob': gl_use_blend=0
        if arg=='-b': gl_use_blend=1
        if arg=='-g':
            s=scooter()
            fwdl=[]
            revl=[]
            for speed in arange(-9.0, 9.0, 0.1):
                maxt,mint=s.motor_torque_limits(speed)
                if mint>maxt: continue
                fwdl.append((speed,maxt))
                revl.append((speed,mint))
            revl.reverse()
            grfp=open('motor_torque_limits.graph','w')
            for speed,torque in fwdl+revl+fwdl[0:1]:
                print>>grfp, speed,torque
            grfp.close()
            sys.exit(0)

    app=scooterapp(style)
    # aim for 30 fps. You can change this to suit your hardware.
    # I find that on a 1.2 GHz P4, a 9 mS interval gives 100 fps and takes 50% cpu power
    gobject.timeout_add(30, app.periodic) 
    gtk.main()
