#!/usr/bin/python
#
# Copyright 2011, Mark Kalmes
# portions Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Mark Kalmes nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY MARK KALMES ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

"""
lib_ssc32 provides the basics of interacting over a serial connection with the
SSC-32 controller. It does not depend on ROS, and can be used outside a ROS
environment.

SSC32_Controller represents the controller itself, contains a mutex object, 
and has a few utility methods. SSC32_Servo provides per-servo
commands and is initialized with a controller object.

Initializing a servo is made easier by passing a dictionary of initialization
values. ros_ssc32 uses this method and passes in a ROS parameter.
"""

import serial
import time
import thread
import sys, optparse
import math

#debug = open('/home/mark/debug.txt', 'w')

class SSC32_Controller():
    """ Manages overall communication with the SSC32
    """
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 115200 ):
        try:
            self.dev_name = string.atoi( dev_name ) # stores the serial port as 0-based integer for Windows
        except:
            self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self.mutex = thread.allocate_lock()
        self.servo_dev = None
        self.move_queue = None

        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def add_move_queue(self, msg):
        """Add string to the movement queue. Will be added to next movement send_serial() commmand"""

        if self.move_queue:
            self.move_queue += msg
        else:
            self.move_queue = msg

    def send_serial(self, msg, move_command=False):
        """send the serial msg. if move_command is set, adds any existing move_queue to the message"""

        # It is up to the caller to acquire / release mutex
        if move_command and self.move_queue:
            #print >> debug, 'Q:', self.move_queue; debug.flush()
            self.servo_dev.write(self.move_queue)
            self.move_queue = None
        self.servo_dev.write( msg )
        #print >> debug, 'W:', msg; debug.flush()

    def read_serial(self, nBytes=1):
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):
        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors 
            # on WinXP (Conversations with Brian Wu @ MIT on 6/23/2010)
            self.servo_dev.close()  
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError('lib_ssc32: Serial port not found!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_ssc32: Serial port not found!\n')

    def get_version(self):
        ''' returns output from VER command on controller '''
        
        self.send_serial("VER\r")
        return self.servo_dev.readline()


    ## functions below are general purpose functions that apply to the controller
    
    def is_moving(self):
        ''' returns True if a servo is moving.
        '''
        self.acq_mutex()
        self.send_serial("Q\r")
        data = self.read_serial(1)
        self.rel_mutex()
        return data == '+'

    def read_digital(self, A=False, B=False, C=False, D=False, latched=False):
        ''' Reads digital inputs (A,B,C,D) if set True
            returns string with "0" for low and "1" for high, ex: "0010"
            If latched is set, output shows if low since last read
        '''
        self.acq_mutex()
        query = ""
        num = 0
        if A:
            query += "AL " if latched else "A "
            num += 1
        if B:
            query += "BL " if latched else "B "
            num += 1
        if C:
            query += "CL " if latched else "C "
            num += 1
        if D:
            query += "DL " if latched else "D "
            num += 1
        if num == 0:
            print "Warning: read_digital called with no params set. Ignoring"
            return ""
        query = query[:-1] + "\r"
        self.send_serial(query)
        result = self.read_serial(num)
        self.rel_mutex()
        return result
    
    def read_analog(self, A=False, B=False, C=False, D=False):
        ''' Reads inputs as analog (A,B,C,D) if set True
            returns list of integers 0..255 giving level of input
            NOTE - SSC32 does NOT give accurate analog read until AFTER 
            the first time inputs are read as analog 
        '''
        self.acq_mutex()
        query = ""
        num = 0
        if A:
            query += "VA "
            num += 1
        if B:
            query += "VB "
            num += 1
        if C:
            query += "VC "
            num += 1
        if D:
            query += "VD "
            num += 1
        if num == 0:
            print "Warning: read_analog called with no params set. Ignoring"
            return []
        query = query[:-1] + "\r"
        self.send_serial(query)
        result = self.read_serial(num)
        self.rel_mutex()
        return [ord(ch) for ch in result]


class SSC32_Servo():
    ''' Represents a servo attached to the SSC32
    '''
    def __init__(self, SSC32, servo_id=None, servo_param=None):
        ''' SSC32 - SSC32_Controller object this servo is attached to
            servo_id - (optional) port servo is attached to (0..31), default 0
            servo_param - (optional) dictionary containing settings
                valid settings are: 
                    id - 0-31 header for servo - replaces servo_id parameter
                    zero_us - microsecond settings for the zero angle point
                              all angles are calculated from the zero_us, so may
                              need to adjust min/max_ang if you change this
                    min_us, max_us - limit to microsecond settings 
                    min_ang, max_ang - limit to radian angles 
                    max_speed - in radians per second
                    
                    init_ang - (optional) init servo to this radian angle
                            if set, move_home() uses this position as home
                            if not set, servo will not init on startup
                    pos_offset - (optional) -100 to 100 microseconds
                            this offset is used by SSC-32 to calibrate the
                            servo and is added to all movement commands
                TODO - support flipping the angles?
        '''

        # these defaults are some basic sane values for HiTech servos
        defaults = {
            'zero_us': 1500, # all angles are calculated from here
            'max_speed': math.radians(90),

            # limits are redundant between _us and _ang, both apply
            'max_us': 2400,
            'min_us': 600,
            'max_ang': math.radians(90),
            'min_ang': math.radians(-90),
            
            'rad_per_us': (math.radians(90) / 900),
                # note that we do _not_ choose to derive this from a combination 
                # of us and angle limits, as I've found it more convenient to 
                # tweak limits without making sure they are matched, and adjust 
                # this value directly if I'm actually dealing with a 
                # non-standard servo
            }
                
        assert(SSC32)
        self.ssc32 = SSC32
        self.servo_id = servo_id if servo_id is not None else 0
        
        # Set various parameters.  Use servo_param dictionary, or set defaults
        self.settings = {}
        if servo_param:
            self.settings = servo_param
            if 'id' in servo_param:
                self.servo_id = servo_param['id']
        for key in defaults.keys():
            self.settings.setdefault(key, defaults[key])   
        #print >> sys.stderr, str(self.settings); sys.stderr.flush()
            
        # calibrate and start at home position
        if 'pos_offset' in self.settings:
            self.pos_offset(self.settings['pos_offset'])
        if 'init_ang' in self.settings:
            self.move_home()
            
            

    def move_home(self):
        ''' reset to the home position, limited to max_speed
        '''
        
        speed = int(round(self.settings['max_speed'] / self.settings['rad_per_us']))
        pos = self.settings['zero_us']
        if 'init_ang' in self.settings:
            pos += int(round(self.settings['init_ang'] / self.settings['rad_per_us']))
        
        self.ssc32.acq_mutex()
        self.ssc32.send_serial("#%i P%i S%i\r" % (self.servo_id, pos, speed))
        self.ssc32.rel_mutex()

    def move_angle(self, ang, angvel=None, endtime=None, endgroup=True):
        ''' move to angle (radians)
            
            optional params:
                angvel - set the rad/s desired speed for servo to transition
                endtime - set the desired time in seconds for transition
                endgroup - if False, queue this movement with other movements. 
                    The last movement should have endgroup=True, and all
                    servos will move simultaneously
        '''
                
        # limit angle
        ang = min(ang, self.settings['max_ang'])
        ang = max(ang, self.settings['min_ang'])
        
        # if velocity is set, recalculate in terms of us/s
        speed = None
        if angvel:
            angvel = min(angvel, self.settings['max_speed']) # limit first
            speed = angvel / self.settings['rad_per_us']
            speed = int(round(speed))
        
        # recalc endtime in ms
        if endtime:
            endtime *= 1000
            endtime = int(round(endtime))
            
        # calculate final position, and limit us
        pos = ang / self.settings['rad_per_us']
        pos += self.settings['zero_us']
        pos = int(round(pos))
        pos = min(pos, self.settings['max_us'])
        pos = max(pos, self.settings['min_us'])
        
        # construct our movement string
        movestr = "#" + str(self.servo_id)
        movestr += " P" + str(pos)
        if speed:
            movestr += " S" + str(speed)
        if endtime:
            movestr += " T" + str(endtime)
            
        self.ssc32.acq_mutex()
        if endgroup:
            self.ssc32.send_serial(movestr + '\r', move_command=True)
        else:
            self.ssc32.add_move_queue(movestr + ' ')
        self.ssc32.rel_mutex()
        
    def read_angle(self):
        ''' Return the current angle (in radians) for this servo. 
            NOTE - this is not actually returning an angle from the servo, 
                only the current input position by the controller
        '''
        query = "QP%i\r" % (self.servo_id)
        self.ssc32.acq_mutex()
        self.ssc32.send_serial(query)
        result = self.ssc32.read_serial(1)
        self.ssc32.rel_mutex()
        pos_us = ord(result) * 10 # one byte returned with position/10
        return (pos_us - self.settings['zero_us']) * self.settings['rad_per_us']

    def pos_offset(self, us):
        ''' set the logical position offset for this servo, this offset is added
            by the controller to subsequent move commands and should be used to 
            adjust center to 1500us. Valid range is -100..100
        '''
        # limit us
        us = min(us, 100)
        us = max(us, -100)
        us = int(round(us))
        
        movestr = "#%i PO %i\r" % (self.servo_id, us)
        self.ssc32.acq_mutex()
        self.ssc32.send_serial(movestr)
        self.ssc32.rel_mutex()
        
    def set_discrete(self, high=True, endgroup=True):
        ''' set output either high or low
            (presumably we are not actually connected to a servo)
        '''
        
        movestr = "#" + str(self.servo_id)
        movestr += "H" if high else "L"
        if endgroup:
            movestr += "\r"
        else:
            movestr += " "
        
        self.ssc32.acq_mutex()
        self.ssc32.send_serial(movestr)
        self.ssc32.rel_mutex()
        

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='dev_name',
                 help='Required: Device string for SSC-32. [eg. /dev/ttyUSB0 for Linux, \'0\' (for COM1) on Windows]')
    p.add_option('--id', action='store', type='int', dest='id',
                 help='id of servo to connect to, [default = 0]', default=0)
    p.add_option('--ang', action='store', type='float', dest='ang',
                 help='Angle to move the servo to (degrees).')
    p.add_option('--ang_vel', action='store', type='float', dest='ang_vel',
                 help='angular velocity. (degrees/sec) [default = 50]', default=50)
    p.add_option('--baud', action='store', type='int', dest='baud',
                 help='baudrate for SSC-32 connection [default = 115200]', default=115200)

    opt, args = p.parse_args()

    if opt.dev_name == None:
        p.print_help()
        sys.exit(0)

    control = SSC32_Controller(opt.dev_name, opt.baud)

    if opt.ang != None:
        servo = SSC32_Servo( control, opt.id )
        servo.move_angle( math.radians(opt.ang), math.radians(opt.ang_vel) )
    
