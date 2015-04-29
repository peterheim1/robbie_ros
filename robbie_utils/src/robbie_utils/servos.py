#!/usr/bin/env python

"""
  servos.py: classes for servo interaction
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from math import pi, radians
import xml.dom.minidom

from std_msgs.msg import Float64
from arbotix_msgs.srv import *
from diagnostic_msgs.msg import *

from arbotix_python.ax12 import *
       

class Servo:
    """ Class to handle services and updates for a single Servo, 
        on an ArbotiX robocontroller's AX/RX-bus. """

    def __init__(self, device, name):
        self.device = device
        self.name = name
        n = "~dynamixels/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.neutral = rospy.get_param(n+"neutral",512)
        self.ticks = rospy.get_param(n+"ticks",1024)
        self.rad_per_tick = radians(rospy.get_param(n+"range",300.0))/self.ticks

        self.max_angle = radians(rospy.get_param(n+"max_angle",150))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-150))
        self.max_speed = radians(rospy.get_param(n+"max_speed",684.0)) 
                                                # max speed = 114 rpm - 684 deg/s
        self.invert = rospy.get_param(n+"invert",False)
        self.readable = rospy.get_param(n+"readable",True)

        self.controller = None
        self.status = "OK"
        self.level = DiagnosticStatus.OK

        self.dirty = False                      # newly updated position?
        self.angle = 0.0                        # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.relaxed = True                     # are we not under torque control?
        self.last = rospy.Time.now()

        self.reads = 0.0                        # number of reads
        self.errors = 0                         # number of failed reads
        self.total_reads = 0.0                  
        self.total_errors = [0.0]

        self.voltage = 0.0
        self.temperature = 0.0
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        rospy.Service(name+'/relax', Relax, self.relaxCb)

    def angleToTicks(self, angle):
        """ Convert an angle to ticks, applying limits. """
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    def ticksToAngle(self, ticks):
        """ Convert an ticks to angle, applying limits. """
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
        #if angle > self.max_angle:
        #    return self.max_angle
        #if angle < self.min_angle:
        #    return self.min_angle
        return angle        

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        if not self.device.fake:
            self.device.disableTorque(self.id)
        self.dirty = False
        self.relaxed = True
        return RelaxResponse()

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.desired != req.data or self.relaxed:
            self.dirty = True   
            self.relaxed = False
            self.desired = req.data

    def update(self, reading=None):
        """ Update angle in radians by reading from servo, or by 
            using position passed in from a sync read (in ticks). """
        if reading == None:                     # read from device (no sync_read)
            if self.readable and not self.device.fake:
                reading = self.device.getPosition(self.id)
        if reading > -1 and reading < self.ticks:     # check validity
            self.reads += 1
            self.total_reads += 1
            last_angle = self.angle
            self.angle = self.ticksToAngle(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.angle - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logdebug("Invalid read of servo: id " + str(self.id) + ", value " + str(reading))
            self.errors += 1
            self.total_reads += 1
            return
        if self.relaxed:
            self.last_cmd = self.angle

    def updateTemp(self, reading=None):
        """ Update temperature by reading from servo, or by passing value from a sync read. """
        if reading == None:                     # read from device (no sync_read)
            if self.readable and not self.device.fake:
                self.temperature = self.device.getTemperature(self.id)
        else:                                   # reading has come from sync_read
            self.temperature = reading
        if self.temperature > 60:
            self.status = "OVERHEATED, SHUTDOWN"
            self.level = DiagnosticStatus.ERROR
        elif self.temperature > 50 and self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OVERHEATING"
            self.level = DiagnosticStatus.WARN
        elif self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OK"
            self.level = DiagnosticStatus.OK

    def updateVoltage(self, reading=None):
        """ Update voltage by reading from servo, or by passing value from a sync read. """
        if reading == None:                     # read from device (no sync_read)
            if self.readable and not self.device.fake:
                self.voltage = self.device.getVoltage(self.id)
        else:                                   # reading has come from sync_read
            self.voltage = reading

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        if self.controller and self.controller.active():
            # under controller?
            return None
        if self.dirty:
            # compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/frame:
                cmd = self.max_speed/frame
            elif cmd < -self.max_speed/frame:
                cmd = -self.max_speed/frame
            # compute angle, apply limits
            ticks = self.angleToTicks(self.last_cmd + cmd)
            self.last_cmd = self.ticksToAngle(ticks)
            self.speed = cmd*frame
            # cap movement
            if self.last_cmd == self.desired:
                self.dirty = False
            if self.device.fake:
                self.angle = self.last_cmd
                return None
            return int(ticks)
        else:
            return None

    def setControl(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in ticks. """
        ticks = self.angleToTicks(position)
        self.desired = self.ticksToAngle(ticks)
        self.last_cmd = self.ticksToAngle(ticks)
        return int(ticks)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = self.level
        msg.message = self.status
        msg.values.append(KeyValue("Position", str(self.angle)))
        msg.values.append(KeyValue("Temperature", str(self.temperature)))
        msg.values.append(KeyValue("Voltage", str(self.voltage)))
        if self.reads + self.errors > 100:
            self.total_errors.append((self.errors*100.0)/(self.reads+self.errors))
            if len(self.total_errors) > 10:
                self.total_errors = self.total_errors[-10:]
            self.reads = 0
            self.errors = 0
        msg.values.append(KeyValue("Reads", str(self.total_reads)))
        msg.values.append(KeyValue("Error Rate", str(sum(self.total_errors)/len(self.total_errors))+"%" ))
        if self.relaxed:
            msg.values.append(KeyValue("Torque", "OFF"))
        else:
            msg.values.append(KeyValue("Torque", "ON"))
        return msg


class HobbyServo(Servo):
    """ Class to handle services and updates for a single Hobby Servo, 
        connected to an ArbotiX robocontroller. """
    def __init__(self, device, params):
        self.device = device
        self.name = name
        n = "~servos/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.neutral = rospy.get_param(n+"neutral",1500)
        self.ticks = rospy.get_param(n+"ticks",2000)
        self.rad_per_tick = radians(rospy.get_param(n+"range",180.0))/self.ticks

        self.max_angle = radians(rospy.get_param(n+"max_angle",90))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-90))
        self.max_speed = radians(rospy.get_param(n+"max_speed",90.0))

        self.invert = rospy.get_param(n+"invert",False)
        self.readable = False

        self.controller = None
        self.status = "OK"
        self.level = DiagnosticStatus.OK

        self.dirty = False             # newly updated position?
        self.angle = 0.0               # current position
        self.velocity = 0.0            # this currently doesn't provide info for hobby servos
        
        # ROS interfaces
        rospy.Subscriber(params.name+'/command', Float64, self.commandCb)

    def commandCb(self, req):
        """ Callback to set position to angle, in radians. """
        self.dirty = True
        self.angle = req.data

    def update(self, value):
        """ If dirty, update value of servo at device. """
        pass

    def interpolate(self, frame):
        if self.dirty:
            # test limits
            if self.angle < self.min_angle:
                self.angle = self.min_angle
            if self.angle > self.max_angle:
                self.angle = self.max_angle
            # send update to hobby servo
            ang = self.angle
            if self.invert:
                ang = ang * -1.0
            ticks = int(round( ang / self.rad_per_tick ))
            if not self.device.fake:
                self.device.setServo(self.id, ticks)
            self.dirty = False

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = "Joint " + self.name
        msg.level = self.level
        msg.message = self.status
        msg.values.append(KeyValue("Position", str(self.angle)))
        return msg


class Servos(dict):

    def __init__(self, device):
        self.device = device
        self.fake = device.fake

        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixels = dict()
        for name in dynamixels.keys():
            self.dynamixels[name] = Servo(device, name)

        hobbyservos = rospy.get_param("~servos", dict())
        self.hobbyservos = dict()
        for name in hobbyservos.keys():
            self.hobbyservos[name] = HobbyServo(device, name)

        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta

        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta

    def values(self):
        return self.dynamixels.values() + self.hobbyservos.values()

    def __getitem__(self, key):
        try:
            return self.dynamixels[key]
        except:
            return self.hobbyservos[key]

    def __setitem__(self, key, value):
        try:
            k = self.dynamixels[key]
            k.update(value)
        except:        
            k = self.hobbyservos[key]
            k.update(value)
    
    def update(self, sync=True):
        """ Read servo positions. """
        if rospy.Time.now() > self.r_next and not self.fake:
            #try:
                if sync:
                    # arbotix/servostik/wifi board sync_read
                    synclist = list()
                    for servo in self.dynamixels.values():
                        if servo.readable:
                            synclist.append(servo.id)
                        else:
                            servo.update(-1)
                    if len(synclist) > 0:
                        val = self.device.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                        if val: 
                            for servo in self.dynamixels.values():
                                try:
                                    i = synclist.index(servo.id)*2
                                    servo.update(val[i]+(val[i+1]<<8))
                                except:
                                    # not a readable servo
                                    continue 
                else:
                    # direct connection, or other hardware with no sync_read capability
                    for servo in self.dynamixels.values():
                        servo.update(None)
            #except:
            #    rospy.loginfo("Error in updating positions.")  
            #    return           
                self.r_next = rospy.Time.now() + self.r_delta
                        
    def updateStats(self, sync=True):
        """ Read servo voltages, temperatures. """
        if self.fake: 
            return
        try:
            if sync:
                # arbotix/servostik/wifi board sync_read
                synclist = list()
                for servo in self.dynamixels.values():
                    if servo.readable:
                        synclist.append(servo.id)
                    else:
                        servo.update(-1)
                if len(synclist) > 0:
                    val = self.device.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
                    if val: 
                        for servo in self.dynamixels.values():
                            try:
                                i = synclist.index(servo.id)*2
                                servo.voltage = val[i]/10.0
                                servo.temperature = val[i+1]
                            except:
                                # not a readable servo
                                continue 
            else:
                # direct connection, or other hardware with no sync_read capability
                for servo in self.dynamixels.values():
                    if servo.readable:
                        val = self.device.read(servo.id, P_PRESENT_VOLTAGE, 2)
                        servo.voltage = val[0]
                        servo.temperature = val[1]
        except:
            rospy.logdebug("Error in updating stats.")  
            return           

    def interpolate(self, sync=True):
        """ Write updated positions to servos. """
        if rospy.Time.now() > self.w_next:
            if sync and not self.fake:
                syncpkt = list()
                for servo in self.dynamixels.values():
                    v = servo.interpolate(1.0/self.w_delta.to_sec())
                    if v != None:   # if was dirty
                        syncpkt.append([servo.id,int(v)%256,int(v)>>8]) 
                for servo in self.hobbyservos.values(): 
                    servo.interpolate(1.0/self.w_delta.to_sec())
                if len(syncpkt) > 0:      
                    self.device.syncWrite(P_GOAL_POSITION_L,syncpkt)
            else:
                for servo in self.dynamixels.values():
                    v = servo.interpolate(1.0/self.w_delta.to_sec())
                    if v != None:   # if was dirty      
                        self.device.setPosition(servo.id, int(v))
                for servo in self.hobbyservos.values(): 
                    servo.interpolate(1.0/self.w_delta.to_sec())
            self.w_next = rospy.Time.now() + self.w_delta



def getServosFromURDF():
    """ Get servo parameters from URDF. """
    try:
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        joints = {}
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                  continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                joints[name] = joint
        return joints
    except:
        rospy.loginfo('No URDF defined, proceeding with defaults')
        return dict()


def getServoLimits(name, joint_defaults, default_min=-150, default_max=150):
    """ Get limits of servo, from YAML, then URDF, then defaults if neither is defined. """
    min_angle = radians(default_min)
    max_angle = radians(default_max)
    
    try: 
        min_angle = joint_defaults[name]['min']
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/min_angle"))
    except:
        pass

    try: 
        max_angle = joint_defaults[name]['max']
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/max_angle"))
    except:
        pass

    return (min_angle, max_angle)


