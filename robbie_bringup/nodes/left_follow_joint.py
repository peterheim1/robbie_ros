#!/usr/bin/env python
     
# freely inspired by http://www.ros.org/wiki/arbotix_python/follow_controller
     
import roslib
     
import rospy, actionlib
     
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
     
     
class JointSubscriber():
       
    def __init__(self,joint):
           
        self.ns ='robbie'
        rospy.Subscriber('robbie/' + joint + '/state', JointControllerState, self.joint_state_cb)
           
        rospy.loginfo('Subscribing for %s joint state.',joint)
           
        self.joint_name = joint
        self.state = JointControllerState()
        self.received = False
           
           
    def joint_state_cb(self,msg):
           
        if self.received is False:
               
            self.received = True
           
        self.state = msg
           
    def get_position(self):
           
        return self.state.process_value
           
           
class JointCommander():
       
    def __init__(self,joint):
           
        self.joint_name = joint
        self.pub = rospy.Publisher('robbie/' + joint + '/command',Float64, queue_size=5)
           
    def command(self,pos):
           
        rospy.loginfo('publishing, joint ' + self.joint_name + ', value ' + str(pos))
        #rospy.loginfo('publishing, joint ' +pos)      
        self.pub.publish(pos)
        #rospy.loginfo(self.pub)
     
class FollowController():
     
    def __init__(self):
           
        self.ns = 'left_arm_controller'
           
        
        self.joints = rospy.get_param('left_joints', '')
        namespace = rospy.get_namespace()
        rospy.loginfo('Configured for ' + str(len(self.joints)) + 'joints')
        self.joint_subs = [JointSubscriber(name) for name in self.joints]
        self.joint_pubs = [JointCommander(name) for name in self.joints]
           
        self.joints_names = []
           
        for idx in range(0,len(self.joints)):
           
            self.joints_names.append(self.joints[idx])
            #rospy.logerr(self.joints_names)
     
        # action server
        
        self.name = self.ns + '/follow_joint_trajectory'
        self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
     
        rospy.loginfo("Started FollowController ("+self.name+"). Joints: " + str(self.joints))
     
    def startup(self):
        self.server.start()
     
    def actionCb(self, goal):
           
        rospy.loginfo(self.name + ": left arm Action goal recieved.")
        traj = goal.trajectory
     
        if set(self.joints_names) != set(traj.joint_names):
            msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names )
            rospy.logerr(msg)
            rospy.logerr(self.joints_names)
            self.server.set_aborted(text=msg)
            return
     
        if not traj.points:
            msg = "Trajectory empty."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
     
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
     
        if self.executeTrajectory(traj):  
            self.server.set_succeeded()
            rospy.loginfo('Executed.')
        else:
            rospy.logerr('Execution failed.')
            self.server.set_aborted(text="Execution failed.")
     
         
     
    def executeTrajectory(self, traj):
           
        rospy.loginfo("Executing trajectory with " + str(len(traj.points)) + ' point(s)')
           
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints_names]
        except ValueError as val:
            return False
           
        time = rospy.Time.now()
        start = traj.header.stamp
           
        #success = True
           
        for point in traj.points:
               
            if self.server.is_preempt_requested():
                   
                rospy.loginfo('Stopping arm movement')
                   
                self.server.set_preempted()
                #success = False
                break
               
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
                   
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
                               
            for i in range(0,len(self.joints)):
                   
                self.joint_pubs[i].command(desired[i])
     
     
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                   
                rospy.sleep(0.01)
     
                   
        return True
     
#===============================================================================
#    def active(self):
#        """ Is controller overriding servo internal control? """
#        return self.server.is_active() or self.executing
#
#    def getDiagnostics(self):
#        """ Get a diagnostics status. """
#        msg = DiagnosticStatus()
#        msg.name = self.name
#        msg.level = DiagnosticStatus.OK
#        msg.message = "OK"
#        if self.active():
#            msg.values.append(KeyValue("State", "Active"))
#        else:
#            msg.values.append(KeyValue("State", "Not Active"))
#        return msg
#===============================================================================
       
       
if __name__ == '__main__':
       
    rospy.init_node('follow_joint_controller', anonymous=False)
     
    rospy.loginfo('left_Follow joint action node.')
       
    c = FollowController()
       
    rospy.loginfo('Starting left arm action server')
       
    c.startup()
       
    rospy.loginfo('Spinning')
       
    rospy.spin()


