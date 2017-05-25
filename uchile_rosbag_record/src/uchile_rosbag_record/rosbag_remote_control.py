#!/usr/bin/env python

__author__ = "Luz Martinez"

import rospy
import roslib
import time
import signal
import psutil
import subprocess

from threading import Thread, Lock

from uchile_srvs.srv import RosbagRecord, RosbagRecordResponse

class ROSRecordConnector(Thread):
    """
    Base class for ROSRecordConnector
    """
    _type = "rosbag_record"

    def __init__(self):
        Thread.__init__(self)
        self._description = "Rosbag Record"

        self.namespace_prefix = (rospy.get_namespace()+'/'+ROSRecordConnector._type+'/').replace('//','/')
        self.active_service = rospy.Service(self.namespace_prefix+'active', RosbagRecord, self._active)

        self.msglimit = None#_msglimit #TODO
        self._control = {'id_name': ""}

    def _active(self, req):
        if req.status:
            self._control[req.id_name] = {'status': True, 'process': None, 'topics': " ".join(req.topics), 'bag_filename': req.bag_filename}
            rospy.loginfo("Turning rosbag record. id : "+req.id_name)
        else:
            rospy.loginfo("Turning off rosbag record. id : "+req.id_name)

        if req.id_name in self._control:
            self.record_handling(req.status, self._control[req.id_name])

        return RosbagRecordResponse()


    def record_handling(self, record, control_node=None):      # already recording 
        if (record and control_node['process'] is not None and control_node['process'].is_recording):
            return False 

        if (not record and control_node['process'] is None):   # already stopped
            return False  

          
        if record and control_node['bag_filename'] is not None:
            control_node['process'] = RecordBAG(control_node['bag_filename'], control_node['topics'], self.msglimit)
            control_node['process'].start()
            return True
        else:
            if control_node['process'] is not None:
                if control_node['process'].stop():
                    control_node['process'] = None
                    return True
                else:
                    return False
            return True



class RecordBAG(Thread):
    def __init__(self, _name, _scope, _msg_limit=None):
        Thread.__init__(self)
        self.name = _name.strip()
        self.scope = _scope.strip()
        self.is_recording = False
        self.process = None
        self.msg_limit = _msg_limit

    def stop(self):
        if not self.is_recording: # already stopped
            return True
        self.is_recording = False

        try:
            p = psutil.Process(self.process.pid)
            try:
                for sub in p.get_children(recursive=True):
                    sub.send_signal(signal.SIGINT)
                    self.process.send_signal(signal.SIGINT)
                return True
            except AttributeError:
                # newer psutils
                for sub in p.children(recursive=True):
                    sub.send_signal(signal.SIGINT)
                    self.process.send_signal(signal.SIGINT)
                return True
        except Exception as e:
            rospy.logwar( "The process is already dead? " )
            return False

    def run(self):
        rospy.loginfo( " Recording topics: {} ".format( self.scope) )
        rospy.loginfo( " Filename:  {}.bag " .format(self.name) )
        
        if self.msg_limit is not None:
            rospy.loginfo( "  stopping after {} messages" .format(str(self.msg_limit) ) )
            self.process = subprocess.Popen("rosbag record -l %s -O %s.bag %s" % (str(self.msg_limit), self.name, self.scope), shell=True)
        else:
            self.process = subprocess.Popen("rosbag record -O %s.bag %s" % (self.name, self.scope), shell=True)
        self.is_recording = True
        self.process.wait()
        rospy.loginfo( " Recording: {} stopped" .format( self.scope) )
        self.is_recording = False



if __name__ == '__main__':

    rospy.init_node('RosbagControl')

    r = ROSRecordConnector( )
    r.start()
    rospy.spin()