#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from std_msgs.msg import *
from press_machine_operator import PressMachineOperator


class PressMachineOperatorMock(PressMachineOperator):
    def __init__(self, machine_prefix='press_machine'):
        self.machine_prefix = str(machine_prefix)

        # publisher
        self.pub_complete_flag           = rospy.Publisher('/' + self.machine_prefix + '_operator/complete_' + self.machine_prefix, Bool, queue_size=1)
        self.pub_complete_change_setting = rospy.Publisher('/' + self.machine_prefix + '_operator/complete_change_setting', Bool, queue_size=1)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/press_operation', Bool, self.press_operation_cb, queue_size=1)


    # override
    def press_operation(self):
        time.sleep(1)  # for correct pub/sub

        # complete flag
        msg_comp = Bool(data=True)
        self.pub_complete_flag.publish(msg_comp)
        print '[Complete] Press operation {}\n'.format(self.machine_prefix)



class HeatpressOperatorMock(PressMachineOperatorMock):
    def __init__(self):
        self.machine_prefix = 'heatpress'
        super(HeatpressOperatorMock, self).__init__(self.machine_prefix)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/change_setting',  Float32MultiArray, self.change_setting_cb, queue_size=1)


    # override
    def change_setting_cb(self, msg):
        self.change_setting()


    # override
    def change_setting(self):
        time.sleep(1)  # for correct pub/sub

        # pub complete flag
        super(HeatpressOperatorMock, self).publish_complete_change_setting()
        print '[Complete] Change setting {}\n'.format(self.machine_prefix)



class ColdpressOperatorMock(PressMachineOperatorMock):
    def __init__(self):
        self.machine_prefix = 'coldpress'
        super(ColdpressOperatorMock, self).__init__(self.machine_prefix)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/change_setting',  Float32MultiArray, self.change_setting_cb, queue_size=1)


    # override
    def change_setting_cb(self, msg):
        self.change_setting()


    # override
    def change_setting(self):
        time.sleep(1)  # for correct pub/sub

        # pub complete flag
        super(ColdpressOperatorMock, self).publish_complete_change_setting()
        print '[Complete] Change setting {}\n'.format(self.machine_prefix)
