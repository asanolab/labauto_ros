#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from std_msgs.msg import *
from press_machine_operator import PressMachineOperator


class ColdpressOperator(PressMachineOperator):
    def __init__(self):
        self.machine_prefix = 'coldpress'
        super(ColdpressOperator, self).__init__(self.machine_prefix)

        self.press_force_kN_setting = 0
        self.press_force_setting = 0
        self.press_time_setting = 0

        self.msg_press_force = None
        self.msg_press_time = None

        self.msg_press_force_setting = None
        self.msg_press_time_setting = None

        # publisher
        ## for setting
        self.pub_press_force = rospy.Publisher('/' + self.machine_prefix + '/write/press_force/setting', Int16, queue_size=1)
        self.pub_press_time  = rospy.Publisher('/' + self.machine_prefix + '/write/press_time/setting',  Int32, queue_size=1)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/change_setting',  Float32MultiArray, self.change_setting_cb, queue_size=1)


    # callback
    def change_setting_cb(self, msg):
        self.change_setting(msg.data[0], msg.data[1])


    # change setting
    # - press_force: [kN]
    # - press_time : [sec]
    def change_setting(self, press_force_kN, press_time):
        print '[Start] Change setting {}'.format(self.machine_prefix)
        # limit
        if press_force_kN > 2.0:
            print 'press_force_kN {} is above the limit'.format(press_force_kN)
            self.press_force_kN_setting = 2.0
        else:
            self.press_force_kN_setting = press_force_kN  # not round() here
        self.press_force_setting = int(self.press_force_kN_setting * 100)  # by manual sheet

        if press_time > 65535:
            print 'press_time {} is above the limit'.format(press_time)
            self.press_time_setting = 65535
        else:
            self.press_time_setting = round(press_time)

        # set
        self.msg_press_force = Int16(data=self.press_force_setting)
        self.msg_press_time  = Int32(data=self.press_time_setting)
        self.pub_press_force.publish(self.msg_press_force)
        self.pub_press_time.publish(self.msg_press_time)
        # check
        self.get_current_setting()

        # pub complete flag
        super(ColdpressOperator, self).publish_complete_change_setting()

        print '[Complete] Change setting {}\n'.format(self.machine_prefix)


    def get_current_setting(self):
        self.msg_press_force_setting = rospy.wait_for_message('/' + self.machine_prefix + '/read/press_force/setting', Int16, timeout=None)
        self.msg_press_time_setting  = rospy.wait_for_message('/' + self.machine_prefix + '/read/press_time/setting',  Int32, timeout=None)
        self.press_force_kN_setting = self.msg_press_force_setting.data / 100.0
        self.press_time_setting     = self.msg_press_time_setting.data

        print 'Current setting:'
        print ' press_force [kN]  : {}'.format(self.press_force_kN_setting)
        print ' press_time  [s]   : {}'.format(self.press_time_setting)
