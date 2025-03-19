#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from std_msgs.msg import *
from press_machine_operator import PressMachineOperator


class HeatpressOperator(PressMachineOperator):
    def __init__(self):
        self.machine_prefix = 'heatpress'
        super(HeatpressOperator, self).__init__(self.machine_prefix)

        self.temp_top_setting = 0
        self.temp_bottom_setting = 0
        self.press_force_kN_setting = 0
        self.press_force_setting = 0
        self.press_time_setting = 0

        self.msg_temp_top = None
        self.msg_temp_bottom = None
        self.msg_press_force = None
        self.msg_press_time = None

        self.msg_temp_top_setting = None
        self.msg_temp_bottom_setting = None
        self.msg_press_force_setting = None
        self.msg_press_time_setting = None

        # publisher
        ## for setting
        self.pub_temp_top    = rospy.Publisher('/' + self.machine_prefix + '/write/seg0/temp_top/setting',    Int16, queue_size=1)
        self.pub_temp_bottom = rospy.Publisher('/' + self.machine_prefix + '/write/seg0/temp_bottom/setting', Int16, queue_size=1)
        self.pub_press_force = rospy.Publisher('/' + self.machine_prefix + '/write/seg0/press_force/setting', Int16, queue_size=1)
        self.pub_press_time  = rospy.Publisher('/' + self.machine_prefix + '/write/seg0/press_time/setting',  Int32, queue_size=1)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/change_setting',  Float32MultiArray, self.change_setting_cb, queue_size=1)


    # callback
    def change_setting_cb(self, msg):
        self.change_setting(msg.data[0], msg.data[1], msg.data[2], msg.data[3])


    # change setting
    # - seg        : Segment No
    # - temp_top   : [degC]
    # - temp_bottom: [degC]
    # - press_force: [kN]
    # - press_time : [sec]
    def change_setting(self, temp_top, temp_bottom, press_force_kN, press_time):
        print '[Start] Change setting {}'.format(self.machine_prefix)
        # limit
        if temp_top > 999:
            print 'temp_top {} is above the limit'.format(temp_top)
            self.temp_top_setting = 999
        else:
            self.temp_top_setting = round(temp_top)

        if temp_bottom > 999:
            print 'temp_bottom {} is above the limit'.format(temp_bottom)
            self.temp_bottom_setting = 999
        else:
            self.temp_bottom_setting = round(temp_bottom)

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
        self.msg_temp_top    = Int16(data=self.temp_top_setting)  # convert to integer
        self.msg_temp_bottom = Int16(data=self.temp_bottom_setting)
        self.msg_press_force = Int16(data=self.press_force_setting)
        self.msg_press_time  = Int32(data=self.press_time_setting)
        self.pub_temp_top.publish(self.msg_temp_top)
        self.pub_temp_bottom.publish(self.msg_temp_bottom)
        self.pub_press_force.publish(self.msg_press_force)
        self.pub_press_time.publish(self.msg_press_time)
        # check
        self.get_current_setting()

        # pub complete flag
        super(HeatpressOperator, self).publish_complete_change_setting()

        print '[Complete] Change setting {}\n'.format(self.machine_prefix)


    def change_setting_polyethylene(self):
        self.change_setting(180, 180, 0.3, 300)


    def get_current_setting(self):
        self.msg_temp_top_setting    = rospy.wait_for_message('/' + self.machine_prefix + '/read/temp_top/setting',    Int16, timeout=None)
        self.msg_temp_bottom_setting = rospy.wait_for_message('/' + self.machine_prefix + '/read/temp_bottom/setting', Int16, timeout=None)
        self.msg_press_force_setting = rospy.wait_for_message('/' + self.machine_prefix + '/read/press_force/setting', Int16, timeout=None)
        self.msg_press_time_setting  = rospy.wait_for_message('/' + self.machine_prefix + '/read/press_time/setting',  Int32, timeout=None)
        self.temp_top_setting       = self.msg_temp_top_setting.data
        self.temp_bottom_setting    = self.msg_temp_bottom_setting.data
        self.press_force_kN_setting = self.msg_press_force_setting.data / 100.0
        self.press_time_setting     = self.msg_press_time_setting.data

        print 'Current setting:'
        print ' temp_top    [degC]: {}'.format(self.temp_top_setting)
        print ' temp_bottom [degC]: {}'.format(self.temp_bottom_setting)
        print ' press_force [kN]  : {}'.format(self.press_force_kN_setting)
        print ' press_time  [s]   : {}'.format(self.press_time_setting)
