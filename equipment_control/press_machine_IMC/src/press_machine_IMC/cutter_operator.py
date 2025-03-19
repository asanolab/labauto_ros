#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from std_msgs.msg import *
from press_machine_operator import PressMachineOperator


class CutterOperator(PressMachineOperator):
    def __init__(self):
        self.machine_prefix = 'cutter'
        super(CutterOperator, self).__init__(self.machine_prefix)

        # publisher
        ## for setting
        self.pub_press_time  = rospy.Publisher('/' + self.machine_prefix + '/write/press_time/setting',  Int32, queue_size=1)

        # subscriber
        rospy.Subscriber('/' + self.machine_prefix + '_operator/change_setting',  Int32, self.change_setting_cb, queue_size=1)


    # callback
    def change_setting_cb(self, msg):
        self.press_time_setting = msg.data
        self.change_setting(self.press_time_setting)


    # change setting
    # - press_time : [sec]
    def change_setting(self, press_time):
        print '[Start] Change setting {}'.format(self.machine_prefix)
        # limit
        if press_time > 65535:
            print 'press_time {} is above the limit'.format(press_time)
            press_time = 65535
        # set
        msg_press_time = Int32(data=round(press_time))
        self.pub_press_time.publish(msg_press_time)
        # check
        self.get_current_setting()

        print '[Complete] Change setting {}\n'.format(self.machine_prefix)


    def get_current_setting(self):
        msg_press_time_setting = rospy.wait_for_message('/' + self.machine_prefix + '/read/press_time/setting', Int32, timeout=None)
        self.press_time_setting = msg_press_time_setting.data

        print 'Current setting:'
        print ' press_time  [s]   : {}'.format(self.press_time_setting)


    # operation function (override)
    def press_operation(self):
        # to initialize self.press_time_setting
        self.get_current_setting()

        print '[Start] Press operation {}'.format(self.machine_prefix)
        # reset
        self.reset_all()
        # flag on
        self.enable_operation_start()
        self.enable_carry_in_complete()

        # wait until finish press
        print 'Waiting for pressing finished'

        # バグ?
        # - carry_out_possibleが自動的にonにならないので手動でonにする
        # -- HP/CPは自動的にonになる. タイムチャートを見るとprogram_endに連動している気がするが,cutterではなくなっている.
        # -- 手動でプレス時間だけ待つようにしているが,待ち時間にずれが生じている.
        # -- 仕様書を見ると, press_timeのところにx100msと書いてあるので, 本当であれば本体仕様に応じて時間を合わせる必要がある.
        self.carry_out_possible = True
        print ' carry_out_possible turns: {}'.format(self.carry_out_possible)
        time.sleep(3)  # pressが下に降りるまでの時間
        time.sleep(self.press_time_setting)  # pressの時間

        self.enable_carry_out_complete()  # this flag enable is necessary to finish the operation.

        self.reset_human_flag()  # nowait reset within 500ms is neccessary
        self.carry_out_possible = False  # for next press

        # complete flag
        msg_comp = Bool(data=True)
        self.pub_complete_flag.publish(msg_comp)

        print '[Complete] Press operation {}\n'.format(self.machine_prefix)
