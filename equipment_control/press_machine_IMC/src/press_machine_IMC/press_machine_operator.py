#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from std_msgs.msg import *
from __future__ import print_function


class PressMachineOperator(object):
    def __init__(self, machine_prefix='press_machine'):
        self.temp_top_setting = 0
        self.temp_bottom_setting = 0
        self.press_force_setting = 0
        self.press_time_setting = 0
        self.carry_out_possible = False

        self.machine_prefix = str(machine_prefix)

        self.msg_complete_change_setting = None

        # publisher
        # - publish topic to plc_ros_bridge node according to funcions
        ## flag - operation PC
        self.pub_operation_start    = rospy.Publisher('/' + self.machine_prefix + '/write/operation_start',    Bool, queue_size=1)
        self.pub_operation_reset    = rospy.Publisher('/' + self.machine_prefix + '/write/operation_reset',    Bool, queue_size=1)
        self.pub_carry_in_complete  = rospy.Publisher('/' + self.machine_prefix + '/write/carry_in_complete',  Bool, queue_size=1)
        self.pub_carry_out_complete = rospy.Publisher('/' + self.machine_prefix + '/write/carry_out_complete', Bool, queue_size=1)
        ## flag - machine
        self.pub_start_waiting      = rospy.Publisher('/' + self.machine_prefix + '/write/start_waiting',      Bool, queue_size=1)
        self.pub_fullauto_running   = rospy.Publisher('/' + self.machine_prefix + '/write/fullauto_running',   Bool, queue_size=1)
        self.pub_program_start      = rospy.Publisher('/' + self.machine_prefix + '/write/program_start',      Bool, queue_size=1)
        self.pub_program_end        = rospy.Publisher('/' + self.machine_prefix + '/write/program_end',        Bool, queue_size=1)
        self.pub_carry_in_possible  = rospy.Publisher('/' + self.machine_prefix + '/write/carry_in_possible',  Bool, queue_size=1)
        self.pub_carry_out_possible = rospy.Publisher('/' + self.machine_prefix + '/write/carry_out_possible', Bool, queue_size=1)
        self.pub_fullauto_end       = rospy.Publisher('/' + self.machine_prefix + '/write/fullauto_end',       Bool, queue_size=1)
        ## flag - others
        self.pub_reset_press_force  = rospy.Publisher('/' + self.machine_prefix + '/write/reset_press_force',  Bool, queue_size=1)
        ## complete flag
        self.pub_complete_flag           = rospy.Publisher('/' + self.machine_prefix + '_operator/complete_' + self.machine_prefix, Bool, queue_size=1)
        self.pub_complete_change_setting = rospy.Publisher('/' + self.machine_prefix + '_operator/complete_change_setting', Bool, queue_size=1)

        # subscriber
        # - subscribe to execute functions
        rospy.Subscriber('/' + self.machine_prefix + '_operator/press_operation', Bool, self.press_operation_cb, queue_size=1)


    # callback
    def press_operation_cb(self, msg):
        if msg.data == True:
            self.press_operation()


    # publish
    def publish_complete_change_setting(self):
        self.msg_complete_change_setting = Bool(data=True)
        self.pub_complete_change_setting.publish(self.msg_complete_change_setting)


    # for reset
    ## oneshot reset
    def reset_operation(self):
        print('reset_operation')
        # on and off
        self.enable_operation_reset()
        time.sleep(0.5)
        self.disable_operation_reset()


    def reset_machine_flag(self):
        print('reset_machine_flag')
        self.disable_start_waiting()
        self.disable_fullauto_running()
        self.disable_carry_in_possible()
        self.disable_carry_out_possible()
        self.disable_fullauto_end()


    def reset_human_flag(self):
        print("reset_human_flag")
        self.disable_operation_start()
        self.disable_carry_in_complete()
        self.disable_carry_out_complete()


    def reset_press_force(self):
        print("reset_press_force")
        self.enable_reset_press_force()


    def reset_all(self):
        print("reset_all")
        self.reset_operation()
        self.reset_human_flag()
        self.reset_machine_flag()
        self.reset_press_force()


    # enable/disable flags
    ## operation flag
    def enable_operation_start(self):
        print(' enable operation_start')
        msg = Bool(data=True)
        self.pub_operation_start.publish(msg)
        time.sleep(0.1)


    def disable_operation_start(self):
        print(' disable operation_start')
        msg = Bool(data=False)
        self.pub_operation_start.publish(msg)
        time.sleep(0.1)


    def enable_operation_reset(self):
        print(' enable operation_reset')
        msg = Bool(data=True)
        self.pub_operation_reset.publish(msg)
        time.sleep(0.1)


    def disable_operation_reset(self):
        print(' disable operation_reset')
        msg = Bool(data=False)
        self.pub_operation_reset.publish(msg)
        time.sleep(0.1)


    def enable_carry_in_complete(self):
        print(' enable carry_in_complete')
        msg = Bool(data=True)
        self.pub_carry_in_complete.publish(msg)
        time.sleep(0.1)


    def disable_carry_in_complete(self):
        print(' disable carry_in_complete')
        msg = Bool(data=False)
        self.pub_carry_in_complete.publish(msg)
        time.sleep(0.1)


    def enable_carry_out_complete(self):
        print(' enable carry_out_complete')
        msg = Bool(data=True)
        self.pub_carry_out_complete.publish(msg)
        time.sleep(0.1)


    def disable_carry_out_complete(self):
        print(' disable carry_out_complete')
        msg = Bool(data=False)
        self.pub_carry_out_complete.publish(msg)
        time.sleep(0.1)


    ## machine flag
    def enable_start_waiting(self):
        print(' enable start_waiting')
        msg = Bool(data=True)
        self.pub_start_waiting.publish(msg)
        time.sleep(0.1)


    def disable_start_waiting(self):
        print(' disable start_waiting')
        msg = Bool(data=False)
        self.pub_start_waiting.publish(msg)
        time.sleep(0.1)


    def enable_fullauto_running(self):
        print(' enable fullauto_running')
        msg = Bool(data=True)
        self.pub_fullauto_running.publish(msg)
        time.sleep(0.1)


    def disable_fullauto_running(self):
        print(' disable fullauto_running')
        msg = Bool(data=False)
        self.pub_fullauto_running.publish(msg)
        time.sleep(0.1)


    def enable_carry_in_possible(self):
        print(' enable carry_in_possible')
        msg = Bool(data=True)
        self.pub_carry_in_possible.publish(msg)
        time.sleep(0.1)


    def disable_carry_in_possible(self):
        print(' disable carry_in_possible')
        msg = Bool(data=False)
        self.pub_carry_in_possible.publish(msg)
        time.sleep(0.1)


    def enable_carry_out_possible(self):
        print(' enable carry_out_possible')
        msg = Bool(data=True)
        self.pub_carry_out_possible.publish(msg)
        time.sleep(0.1)


    def disable_carry_out_possible(self):
        print(' disable carry_out_possible')
        msg = Bool(data=False)
        self.pub_carry_out_possible.publish(msg)
        time.sleep(0.1)


    def enable_fullauto_end(self):
        print(' enable fullauto_end')
        msg = Bool(data=True)
        self.pub_fullauto_end.publish(msg)
        time.sleep(0.1)


    def disable_fullauto_end(self):
        print(' disable fullauto_end')
        msg = Bool(data=False)
        self.pub_fullauto_end.publish(msg)
        time.sleep(0.1)


    ## other flag
    def enable_reset_press_force(self):
        print(' enable reset_press_force')
        msg = Bool(data=True)
        self.pub_reset_press_force.publish(msg)
        time.sleep(0.1)


    # operation function
    def press_operation(self):
        print('[Start] Press operation {}'.format(self.machine_prefix))
        # reset
        self.reset_all()
        # flag on
        self.enable_operation_start()
        self.enable_carry_in_complete()
        # carry_in_possible will automaitcally rise when reaching setting temp, and then start pressing

        # wait until finish press
        print('Waiting for pressing finished (= carry_out_possible turns true)')

        while not self.carry_out_possible:
            msg_cop = rospy.wait_for_message('/' + self.machine_prefix + '/read/carry_out_possible', Bool, timeout=None)
            self.carry_out_possible = msg_cop.data

        print(' carry_out_possible turns: {}'.format(self.carry_out_possible))
        self.enable_carry_out_complete()  # this flag enable is necessary to finish the operation.
        self.reset_human_flag()  # nowait reset within 500ms is neccessary
        self.carry_out_possible = False  # for next press

        # complete flag
        msg_comp = Bool(data=True)
        self.pub_complete_flag.publish(msg_comp)

        print('[Complete] Press operation {}\n'.format(self.machine_prefix))
