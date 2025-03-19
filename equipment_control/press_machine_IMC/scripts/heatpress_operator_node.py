#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from press_machine_IMC.heatpress_operator import HeatpressOperator


def main():
    rospy.init_node('heatpress_operator_node', anonymous=True)

    # Generate instance
    hp_operator = HeatpressOperator()

    # Run
    rospy.spin()


if __name__ == '__main__':
    main()
