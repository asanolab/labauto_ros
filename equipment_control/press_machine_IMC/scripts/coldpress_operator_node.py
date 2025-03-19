#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from press_machine_IMC.coldpress_operator import ColdpressOperator


def main():
    rospy.init_node('coldpress_operator_node', anonymous=True)

    # Generate instance
    cp_operator = ColdpressOperator()

    # Run
    rospy.spin()


if __name__ == '__main__':
    main()
