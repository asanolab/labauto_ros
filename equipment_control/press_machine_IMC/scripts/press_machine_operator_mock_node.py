#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from press_machine_IMC.press_machine_operator_mock import HeatpressOperatorMock
from press_machine_IMC.press_machine_operator_mock import ColdpressOperatorMock


def main():
    rospy.init_node('press_machine_operator_mock_node', anonymous=True)

    # Generate instance
    hp_operator_mock = HeatpressOperatorMock()
    cp_operator_mock = ColdpressOperatorMock()

    # Run
    rospy.spin()


if __name__ == '__main__':
    main()
