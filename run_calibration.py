#!/usr/bin/env python

from tasks.calibrate import Calibration

if __name__ == '__main__':
    calibration = Calibration(tool_orientation=[1, 0, 0, 0])
    calibration.run()
