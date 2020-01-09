#!/usr/bin/env python
import numpy as np

from tasks.calibrate import Calibration

if __name__ == '__main__':
    calibration = Calibration(tool_orientation=[-np.pi / 2, 0, 0])
    calibration.run()
