# The MIT License (MIT)
#
# Copyright (c) 2016 Utthawut Bootdee
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Common define"""

import numpy as np

X_AXIS, Y_AXIS, Z_AXIS = 0, 1, 2
DIM_RATIO = 10  # metric multiply 10 = display on LCD

# Border Range LOWER/LEFT  <  x  <  UPPER/RIGHT
POSIT_LOWER, POSIT_UPPER = 0, 1
POSIT_LEFT,  POSIT_RIGHT = 0, 1

DEF_BALL_R = 0.028575
DEF_BALL_MASS = 0.17
DEF_CUE_MASS = 0.25

POS_COSINE_EPSIL = 0.0000001
NEG_COSINE_EPSIL = -0.0000001

POS_VELO_EPSIL = 0.001
NEG_VELO_EPSIL = -0.001

POS_SLIDE_VELO_EPSIL = 0.001
NEG_SLIDE_VELO_EPSIL = -0.001
POS_ROLL_VELO_EPSIL = 0.00001
NEG_ROLL_VELO_EPSIL = -0.00001

POS_EPSIL = 0.001234
NEG_EPSIL = -0.001234

POS_EPSIL_PREDICT = POS_EPSIL/100
NEG_EPSIL_PREDICT = NEG_EPSIL/100

SMALL_POS_EPSIL = POS_EPSIL/100
SMALL_NEG_EPSIL = NEG_EPSIL/100


MAX_TIME = 10
MAX_EVENT_TIME = 9
ZERO_VECTOR = np.array([0, 0, 0], dtype = float)
Z_UP_VECTOR = np.array([0, 0, 1], dtype = float)

SIN_30 = np.sin(np.radians(30))
COS_30 = np.cos(np.radians(30))

NOR_SAMP_PERIOD = 0.032  # 32 ms
NOR_FRAME_PER_SEC = 1/NOR_SAMP_PERIOD  # 30.xx frame/sec

SAMP_PERIOD_EPSIL = NOR_SAMP_PERIOD/100

COMPANION_METRIX = np.zeros((4, 4), dtype = float)
COMPANION_METRIX[1:, :-1] = np.eye(3)
