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

"""Table for Pool or Snooker"""

import common
import numpy as np


# Constant for table properties


class LineEq(object):
    A_COEF = 0
    B_COEF = 1
    C_COEF = 2


class LineSeg(object):
    START = 0
    END = 1


class TableType(object):
    POOL = 0
    SNOOKER = 1


class GameType(object):
    POOL_8_BALL = 0
    POOL_9_BALL = 1
    SNOOKER = 2


class TableSize(object):
    SEVEN_FT = 7
    EIGHT_FT = 8
    NINE_FT = 9


class PockRailIndex(object):
    BL_HORIZON = 0
    BL_VERTICAL = 1
    BR_HORIZON = 2
    BR_VERTICAL = 3
    TL_HORIZON = 4
    TL_VERTICAL = 5
    TR_HORIZON = 6
    TR_VERTICAL = 7
    L_UPPER = 8
    L_LOWER = 9
    R_UPPER = 10
    R_LOWER = 11


class RailIndex(object):
    LEFT = 0
    RIGHT = 1
    TOP = 2
    BOTTOM = 3


class Pool8ftDetail(object):
    BALL_R = 0.028575
    WIDTH = 1.097  # 1.096518
    LENGTH = 2.265  # 2.264918
    HEIGHT = 0.762
    
    VIRTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL = (np.array([-0.56838, -1.18402,	HEIGHT+BALL_R]),
                                              np.array([-0.49677, -1.11218, HEIGHT+BALL_R]))
    VIRTUAL_BL_CORNER_POCK_RAIL_VERTICAL = (np.array([-0.60053, -1.14524, HEIGHT+BALL_R]),
                                            np.array([-0.52869, -1.08146, HEIGHT+BALL_R]))
    VIRTUAL_L_SIDE_POCK_RAIL_UPPER = (np.array([-0.60972, 0.02362, HEIGHT+BALL_R]),
                                      np.array([-0.53968, 0.04804, HEIGHT+BALL_R]))
    VIRTUAL_L_SIDE_POCK_RAIL_LOWER = (np.array([-0.60972, -0.02362,	HEIGHT+BALL_R]),
                                      np.array([-0.53968, -0.04804, HEIGHT+BALL_R]))
    
    ACTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL = (np.array([-0.54828, -1.20432, HEIGHT+BALL_R]),
                                             np.array([-0.47667, -1.13248, HEIGHT+BALL_R]))
    ACTUAL_BL_CORNER_POCK_RAIL_VERTICAL = (np.array([-0.62012, -1.12465, HEIGHT+BALL_R]),
                                           np.array([-0.54828, -1.06087, HEIGHT+BALL_R]))
    ACTUAL_L_SIDE_POCK_RAIL_UPPER = (np.array([-0.61832,  0.0508, HEIGHT+BALL_R]),
                                     np.array([-0.54828, 0.07522, HEIGHT+BALL_R]))
    ACTUAL_L_SIDE_POCK_RAIL_LOWER = (np.array([-0.61832, -0.0508, HEIGHT+BALL_R]),
                                     np.array([-0.54828, -0.07522, HEIGHT+BALL_R]))
    
    ACTUAL_BL_CORNER_POCK_CENTER = np.array([-0.5842, -1.1684, HEIGHT+BALL_R])
    ACTUAL_L_SIDE_POCK_CENTER = np.array([-0.61832, 0, HEIGHT+BALL_R])
    
    RAIL_BORDER_Y_POSIT_UPPER = (0.07522, 1.06087)
    RAIL_BORDER_Y_POSIT_LOWER = (-RAIL_BORDER_Y_POSIT_UPPER[common.POSIT_UPPER],
                                 -RAIL_BORDER_Y_POSIT_UPPER[common.POSIT_LOWER])
    
    RAIL_BORDER_X_POSIT = (-0.47667, 0.47667)
    
    POCKET_WIDTH = 0.1016
    
    
class BilliardTable(object):
    
    @classmethod    
    def set_virtual_pock_rail(cls,
                              bl_corner_pock_rail_horizon_start, bl_corner_pock_rail_horizon_end,
                              bl_corner_pock_rail_vertical_start, bl_corner_pock_rail_vertical_end,
                              l_side_pock_rail_upper_start, l_side_pock_rail_upper_end,
                              l_side_pock_rail_lower_start, l_side_pock_rail_lower_end):
            
        # Virtual Corner Rail Pocket tuple(0) is start-point,
        # tuple(1) is end-point The reference is bottom-left-corner-rail, the others is mirror
        virtual_bl_corner_pock_rail_horizontal = (bl_corner_pock_rail_horizon_start, bl_corner_pock_rail_horizon_end)
        virtual_bl_corner_pock_rail_vertical = (bl_corner_pock_rail_vertical_start, bl_corner_pock_rail_vertical_end)

        virtual_br_corner_pock_rail_horizontal = (np.array(
            [-virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                  np.array(
            [-virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        virtual_br_corner_pock_rail_vertical = (np.array(
            [-virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                                np.array(
            [-virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        virtual_tl_corner_pock_rail_horizontal = (np.array(
            [virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             -virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                  np.array(
            [virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             -virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        virtual_tl_corner_pock_rail_vertical = (np.array(
            [virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             -virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                                np.array(
            [virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             -virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        virtual_tr_corner_pock_rail_horizontal = (np.array(
            [-virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             -virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                  np.array(
            [-virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             -virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        virtual_tr_corner_pock_rail_vertical = (np.array(
            [-virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             -virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                                np.array(
            [-virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             -virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        # Virtual Side Rail Pocket tuple(0) is start-point,
        # tuple(1) is end-point The reference is left-side-rail, another is mirror
        virtual_l_side_pock_rail_upper = (l_side_pock_rail_upper_start, l_side_pock_rail_upper_end)
        virtual_l_side_pock_rail_lower = (l_side_pock_rail_lower_start, l_side_pock_rail_lower_end)

        virtual_r_side_pock_rail_upper = (np.array([-virtual_l_side_pock_rail_upper[LineSeg.START][common.X_AXIS],
                                                    virtual_l_side_pock_rail_upper[LineSeg.START][common.Y_AXIS],
                                                    virtual_l_side_pock_rail_upper[LineSeg.START][common.Z_AXIS]]),
                                          np.array([-virtual_l_side_pock_rail_upper[LineSeg.END][common.X_AXIS],
                                                    virtual_l_side_pock_rail_upper[LineSeg.END][common.Y_AXIS],
                                                    virtual_l_side_pock_rail_upper[LineSeg.END][common.Z_AXIS]]))
        virtual_r_side_pock_rail_lower = (np.array([-virtual_l_side_pock_rail_lower[LineSeg.START][common.X_AXIS],
                                                    virtual_l_side_pock_rail_lower[LineSeg.START][common.Y_AXIS],
                                                    virtual_l_side_pock_rail_lower[LineSeg.START][common.Z_AXIS]]),
                                          np.array([-virtual_l_side_pock_rail_lower[LineSeg.END][common.X_AXIS],
                                                    virtual_l_side_pock_rail_lower[LineSeg.END][common.Y_AXIS],
                                                    virtual_l_side_pock_rail_lower[LineSeg.END][common.Z_AXIS]]))
                          
        # Virtual Corner Rail Pocket, Straight line equation
        virtual_eq_bl_corner_pock_rail_horizontal = (
            virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS] -
            virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
            virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] -
            virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
            (virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS] *
             virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS]) -
            (virtual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] *
             virtual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS]))

        virtual_eq_bl_corner_pock_rail_vertical = (
            virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS] -
            virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
            virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] -
            virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
            (virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS] *
             virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS]) -
            (virtual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] *
             virtual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_br_corner_pock_rail_horizontal = (
            virtual_br_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS] -
            virtual_br_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
            virtual_br_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] -
            virtual_br_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
            (virtual_br_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS] *
             virtual_br_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS]) -
            (virtual_br_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] *
             virtual_br_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_br_corner_pock_rail_vertical = (
            virtual_br_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS] -
            virtual_br_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
            virtual_br_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] -
            virtual_br_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
            (virtual_br_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS] *
             virtual_br_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS]) -
            (virtual_br_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] *
             virtual_br_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS]))

        virtual_eq_tl_corner_pock_rail_horizontal = (
            virtual_tl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS] -
            virtual_tl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
            virtual_tl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] -
            virtual_tl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
            (virtual_tl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS] *
             virtual_tl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS]) -
            (virtual_tl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] *
             virtual_tl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_tl_corner_pock_rail_vertical = (
            virtual_tl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS] -
            virtual_tl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
            virtual_tl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] -
            virtual_tl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
            (virtual_tl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS] *
             virtual_tl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS]) -
            (virtual_tl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] *
             virtual_tl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_tr_corner_pock_rail_horizontal = (
            virtual_tr_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS] -
            virtual_tr_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
            virtual_tr_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] -
            virtual_tr_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
            (virtual_tr_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS] *
             virtual_tr_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS]) -
            (virtual_tr_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS] *
             virtual_tr_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_tr_corner_pock_rail_vertical = (
            virtual_tr_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS] -
            virtual_tr_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
            virtual_tr_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] -
            virtual_tr_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
            (virtual_tr_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS] *
             virtual_tr_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS]) -
            (virtual_tr_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS] *
             virtual_tr_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS]))
                                                    
        # Virtual Side Rail Pocket, Straight line equation
        virtual_eq_l_side_pock_rail_upper = (
            virtual_l_side_pock_rail_upper[LineSeg.START][common.Y_AXIS] -
            virtual_l_side_pock_rail_upper[LineSeg.END][common.Y_AXIS],
            virtual_l_side_pock_rail_upper[LineSeg.END][common.X_AXIS] -
            virtual_l_side_pock_rail_upper[LineSeg.START][common.X_AXIS],
            (virtual_l_side_pock_rail_upper[LineSeg.START][common.X_AXIS] *
             virtual_l_side_pock_rail_upper[LineSeg.END][common.Y_AXIS]) -
            (virtual_l_side_pock_rail_upper[LineSeg.END][common.X_AXIS] *
             virtual_l_side_pock_rail_upper[LineSeg.START][common.Y_AXIS]))

        virtual_eq_l_side_pock_rail_lower = (
            virtual_l_side_pock_rail_lower[LineSeg.START][common.Y_AXIS] -
            virtual_l_side_pock_rail_lower[LineSeg.END][common.Y_AXIS],
            virtual_l_side_pock_rail_lower[LineSeg.END][common.X_AXIS] -
            virtual_l_side_pock_rail_lower[LineSeg.START][common.X_AXIS],
            (virtual_l_side_pock_rail_lower[LineSeg.START][common.X_AXIS] *
             virtual_l_side_pock_rail_lower[LineSeg.END][common.Y_AXIS]) -
            (virtual_l_side_pock_rail_lower[LineSeg.END][common.X_AXIS] *
             virtual_l_side_pock_rail_lower[LineSeg.START][common.Y_AXIS]))
                                                    
        virtual_eq_r_side_pock_rail_upper = (
            virtual_r_side_pock_rail_upper[LineSeg.START][common.Y_AXIS] -
            virtual_r_side_pock_rail_upper[LineSeg.END][common.Y_AXIS],
            virtual_r_side_pock_rail_upper[LineSeg.END][common.X_AXIS] -
            virtual_r_side_pock_rail_upper[LineSeg.START][common.X_AXIS],
            (virtual_r_side_pock_rail_upper[LineSeg.START][common.X_AXIS] *
             virtual_r_side_pock_rail_upper[LineSeg.END][common.Y_AXIS]) -
            (virtual_r_side_pock_rail_upper[LineSeg.END][common.X_AXIS] *
             virtual_r_side_pock_rail_upper[LineSeg.START][common.Y_AXIS]))

        virtual_eq_r_side_pock_rail_lower = (
            virtual_r_side_pock_rail_lower[LineSeg.START][common.Y_AXIS] -
            virtual_r_side_pock_rail_lower[LineSeg.END][common.Y_AXIS],
            virtual_r_side_pock_rail_lower[LineSeg.END][common.X_AXIS] -
            virtual_r_side_pock_rail_lower[LineSeg.START][common.X_AXIS],
            (virtual_r_side_pock_rail_lower[LineSeg.START][common.X_AXIS] *
             virtual_r_side_pock_rail_lower[LineSeg.END][common.Y_AXIS]) -
            (virtual_r_side_pock_rail_lower[LineSeg.END][common.X_AXIS] *
             virtual_r_side_pock_rail_lower[LineSeg.START][common.Y_AXIS]))

        # Equation of Virtual Rail-Pocket Collection
        cls.virtual_eq_pock_rail_all = (virtual_eq_bl_corner_pock_rail_horizontal,
                                        virtual_eq_bl_corner_pock_rail_vertical,
                                        virtual_eq_br_corner_pock_rail_horizontal,
                                        virtual_eq_br_corner_pock_rail_vertical,
                                        virtual_eq_tl_corner_pock_rail_horizontal,
                                        virtual_eq_tl_corner_pock_rail_vertical,
                                        virtual_eq_tr_corner_pock_rail_horizontal,
                                        virtual_eq_tr_corner_pock_rail_vertical,
                                        virtual_eq_l_side_pock_rail_upper,
                                        virtual_eq_l_side_pock_rail_lower,
                                        virtual_eq_r_side_pock_rail_upper,
                                        virtual_eq_r_side_pock_rail_lower)
    
    @classmethod                                     
    def set_actual_pock_rail(cls, 
                             bl_corner_pock_rail_horizon_start, bl_corner_pock_rail_horizon_end,
                             bl_corner_pock_rail_vertical_start, bl_corner_pock_rail_vertical_end,
                             l_side_pock_rail_upper_start, l_side_pock_rail_upper_end,
                             l_side_pock_rail_lower_start, l_side_pock_rail_lower_end):
                                  
        # Actual Corner Rail Pocket tuple(0) is start-point,
        # tuple(1) is end-point The reference is bottom-left-corner-rail, the others is mirror
        actual_bl_corner_pock_rail_horizontal = (bl_corner_pock_rail_horizon_start, bl_corner_pock_rail_horizon_end)
        actual_bl_corner_pock_rail_vertical = (bl_corner_pock_rail_vertical_start, bl_corner_pock_rail_vertical_end)

        actual_br_corner_pock_rail_horizontal = (np.array(
            [-actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                 np.array(
            [-actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        actual_br_corner_pock_rail_vertical = (np.array(
            [-actual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                               np.array(
            [-actual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        actual_tl_corner_pock_rail_horizontal = (np.array(
            [actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             -actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                 np.array(
            [actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             -actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        actual_tl_corner_pock_rail_vertical = (np.array(
            [actual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             -actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                               np.array(
            [actual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             -actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        actual_tr_corner_pock_rail_horizontal = (np.array(
            [-actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.X_AXIS],
             -actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.START][common.Z_AXIS]]),
                                                 np.array(
            [-actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.X_AXIS],
             -actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_horizontal[LineSeg.END][common.Z_AXIS]]))
        actual_tr_corner_pock_rail_vertical = (np.array(
            [-actual_bl_corner_pock_rail_vertical[LineSeg.START][common.X_AXIS],
             -actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.START][common.Z_AXIS]]),
                                               np.array(
            [-actual_bl_corner_pock_rail_vertical[LineSeg.END][common.X_AXIS],
             -actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Y_AXIS],
             actual_bl_corner_pock_rail_vertical[LineSeg.END][common.Z_AXIS]]))

        # Actual Side Rail Pocket tuple(0) is start-point,
        # tuple(1) is end-point The reference is left-side-rail, another is mirror
        actual_l_side_pock_rail_upper = (l_side_pock_rail_upper_start, l_side_pock_rail_upper_end)
        actual_l_side_pock_rail_lower = (l_side_pock_rail_lower_start, l_side_pock_rail_lower_end)

        actual_r_side_pock_rail_upper = (np.array([-actual_l_side_pock_rail_upper[LineSeg.START][common.X_AXIS],
                                                   actual_l_side_pock_rail_upper[LineSeg.START][common.Y_AXIS],
                                                   actual_l_side_pock_rail_upper[LineSeg.START][common.Z_AXIS]]),
                                         np.array([-actual_l_side_pock_rail_upper[LineSeg.END][common.X_AXIS],
                                                   actual_l_side_pock_rail_upper[LineSeg.END][common.Y_AXIS],
                                                   actual_l_side_pock_rail_upper[LineSeg.END][common.Z_AXIS]]))
        actual_r_side_pock_rail_lower = (np.array([-actual_l_side_pock_rail_lower[LineSeg.START][common.X_AXIS],
                                                   actual_l_side_pock_rail_lower[LineSeg.START][common.Y_AXIS],
                                                   actual_l_side_pock_rail_lower[LineSeg.START][common.Z_AXIS]]),
                                         np.array([-actual_l_side_pock_rail_lower[LineSeg.END][common.X_AXIS],
                                                   actual_l_side_pock_rail_lower[LineSeg.END][common.Y_AXIS],
                                                   actual_l_side_pock_rail_lower[LineSeg.END][common.Z_AXIS]]))
        
        # Checking circle and end-point collision prediction
        cls.actual_end_point_pock_rail_all = (actual_bl_corner_pock_rail_horizontal[LineSeg.END],
                                              actual_bl_corner_pock_rail_vertical[LineSeg.END],
                                              actual_br_corner_pock_rail_horizontal[LineSeg.END],
                                              actual_br_corner_pock_rail_vertical[LineSeg.END],
                                              actual_tl_corner_pock_rail_horizontal[LineSeg.END],
                                              actual_tl_corner_pock_rail_vertical[LineSeg.END],
                                              actual_tr_corner_pock_rail_horizontal[LineSeg.END],
                                              actual_tr_corner_pock_rail_vertical[LineSeg.END],
                                              actual_l_side_pock_rail_upper[LineSeg.END],
                                              actual_l_side_pock_rail_lower[LineSeg.END],
                                              actual_r_side_pock_rail_upper[LineSeg.END],
                                              actual_r_side_pock_rail_lower[LineSeg.END])
                                              
        # Checking Circle and Line-Segment intersection
        cls.actual_pock_rail_all = (actual_bl_corner_pock_rail_horizontal, actual_bl_corner_pock_rail_vertical,
                                    actual_br_corner_pock_rail_horizontal, actual_br_corner_pock_rail_vertical,
                                    actual_tl_corner_pock_rail_horizontal, actual_tl_corner_pock_rail_vertical,
                                    actual_tr_corner_pock_rail_horizontal, actual_tr_corner_pock_rail_vertical,
                                    actual_l_side_pock_rail_upper, actual_l_side_pock_rail_lower,
                                    actual_r_side_pock_rail_upper, actual_r_side_pock_rail_lower)
                                         
    @classmethod
    def set_actual_pock_center(cls, bl_corner_pock_center, l_side_pock_center):
        # Real Pocket-Center Position
        real_bl_corner_pock_center = bl_corner_pock_center
        real_br_corner_pock_center = np.array([-real_bl_corner_pock_center[common.X_AXIS],
                                               real_bl_corner_pock_center[common.Y_AXIS],
                                               real_bl_corner_pock_center[common.Z_AXIS]])
        real_tl_corner_pock_center = np.array([real_bl_corner_pock_center[common.X_AXIS],
                                               -real_bl_corner_pock_center[common.Y_AXIS],
                                               real_bl_corner_pock_center[common.Z_AXIS]])
        real_tr_corner_pock_center = np.array([-real_bl_corner_pock_center[common.X_AXIS],
                                               -real_bl_corner_pock_center[common.Y_AXIS],
                                               real_bl_corner_pock_center[common.Z_AXIS]])

        real_l_side_pock_center = l_side_pock_center
        real_r_side_pock_center = np.array([-real_l_side_pock_center[common.X_AXIS],
                                            real_l_side_pock_center[common.Y_AXIS],
                                            real_l_side_pock_center[common.Z_AXIS]])

        # Checking Pocket Collision
        cls.actual_pock_center_all = (real_bl_corner_pock_center, real_br_corner_pock_center,
                                      real_tl_corner_pock_center, real_tr_corner_pock_center,
                                      real_l_side_pock_center, real_r_side_pock_center)
                                           
    @classmethod
    def set_rail_border(cls, border_y_upper, border_y_lower, border_x):
        cls.rail_border_y_posit_upper = border_y_upper
        cls.rail_border_y_posit_lower = border_y_lower
        cls.rail_border_x_posit = border_x
        
    @classmethod
    def set_rectan_xy_rail(cls):
        cls.left_rail = -cls.table_width/2.0
        cls.right_rail = cls.table_width/2.0
        cls.top_rail = cls.table_length/2.0
        cls.bot_rail = -cls.table_length/2.0
        
        cls.left_rail_r = cls.left_rail + cls.r
        cls.right_rail_r = cls.right_rail - cls.r
        cls.top_rail_r = cls.top_rail - cls.r
        cls.bot_rail_r = cls.bot_rail + cls.r
        
        cls.left_rail_r_epsil = cls.left_rail + cls.r_epsil
        cls.right_rail_r_epsil = cls.right_rail - cls.r_epsil
        cls.top_rail_r_epsil = cls.top_rail - cls.r_epsil
        cls.bot_rail_r_epsil = cls.bot_rail + cls.r_epsil

        cls.left_rail_r_predict_outer = cls.left_rail_r - common.POS_EPSIL_PREDICT
        cls.right_rail_r_predict_outer = cls.right_rail_r + common.POS_EPSIL_PREDICT
        cls.top_rail_r_predict_outer = cls.top_rail_r + common.POS_EPSIL_PREDICT
        cls.bot_rail_r_predict_outer = cls.bot_rail_r - common.POS_EPSIL_PREDICT

        cls.left_rail_r_predict = cls.left_rail + cls.r_predict
        cls.right_rail_r_predict = cls.right_rail - cls.r_predict
        cls.top_rail_r_predict = cls.top_rail - cls.r_predict
        cls.bot_rail_r_predict = cls.bot_rail + cls.r_predict

        cls.rail_r_all = (cls.left_rail_r, cls.right_rail_r, cls.top_rail_r, cls.bot_rail_r)
        
    @classmethod
    def set_pocket_width(cls, pocket_width):
        cls.pocket_width = pocket_width
        cls.r_of_pocket_width = cls.pocket_width/2.0
        cls.r_of_pocket_width_square = cls.r_of_pocket_width * cls.r_of_pocket_width
        cls.r_of_pocket_width_epsilon = cls.r_of_pocket_width + common.POS_EPSIL
        cls.r_of_pocket_width_square_epsilon = cls.r_of_pocket_width_epsilon * cls.r_of_pocket_width_epsilon
        cls.r_of_pocket_width_predict = cls.r_of_pocket_width + common.POS_EPSIL_PREDICT
        cls.r_of_pocket_width_square_predict = cls.r_of_pocket_width_predict * cls.r_of_pocket_width_predict

    @classmethod
    def set_detail_auto(cls, table_type=TableType.POOL, table_size=TableSize.EIGHT_FT, game_type=GameType.POOL_9_BALL):
        cls.table_type = table_type
        cls.table_size = table_size
        cls.game_type = game_type
        
        if cls.game_type == GameType.POOL_8_BALL:
            cls.num_of_ball = 16  # Including cue ball
        elif cls.game_type == GameType.POOL_9_BALL:
            cls.num_of_ball = 10  # Including cue ball
        elif cls.game_type == GameType.SNOOKER:
            cls.num_of_ball = 22  # Including cue ball
        
        if cls.table_type == TableType.POOL:
            if cls.table_size == TableSize.SEVEN_FT:
                cls.table_detail = Pool8ftDetail  # *Waiting for update
            elif cls.table_size == TableSize.EIGHT_FT:
                cls.table_detail = Pool8ftDetail  # Detail of Table (class object)
            elif cls.table_size == TableSize.NINE_FT:
                cls.table_detail = Pool8ftDetail  # *Waiting for update
        elif cls.table_type == TableType.SNOOKER:
            if cls.table_size == TableSize.SEVEN_FT:
                cls.table_detail = Pool8ftDetail  # *Waiting for update
            elif cls.table_size == TableSize.EIGHT_FT:
                cls.table_detail = Pool8ftDetail  # *Waiting for update
            elif cls.table_size == TableSize.NINE_FT:
                cls.table_detail = Pool8ftDetail  # *Waiting for update
        
        cls.table_width = cls.table_detail.WIDTH  # unit is metric
        cls.table_length = cls.table_detail.LENGTH  # unit is metric
        cls.table_height = cls.table_detail.HEIGHT  # unit is metric
        cls.r = cls.table_detail.BALL_R  # unit is metric
        cls.r_epsil = cls.r + common.POS_EPSIL
        cls.r_predict = cls.r + common.POS_EPSIL_PREDICT
        
        cls.set_virtual_pock_rail(
                              cls.table_detail.VIRTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL[LineSeg.START],
                              cls.table_detail.VIRTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL[LineSeg.END],
                              cls.table_detail.VIRTUAL_BL_CORNER_POCK_RAIL_VERTICAL[LineSeg.START],
                              cls.table_detail.VIRTUAL_BL_CORNER_POCK_RAIL_VERTICAL[LineSeg.END],
                              cls.table_detail.VIRTUAL_L_SIDE_POCK_RAIL_UPPER[LineSeg.START],
                              cls.table_detail.VIRTUAL_L_SIDE_POCK_RAIL_UPPER[LineSeg.END],
                              cls.table_detail.VIRTUAL_L_SIDE_POCK_RAIL_LOWER[LineSeg.START],
                              cls.table_detail.VIRTUAL_L_SIDE_POCK_RAIL_LOWER[LineSeg.END])
                              
        cls.set_actual_pock_rail(
                              cls.table_detail.ACTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL[LineSeg.START],
                              cls.table_detail.ACTUAL_BL_CORNER_POCK_RAIL_HORIZONTAL[LineSeg.END],
                              cls.table_detail.ACTUAL_BL_CORNER_POCK_RAIL_VERTICAL[LineSeg.START],
                              cls.table_detail.ACTUAL_BL_CORNER_POCK_RAIL_VERTICAL[LineSeg.END],
                              cls.table_detail.ACTUAL_L_SIDE_POCK_RAIL_UPPER[LineSeg.START],
                              cls.table_detail.ACTUAL_L_SIDE_POCK_RAIL_UPPER[LineSeg.END],
                              cls.table_detail.ACTUAL_L_SIDE_POCK_RAIL_LOWER[LineSeg.START],
                              cls.table_detail.ACTUAL_L_SIDE_POCK_RAIL_LOWER[LineSeg.END])
            
        cls.set_actual_pock_center(cls.table_detail.ACTUAL_BL_CORNER_POCK_CENTER,
                                   cls.table_detail.ACTUAL_L_SIDE_POCK_CENTER)
                              
        cls.set_rail_border(cls.table_detail.RAIL_BORDER_Y_POSIT_UPPER,
                            cls.table_detail.RAIL_BORDER_Y_POSIT_LOWER,
                            cls.table_detail.RAIL_BORDER_X_POSIT)
                        
        cls.set_rectan_xy_rail() 
        
        cls.set_pocket_width(cls.table_detail.POCKET_WIDTH)
