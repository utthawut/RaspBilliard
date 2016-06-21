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

import math


class InputMode(object):
    """
    Input mode selectable
    """
    NORMAL = 0
    VIRTUAL = 1
    ANDROID = 2


class InputMax(object):
    """
    Maximum value of each parameter
    """
    CAM_ROTATE = 360
    CAM_TILT = 90
    CAM_RADIUS = 120
    V_CUE = 40
    CUE_ANGLE = 89
    SPIN = 60


class InputMin(object):
    """
    Minimum value of each parameter
    """
    CAM_TILT = 0
    CAM_RADIUS = 10
    V_CUE = 1
    CUE_ANGLE = 0
    SPIN = -60


class InputRes(object):
    """
    Resolution/Decimal Place of input
    """
    V_CUE_DP = 10
    CUE_ANGLE_DP = 100
    SPIN_DP = 100


class InputBase(object):
    """
    Define common input parameters and conditions. its subclasses should implement physical/low level reading
    """

    def __init__(self, input_mode):
        self.input_mode = input_mode

        self.cam_radius = 15
        self.cam_rotate = 0
        self.cam_tilt = 25
        self.cam_dist = None

        self.v_cue = 0
        self.cue_angle = 3

        self.tb_spin = 0
        self.lr_spin = 0

        self.cue_stick_changed = True

        self.mx = self.my = self.omx = self.omy = 0

    def update_all_input(self):
        """
        Update all input except start_shot, start_aim and exit methods should manually be checked in main)
        """
        if self.input_mode == InputMode.NORMAL:
            if self.get_mouse():
                if self.update_cam_rotate_by_mouse_or_touch():
                    self.cue_stick_changed = True
                    self.cam_dist = None
                if self.update_cam_tilt_by_mouse_or_touch():
                    self.cam_dist = None
                self.omx, self.omy = self.mx, self.my

            if self.get_key():
                # Only one key can be occurred
                if self.update_v_cue():
                    self.cue_stick_changed = True
                elif self.update_cue_angle():
                    self.cue_stick_changed = True
                elif self.update_tb_spin():
                    self.cue_stick_changed = True
                elif self.update_lr_spin():
                    self.cue_stick_changed = True
                elif self.update_cam_rotate():
                    self.cue_stick_changed = True
                    self.cam_dist = None
                elif self.update_cam_tilt():
                    self.cam_dist = None
                elif self.update_cam_radius():
                    pass
        elif self.input_mode == InputMode.VIRTUAL:
            pass    # future
        elif self.input_mode == InputMode.ANDROID:
            pass    # future
        else:
            raise ValueError("Input Mode is invalid")

    def update_camera_move(self):
        """
        Update all input except start_shot, start_aim and exit methods should manually be checked in main)
        """
        if self.input_mode == InputMode.NORMAL:
            if self.get_mouse():
                if self.update_cam_rotate_by_mouse_or_touch():
                    self.cam_dist = None
                if self.update_cam_tilt_by_mouse_or_touch():
                    self.cam_dist = None
                self.omx, self.omy = self.mx, self.my

            if self.get_key():
                # Only one key can be occurred
                if self.update_cam_rotate():
                    self.cam_dist = None
                elif self.update_cam_tilt():
                    self.cam_dist = None
                elif self.update_cam_radius():
                    pass
        elif self.input_mode == InputMode.VIRTUAL:
            pass    # future
        elif self.input_mode == InputMode.ANDROID:
            pass    # future
        else:
            raise ValueError("Input Mode is invalid")

    def get_mouse(self):
        """
        If Input Mode has mouse or touch screen available, you can implement this method in subclass
        :return: new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_key(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_v_cue(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cue_angle(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_tb_spin(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_lr_spin(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cam_rotate(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cam_tilt(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cam_radius(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_star_shot(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_start_aim(self):
        """
        This is optional because some mode is no need to use this aiming line
        And if aiming line is automatically generated by game, you can ignore this method
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_exit(self):
        """
        Low level reading
        :return new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cam_rotate_by_mouse_or_touch(self):
        """
        If Input Mode has mouse or touch screen available, you can implement this method in subclass
        :return: new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def get_cam_tilt_by_mouse_or_touch(self):
        """
        If Input Mode has mouse or touch screen available, you can implement this method in subclass
        :return: new_value or False/None (when input is not changed (Ex. no pressing key in normal mode):
        """
        raise NotImplementedError("This method should be implemented by subclasses")

    def update_v_cue(self):
        new_v_cue = self.get_v_cue()
        if new_v_cue is not None:
            if InputMin.V_CUE <= new_v_cue <= InputMax.V_CUE:
                self.v_cue = new_v_cue
                return True

        return False

    def update_cue_angle(self):
        new_cue_angle = self.get_cue_angle()
        if new_cue_angle is not None:
            if InputMin.CUE_ANGLE <= new_cue_angle <= InputMax.CUE_ANGLE:
                self.cue_angle = new_cue_angle
                return True

        return False

    def update_tb_spin(self):
        new_tb_spin = self.get_tb_spin()
        if new_tb_spin is not None:
            if InputMin.SPIN <= new_tb_spin <= InputMax.SPIN:
                collide_point = math.hypot(self.lr_spin, self.tb_spin)
                if collide_point <= InputMax.SPIN:
                    self.tb_spin = new_tb_spin
                    return True

        return False
    
    def update_lr_spin(self):
        new_lr_spin = self.get_lr_spin()
        if new_lr_spin is not None:
            if InputMin.SPIN <= new_lr_spin <= InputMax.SPIN:
                collide_point = math.hypot(self.lr_spin, self.tb_spin)
                if collide_point <= InputMax.SPIN:
                    self.lr_spin = new_lr_spin
                    return True

        return False

    def update_cam_rotate(self):
        new_cam_rotate = self.get_cam_rotate()
        if new_cam_rotate is not None:
            self.cam_rotate = new_cam_rotate % InputMax.CAM_ROTATE
            return True

        return False

    def update_cam_tilt(self):
        new_cam_tilt = self.get_cam_tilt()
        if new_cam_tilt is not None:
            if InputMin.CAM_TILT <= new_cam_tilt <= InputMax.CAM_TILT:
                self.cam_tilt = new_cam_tilt
                return True

        return False

    def update_cam_radius(self):
        new_cam_radius = self.get_cam_radius()
        if new_cam_radius is not None:
            if InputMin.CAM_RADIUS <= new_cam_radius <= InputMax.CAM_RADIUS:
                self.cam_radius = new_cam_radius
                return True

        return False

    def update_cam_rotate_by_mouse_or_touch(self):
        new_cam_rotate = self.get_cam_rotate_by_mouse_or_touch()
        if new_cam_rotate is not None:
            self.cam_rotate = new_cam_rotate % InputMax.CAM_ROTATE
            return True

        return False

    def update_cam_tilt_by_mouse_or_touch(self):
        new_cam_tilt = self.get_cam_tilt_by_mouse_or_touch()
        if new_cam_tilt is not None:
            if InputMin.CAM_TILT <= new_cam_tilt <= InputMax.CAM_TILT:
                self.cam_tilt = new_cam_tilt
            return True

        return False

    def is_start_shot(self):
        if self.get_star_shot():
            return True

        return False

    def is_start_aim(self):
        if self.get_start_aim():
            return True

        return False

    def is_exit(self):
        if self.get_exit():
            return True

        return False


