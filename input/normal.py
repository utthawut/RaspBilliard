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

import input.base as base


class GameInput(base.InputBase):

    def __init__(self, keyboard_obj, mouse_obj=None):
        super(GameInput, self).__init__(base.InputMode.NORMAL)

        self.my_key = keyboard_obj
        self.key = -1

        self.my_mouse = mouse_obj   # optional
        if self.my_mouse:
            self.mx, self.my = self.my_mouse.position()
            self.omx, self.omy = self.mx, self.my

    def get_mouse(self):
        if self.my_mouse:
            self.mx, self.my = self.my_mouse.position()
            return True

        return False

    def get_key(self):
        self.key = self.my_key.read()
        if self.key > -1:
            return True

        return False

    def get_v_cue(self):
        if self.key == 107:     # key 'K'
            return self.v_cue + 1
        elif self.key == 108:    # key 'L'
            return self.v_cue - 1

    def get_cue_angle(self):
        if self.key == 117:      # key 'U'
            return self.cue_angle + 1
        elif self.key == 106:      # key 'J'
            return self.cue_angle - 1

    def get_tb_spin(self):
        if self.key == 259 or self.key == 134:      # key up
            return self.tb_spin + 2
        elif self.key == 258 or self.key == 135:    # key down
            return self.tb_spin - 2

    def get_lr_spin(self):
        if self.key == 260 or self.key == 136:      # key left
            return self.lr_spin + 2
        elif self.key == 261 or self.key == 137:    # key right
            return self.lr_spin - 2

    def get_cam_rotate(self):
        if self.key == 100:         # key 'D' to move rotate right
            return self.cam_rotate + 0.5
        elif self.key == 97:        # key 'A' to move camera rotate left
            return self.cam_rotate - 0.5

    def get_cam_tilt(self):
        if self.key == 119:         # key 'W' to move camera up
            return self.cam_tilt + 1
        elif self.key == 115:       # key 'S' to move camera down
            return self.cam_tilt - 1

    def get_cam_radius(self):
        if self.key == 111:         # key 'O' to zoom out
            return self.cam_radius + 5
        elif self.key == 105:       # key 'I' to zoom in
            return self.cam_radius - 5

    def get_cam_rotate_by_mouse_or_touch(self):
        rotate_dif = self.omx - self.mx
        if rotate_dif:
            return self.cam_rotate + (rotate_dif * 0.05)

    def get_cam_tilt_by_mouse_or_touch(self):
        tilt_dif = self.omy - self.my
        if tilt_dif:
            return self.cam_tilt + (tilt_dif * 0.05)

    def get_star_shot(self):
        if self.key == 103:         # key 'G' for shooting
            return True

        return False

    def get_start_aim(self):
        if self.key == 102:         # key 'F' for aiming
            return True

        return False

    def get_exit(self):
        if self.key == 27:       # key ESC to terminate/back
            return True

        return False
