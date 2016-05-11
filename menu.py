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

import pi3d


class GameType:
    POOL = 0
    SNOOK = 1


class PoolType:
    EIGHT_BALL = 0
    NINE_BALL = 1


class SnookType:
    STANDARD = 0


class PoolTableSize:
    SEVEN_FT = 0
    EIGHT_FT = 1
    NINE_FT	= 2


class SnookTableSize:
    STANDARD = 0
    SEVEN_FT = 1
    NINE_FT = 2


class PlayerMode:
    SINGLE = 0
    MULTIPLAYER	= 1


class BotLevel:
    ROOKIE = 0
    GRADUATE = 1
    EXPERIENCED = 2
    VETERAN = 3


class MultiPlayer:
    TWO = 0
    THREE = 1
    FOUR = 2


class MenuState:
    GAME_TYPE = 0
    SUB_GAME_TYPE = 1
    TABLE_SIZE = 2
    PLAYER_MODE = 3
    SUB_PLAYER_MODE = 4
    FINISH = 5


class GameFixedString(object):
    background_color = (200, 140, 20, 235)

    def __init__(self, string, x_posit, y_posit, font, camera=None, font_color=(255,255,255,255),
                    font_size=24, margin=5.0, justify='C',
                        background_color=None, shader=None, f_type="SMOOTH"):
        self.font = font
        self.string = string
        self.font_color = font_color
        self.font_size = font_size
        self.margin = margin
        self.justify = justify
        self.background_color = background_color
        self.f_type = f_type
        self.camera = camera
        self.shader = shader

        self.x_posit = x_posit
        self.y_posit = y_posit

        self.fixed_string = ""
        self.generate_string()

    def generate_string(self):
        self.fixed_string = pi3d.FixedString(font=self.font, string=self.string, camera=self.camera, 
                                             color=self.font_color, font_size=self.font_size, margin=self.margin, 
                                             justify=self.justify, background_color=self.background_color, 
                                             shader=self.shader, f_type=self.f_type)

        self.fixed_string.sprite.positionX(self.x_posit)
        self.fixed_string.sprite.positionY(self.y_posit)

    def highlight(self):
        self.fixed_string = pi3d.FixedString(font=self.font, string=self.string, camera=self.camera, 
                                             color=self.font_color, font_size=self.font_size, margin=self.margin, 
                                             justify=self.justify, background_color=self.__class__.background_color, 
                                             shader=self.shader, f_type=self.f_type)

        self.fixed_string.sprite.positionX(self.x_posit)
        self.fixed_string.sprite.positionY(self.y_posit)

    def no_highlight(self):
        self.fixed_string = pi3d.FixedString(font= self.font, string=self.string, camera=self.camera,
                                             color=self.font_color, font_size=self.font_size, margin=self.margin,
                                             justify=self.justify, background_color=self.background_color,
                                             shader=self.shader, f_type=self.f_type)

        self.fixed_string.sprite.positionX(self.x_posit)
        self.fixed_string.sprite.positionY(self.y_posit)


class GameMenu(object):
    menu_struct = []

    def __init__(self):
        self.menu_column_state = MenuState.GAME_TYPE
        self.game_type = GameType.POOL
        self.pool_type = PoolType.EIGHT_BALL
        self.snook_type = SnookType.STANDARD
        self.pool_table_size = PoolTableSize.SEVEN_FT
        self.snook_table_size = SnookTableSize.STANDARD
        self.player_mode = PlayerMode.SINGLE
        self.bot_level = BotLevel.ROOKIE
        self.multiplayer = MultiPlayer.TWO

        self.game_type_pool = ""
        self.game_type_snook = ""
        self.sub_game_type_pool_8_ball = ""
        self.sub_game_type_pool_9_ball = ""
        self.sub_game_type_snook_standard = ""
        self.pool_table_size_7 = ""
        self.pool_table_size_8 = ""
        self.pool_table_size_9 = ""
        self.snook_table_size_standard = ""
        self.snook_table_size_7 = ""
        self.snook_table_size_9 = ""
        self.player_mode_single = ""
        self.player_mode_multiplayer = ""
        self.bot_rookie = ""
        self.bot_graduate = ""
        self.bot_experienced = ""
        self.bot_veteran = ""
        self.multi_2 = ""
        self.multi_3 = ""
        self.multi_4 = ""

        self.create_str_game_type()
        self.create_str_sub_game_type_pool()
        self.create_str_sub_game_type_snook()
        self.create_str_pool_table_size()
        self.create_str_snook_table_size()
        self.create_str_player_mode()
        self.create_str_bot_level()
        self.create_str_multiplayer()

        # Initial Game Type
        self.menu_struct.append((self.game_type_pool, self.game_type_snook))
        self.menu_row_state = GameType.POOL
        self.menu_struct[self.menu_column_state][self.menu_row_state].highlight()
        self.previous_menu_row_state = [0, 0, 0, 0, 0]

    def create_str_game_type(self):
        self.game_type_pool = GameFixedString(string="Pool", x_posit=-400, y_posit=50, 
                                              font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D, 
                                              shader=flatsh, f_type='SMOOTH')
        self.game_type_snook = GameFixedString(string="Snooker", x_posit=-400, y_posit=-50, 
                                               font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D, 
                                               shader=flatsh, f_type='SMOOTH')

    def create_str_sub_game_type_pool(self):
        x_posit_tmp = self.game_type_pool.x_posit + 150
        self.sub_game_type_pool_8_ball = GameFixedString(string="8-ball", x_posit=x_posit_tmp, y_posit=25,
                                                         font='media/fonts/FreeSans.ttf', font_size=20,
                                                         camera=CAMERA2D, shader=flatsh, f_type='SMOOTH')
        self.sub_game_type_pool_9_ball = GameFixedString(string="9-ball", x_posit=x_posit_tmp, y_posit=-25,
                                                         font='media/fonts/FreeSans.ttf', font_size=20,
                                                         camera=CAMERA2D, shader=flatsh, f_type='SMOOTH')

    def create_str_sub_game_type_snook(self):
        x_posit_tmp = self.game_type_pool.x_posit + 150
        self.sub_game_type_snook_standard = GameFixedString(string="Standard Snooker", x_posit=x_posit_tmp, 
                                                            y_posit=0, font='media/fonts/FreeSans.ttf', 
                                                            font_size=20, camera=CAMERA2D, shader=flatsh, 
                                                            f_type='SMOOTH')

    def create_str_pool_table_size(self):
        x_posit_tmp = self.sub_game_type_pool_8_ball.x_posit + 150
        self.pool_table_size_7 = GameFixedString(string="7-ft", x_posit=x_posit_tmp, y_posit=50,
                                                 font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                                 shader=flatsh, f_type='SMOOTH')
        self.pool_table_size_8 = GameFixedString(string="8-ft", x_posit=x_posit_tmp, y_posit=0,
                                                 font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                                 shader=flatsh, f_type='SMOOTH')
        self.pool_table_size_9 = GameFixedString(string="8-ft", x_posit=x_posit_tmp, y_posit=-50,
                                                 font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                                 shader=flatsh, f_type='SMOOTH')

    def create_str_snook_table_size(self):
        x_posit_tmp = self.sub_game_type_pool_8_ball.x_posit + 150
        self.snook_table_size_standard = GameFixedString(string="Standard Size", x_posit=x_posit_tmp,
                                                         y_posit=50, font='media/fonts/FreeSans.ttf',
                                                         font_size=20, camera=CAMERA2D,
                                                         shader=flatsh, f_type='SMOOTH')
        self.snook_table_size_7 = GameFixedString(string="7-ft", x_posit=x_posit_tmp, y_posit=0,
                                                  font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                                  shader=flatsh, f_type='SMOOTH')
        self.snook_table_size_9 = GameFixedString(string="9-ft", x_posit=x_posit_tmp, y_posit=-50,
                                                  font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                                  shader=flatsh, f_type='SMOOTH')

    def create_str_player_mode(self):
        x_posit_tmp = self.pool_table_size_7.x_posit + 150
        self.player_mode_single = GameFixedString(string="Single", x_posit=x_posit_tmp, y_posit=25,
                                                  font='media/fonts/FreeSans.ttf', font_size=20,
                                                  camera=CAMERA2D, shader=flatsh, f_type='SMOOTH')
        self.player_mode_multiplayer = GameFixedString(string="Multiplayer", x_posit=x_posit_tmp, y_posit=-25,
                                                       font='media/fonts/FreeSans.ttf', font_size=20,
                                                       camera=CAMERA2D, shader=flatsh, f_type='SMOOTH')

    def create_str_bot_level(self):
        x_posit_tmp = self.player_mode_single.x_posit + 150
        self.bot_rookie = GameFixedString(string="Rookie", x_posit=x_posit_tmp, y_posit=75,
                                          font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                          shader=flatsh, f_type='SMOOTH')
        self.bot_graduate = GameFixedString(string="Graduate", x_posit=x_posit_tmp, y_posit=25,
                                            font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                            shader=flatsh, f_type='SMOOTH')
        self.bot_experienced = GameFixedString(string="Experienced", x_posit=x_posit_tmp, y_posit=-25,
                                               font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                               shader=flatsh, f_type='SMOOTH')
        self.bot_veteran = GameFixedString(string="Veteran", x_posit=x_posit_tmp, y_posit=-75,
                                           font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                           shader=flatsh, f_type='SMOOTH')

    def create_str_multiplayer(self):
        x_posit_tmp = self.player_mode_multiplayer.x_posit + 150
        self.multi_2 = GameFixedString(string="2 players", x_posit=x_posit_tmp, y_posit=50,
                                       font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                       shader=flatsh, f_type='SMOOTH')
        self.multi_3 = GameFixedString(string="3 players", x_posit=x_posit_tmp, y_posit=0,
                                       font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                       shader=flatsh, f_type='SMOOTH')
        self.multi_4 = GameFixedString(string="4 players", x_posit=x_posit_tmp, y_posit=-50,
                                       font='media/fonts/FreeSans.ttf', font_size=20, camera=CAMERA2D,
                                       shader=flatsh, f_type='SMOOTH')

    def select(self):
        if self.menu_column_state == MenuState.FINISH:
            return

        if self.menu_column_state == MenuState.GAME_TYPE:
            self.previous_menu_row_state[MenuState.GAME_TYPE] = self.menu_row_state
            self.game_type = self.menu_row_state
            if self.game_type == GameType.POOL:
                self.menu_struct.append((self.sub_game_type_pool_8_ball, self.sub_game_type_pool_9_ball))
            elif self.game_type == GameType.SNOOK:
                self.menu_struct.append((self.sub_game_type_snook_standard,))
        elif self.menu_column_state == MenuState.SUB_GAME_TYPE:
            self.previous_menu_row_state[MenuState.SUB_GAME_TYPE] = self.menu_row_state
            if self.game_type == GameType.POOL:
                self.pool_type = self.menu_row_state
                self.menu_struct.append((self.pool_table_size_7, self.pool_table_size_8, self.pool_table_size_9))
            elif self.game_type == GameType.SNOOK:
                self.snook_type = self.menu_row_state
                self.menu_struct.append((self.snook_table_size_standard, self.snook_table_size_7,
                                         self.snook_table_size_9))
        elif self.menu_column_state == MenuState.TABLE_SIZE:
            self.previous_menu_row_state[MenuState.TABLE_SIZE] = self.menu_row_state
            if self.game_type == GameType.POOL:
                self.pool_table_size = self.menu_row_state
            elif self.game_type == GameType.SNOOK:
                self.snook_table_size = self.menu_row_state
            self.menu_struct.append((self.player_mode_single, self.player_mode_multiplayer))
        elif self.menu_column_state == MenuState.PLAYER_MODE:
            self.previous_menu_row_state[MenuState.PLAYER_MODE] = self.menu_row_state
            self.player_mode = self.menu_row_state
            if self.player_mode == PlayerMode.SINGLE:
                self.menu_struct.append((self.bot_rookie, self.bot_graduate, self.bot_experienced, self.bot_veteran))
            elif self.player_mode == PlayerMode.MULTIPLAYER:
                self.menu_struct.append((self.multi_2, self.multi_3, self.multi_4))
        elif self.menu_column_state == MenuState.SUB_PLAYER_MODE:
            self.previous_menu_row_state[MenuState.SUB_PLAYER_MODE] = self.menu_row_state
            if self.player_mode == PlayerMode.SINGLE:
                self.bot_level = self.menu_row_state
            elif self.player_mode == PlayerMode.MULTIPLAYER:
                self.multiplayer = self.menu_row_state

        self.menu_column_state += 1
        self.menu_row_state = 0
        if self.menu_column_state == MenuState.FINISH:
            # Change state to enter game play
            pass
        else:
            for str in self.menu_struct[-1]:
                str.no_highlight()
            self.menu_struct[self.menu_column_state][self.menu_row_state].highlight()

    def back(self):
        if self.menu_column_state == MenuState.GAME_TYPE or self.menu_column_state == MenuState.FINISH:
            return

        del self.menu_struct[-1]
        self.menu_column_state -= 1
        self.menu_row_state = self.previous_menu_row_state[self.menu_column_state]

    def up(self):
        if self.menu_column_state == MenuState.FINISH:
            return

        previous_menu_row_state = self.menu_row_state
        if self.menu_row_state == 0:
            self.menu_row_state = len(self.menu_struct[self.menu_column_state]) - 1
        else:
            self.menu_row_state -= 1

        self.menu_struct[self.menu_column_state][previous_menu_row_state].no_highlight()
        self.menu_struct[self.menu_column_state][self.menu_row_state].highlight()

    def down(self):
        if self.menu_column_state == MenuState.FINISH:
            return

        previous_menu_row_state = self.menu_row_state
        if self.menu_row_state == (len(self.menu_struct[self.menu_column_state]) - 1):
            self.menu_row_state = 0
        else:
            self.menu_row_state += 1

        self.menu_struct[self.menu_column_state][previous_menu_row_state].no_highlight()
        self.menu_struct[self.menu_column_state][self.menu_row_state].highlight()


if __name__ == "__main__":
    # Load display screen
    DISPLAY = pi3d.Display.create(x=200, y=200)
    flatsh = pi3d.Shader("uv_flat")
    CAMERA2D = pi3d.Camera(is_3d=False)

    game_menu = GameMenu()

    MyKeys = pi3d.Keyboard()
    while DISPLAY.loop_running():

        for column in game_menu.menu_struct:
            for row in column:
                row.fixed_string.draw()

        k = MyKeys.read()
        if k > -1:
            if k == 27:         # Esc key
                MyKeys.close()
                DISPLAY.destroy()
                break
            elif k == 119:      # up (w key)d
                game_menu.up()
            elif k == 115:      # down (s key)
                game_menu.down()
            elif k == 97:       # back (a key)
                game_menu.back()
            elif k == 100:      # Selection (d key)
                game_menu.select()
