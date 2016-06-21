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

#!/usr/bin/python
from __future__ import absolute_import, division, print_function, unicode_literals

import demo  # for development allows loading of pi3d version loaded in user space (see demo.py)
import pi3d
import common
import table
import calculate
import time
import numpy as np
from input.base import InputMode, InputMax, InputMin, InputRes
import input.normal
import input.virtual
import input.android

# Select input mode here
input_mode = InputMode.NORMAL


class StartShot(object):
    WAITING = 0
    AIMING_INITIATED = 1
    AIMING_READY = 2
    SHOT_INITIATED = 3
    SHOT_READY = 4

# Initial state
start_shot = StartShot.WAITING

# Load Display screen
display = pi3d.Display.create(x=50, y=50, samples=2)

# Initial Game Type
table.BilliardTable.set_detail_auto(table_type=table.TableType.POOL, table_size=table.TableSize.EIGHT_FT,
                                    game_type=table.GameType.POOL_9_BALL)
calculate.BilliardBall.set_r(table.BilliardTable.r)
calculate.BilliardBall.set_mass(common.DEF_BALL_MASS, common.DEF_CUE_MASS)
calculate.CalConst.initial_constant()

# Create Shader
ball_shader = pi3d.Shader("uv_reflect")
table_shader = ball_shader
shadow_shader = pi3d.Shader("uv_flat")
felt_tex = pi3d.Texture("media/textures/felt.jpg")
wood_tex = pi3d.Texture("media/textures/wood.jpg")
cue_tex = pi3d.Texture("media/textures/cue_tex.png")
norm_tex = pi3d.Texture("media/textures/grasstile_n.jpg")
shine_tex = pi3d.Texture("media/textures/photosphere_small.jpg")
shadow_tex = pi3d.Texture("media/textures/shadow.png")

# Create Light
light_source = pi3d.Light(
    lightpos=(10, -(table.BilliardTable.table_height+calculate.BilliardBall.r)*common.DIM_RATIO*5.2, 1),
    lightcol=(0.9, 0.9, 0.9), lightamb=(0.3, 0.3, 0.3), is_point=False)

# Create Table
table_model = pi3d.Model(file_string='media/models/Pool_Table_8ft.obj', name='Table',
                        sx=common.DIM_RATIO, sy=common.DIM_RATIO, sz=common.DIM_RATIO, light=light_source)
#table_model.set_shader(table_shader)
#table_model.set_normal_shine(norm_tex, 500.0, shine_tex, 0.05, bump_factor=0.1)
table_model.buf[0].set_material((0.2, 0.1, 0.15))
table_model.buf[0].set_draw_details(shadow_shader, [felt_tex, norm_tex, shine_tex], 100.0, 0.8, 0.0)
table_model.buf[1].set_draw_details(table_shader, [wood_tex, norm_tex, shine_tex], 100.0, 0.05, 0.1)
table_model.buf[2].set_draw_details(table_shader, [felt_tex, norm_tex, shine_tex], 100.0, 0.0, 0.1)

# Create Trajectories
traject_list = [(i*0.1, i*0.1, i*0.1) for i in range(500)]
traject_empty_list = [(i*0, i*0, i*0) for i in range(500)]
tracksh = pi3d.Shader("mat_flat")
track = pi3d.Lines(vertices=traject_list, material=(1.0, 0.0, 1.0), z=0, line_width=1)
track.set_shader(tracksh)

# Create Balls
r_breaking = table.BilliardTable.r
r_epsil_breaking = table.BilliardTable.r_epsil
DIAMETER = r_epsil_breaking + r_epsil_breaking

for b in (("Cue_Ball", 0, 0, 10),
          ("Pool_Ball_5", 5, 0, 30),
          ("Pool_Ball_6", 6, DIAMETER, 30),
          ("Pool_Ball_4", 4, -DIAMETER, 30),
          ("Pool_Ball_3", 3, r_epsil_breaking, 30 -common.COS_30),
          ("Pool_Ball_2", 2, -r_epsil_breaking, 30 - common.COS_30),
          ("Pool_Ball_1", 1, 0, 30 - 2 * common.COS_30),
          ("Pool_Ball_8", 8, r_epsil_breaking, 30 + common.COS_30),
          ("Pool_Ball_7", 7, -r_epsil_breaking, 30 + common.COS_30),
          ("Pool_Ball_9", 9, 0, 30 + 2 * common.COS_30)):
    ball_obj = calculate.PoolBall(name=b[0], ball_index=b[1], state=calculate.STATIONARY_STATE,
                                  r=np.array([b[2], table.BilliardTable.bot_rail_r + b[3] * DIAMETER,
                                              table.BilliardTable.table_height + r_breaking]),
                                  v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR,
                                  heading_angle=0, traject_instance=False, light=light_source)
    ball_obj.ball_model.set_draw_details(ball_shader, [ball_obj.tex, norm_tex, shine_tex], 0.0, 0.1, bump_factor=0.0)
    ball_obj.shadow.set_draw_details(shadow_shader, [shadow_tex])

cue_ball = calculate.PoolBall.instances[0] # this is the only ball we ref by name later

# Create Cue Stick
cue_stick = pi3d.Triangle(corners=((-0.01, 0.0), (0.0, 0.01), (0.01, 0.0)))
cue_obj = pi3d.TCone(radiusBot=0.1, radiusTop=0.03, height=4.0, sides=24, cy=-2.0)
cue_stick.add_child(cue_obj)
cue_obj.set_material((1.0, 0.5, 0.1))
cue_obj.set_draw_details(ball_shader, [cue_tex, norm_tex, shine_tex], 50.0, 0.1, bump_factor=0.3)
cue_obj.set_alpha(0.75)

# Create background
cube = pi3d.EnvironmentCube(size=500.0, maptype='FACES')
cube.set_draw_details(shadow_shader, pi3d.loadECfiles('media/textures','skybox_hall'))

# Initial Frame Render
frame_to_render = []
render_index = 0
display.frames_per_second = common.NOR_FRAME_PER_SEC

# Create Camera
camera = pi3d.Camera.instance()
cam_enable = True

# Create input of game
my_keys = pi3d.Keyboard()
my_mouse = pi3d.Mouse(restrict=False)
my_mouse.start()
if input_mode == InputMode.NORMAL:
    my_input = input.normal.GameInput(keyboard_obj=my_keys, mouse_obj=my_mouse)
elif input_mode == InputMode.VIRTUAL:
    my_input = input.virtual.GameInput()     # future
elif input_mode == InputMode.ANDROID:
    my_input = input.android.GameInput()    # future
else:
    raise ValueError("Input Mode is invalid")

while display.loop_running():

    cue_ball_location = np.array(cue_ball.empty1.unif[0:3])
    # move Camera circularly around cue ball
    if cam_enable:
        if my_input.cam_dist is None:
            my_input.cam_dist = my_input.cam_radius
        else:
            tmp_dist = ((camera.eye - cue_ball_location) ** 2).sum() ** 0.5  # euclidean distance
            my_input.cam_dist = tmp_dist * 0.98 + my_input.cam_radius * 0.02  # tweening
        camera.relocate(my_input.cam_rotate,
                        -my_input.cam_tilt,
                        cue_ball_location,
                        [-my_input.cam_dist, -my_input.cam_dist, -my_input.cam_dist])

    # move cue stick around cue ball
    if start_shot == StartShot.WAITING:
        cue_stick.position(cue_ball_location[0], cue_ball_location[1] - 2.0, cue_ball_location[2])
        cue_stick.rotateToY(-my_input.cam_rotate)
        v_factor = 0.25 + 0.75 * my_input.v_cue / InputMax.V_CUE
        table_factor = table.BilliardTable.r / 10.0
        cue_obj.position(table_factor * -my_input.lr_spin, table_factor * my_input.tb_spin, -0.25 * v_factor)
        cue_obj.rotateToX(my_input.cue_angle * 0.4 + 105.0)
        cue_obj.rotateToY(360.0 * v_factor)
        cue_obj.set_material((0.6 - v_factor * 0.2,
                              0.4 + v_factor * 0.3,
                              0.6 - v_factor * 0.1))
        cue_stick.draw()

    table_model.draw()

    # Draw Trajectories
    if start_shot == StartShot.AIMING_READY and len(traject_list) > 0:
        track.draw()

    if start_shot == StartShot.SHOT_READY:
        sum_del_t = 0.0
        while sum_del_t < common.NOR_SAMP_PERIOD:   # and render_index < (len(frame_to_render) - 1):
            sum_del_t += 1.0 / frame_to_render[render_index]
            for i, ball_obj_traject in enumerate(calculate.PoolBall.instances_traject):
                ball_obj = calculate.PoolBall.instances[i]
                previous_r = ball_obj.r
                ball_obj.r = ball_obj_traject.r_to_render[render_index]
                # ball_obj.w_roll = ball_obj_traject.w_to_render[render_index]
                ball_obj.present_state = ball_obj_traject.state_to_render[render_index]
                ball_obj.heading_angle = ball_obj_traject.heading_angle_to_render[render_index]
                ball_obj.heading_angle_changed |= ball_obj_traject.heading_angle_changed_to_render[render_index]
                ball_obj.move_rotate(t=sum_del_t, prev_posit=previous_r)

            render_index += 1
            if render_index >= len(frame_to_render):
                start_shot = StartShot.WAITING
                break
    for ball_obj in calculate.PoolBall.instances:
        ball_obj.empty1.draw()
        ball_obj.shadow.draw()

    cube.draw()

    if start_shot != StartShot.SHOT_READY:
        # Update input if Game is not rendering (ball is not moving)
        my_input.update_all_input()
        if my_input.cue_stick_changed:
            my_input.cue_stick_changed = False
            start_shot = StartShot.WAITING
            '''
            print("English", table.BilliardTable.r * (my_input.lr_spin/InputRes.SPIN_DP),
                  "Draw/Follow", table.BilliardTable.r * (my_input.tb_spin/InputRes.SPIN_DP),
                  "Cue_Velo", my_input.v_cue,
                  "Cue_Angle", my_input.cue_angle,
                  "Heading", my_input.cam_rotate)
            '''
        if my_input.is_start_shot():
            if start_shot == StartShot.AIMING_READY:
                start_shot = StartShot.SHOT_READY
                render_index = 0
            else:
                start_shot = StartShot.SHOT_INITIATED
        elif my_input.is_start_aim():
            start_shot = StartShot.AIMING_INITIATED
        elif my_input.is_exit():
            my_keys.close()
            display.destroy()
            break
    else:
        # Update camera input during ball rendering
        my_input.update_camera_move()

    if start_shot == StartShot.SHOT_INITIATED or start_shot == StartShot.AIMING_INITIATED:
        start_time_predict = 0
        start_time_respond = 0
        start_time_over = time.time()
        traject_list = []
        del frame_to_render [:]
        cue_ball.copy_ball_to_traject()

        for ball_obj in calculate.PoolBall.instances_traject:
            if ball_obj.ball_index == 0:  # cue_ball's index = 0
                cue_ball_traject = ball_obj

        v, w = calculate.cal_cue_impact(a=table.BilliardTable.r * (my_input.lr_spin / InputRes.SPIN_DP),
                                        b=table.BilliardTable.r * (my_input.tb_spin / InputRes.SPIN_DP),
                                        theta=my_input.cue_angle,
                                        v_cue=my_input.v_cue / InputRes.V_CUE_DP)
        cue_ball_traject.init_collide_outcome(state=calculate.STATIONARY_STATE,
                                              heading_angle=(my_input.cam_rotate + 360) % 360,
                                              v=v, w=w, u=calculate.cal_relative_velo_impact(v, w),
                                              cue_stick_collide=True)
        cue_ball_traject.update_state_collide()
        # cue_ball_traject.respond_event(find_traject=True, check_event=False)
        traject_list.append((cue_ball_traject.r.real[common.X_AXIS]*common.DIM_RATIO,
                             cue_ball_traject.r.real[common.Z_AXIS]*common.DIM_RATIO,
                             cue_ball_traject.r.real[common.Y_AXIS]*common.DIM_RATIO))
        print("Start v impact***", v)
        print("Start w impact***", w)
        print("Start r***", cue_ball_traject.r)
        print("Start v***", cue_ball_traject.v)
        print("Start v0***", cue_ball_traject.v0)
        print("Start u***", cue_ball_traject.u)
        print("Start u0***", cue_ball_traject.u0)
        print("Start u0_unit***", cue_ball_traject.u0_unit)
        print("Heading Angle", cue_ball_traject.heading_angle)
        calculate.PoolBall.t_table = 0
        time_to_event = cue_ball_traject.find_time_to_collision(find_traject=True)
        normal_loop = int(time_to_event/common.NOR_SAMP_PERIOD)
        remainder_time = time_to_event%common.NOR_SAMP_PERIOD
        while True:
            # No events occurred in normal loop
            for i in range(normal_loop):
                calculate.PoolBall.t_table += common.NOR_SAMP_PERIOD
                for ball in calculate.PoolBall.instances_traject:
                    ball.advance_state(common.NOR_SAMP_PERIOD)
                    ball.r_to_render.append(ball.r)
                    # ball.w_to_render.append(ball.w_roll)
                    ball.state_to_render.append(ball.present_state)
                    ball.heading_angle_to_render.append(ball.heading_angle)
                    ball.heading_angle_changed_to_render.append(False)
                traject_list.append((cue_ball_traject.r.real[common.X_AXIS]*common.DIM_RATIO,
                                     cue_ball_traject.r.real[common.Z_AXIS]*common.DIM_RATIO,
                                     cue_ball_traject.r.real[common.Y_AXIS]*common.DIM_RATIO))
                frame_to_render.append(common.NOR_FRAME_PER_SEC)

            # Events occurred in the lasted loop
            calculate.PoolBall.t_table += remainder_time
            for ball in calculate.PoolBall.instances_traject:
                ball.advance_state(remainder_time)
                ball.r_to_render.append(ball.r)
                # ball.w_to_render.append(ball.w_roll)
                ball.heading_angle_changed = False

            tmp_time_respond = time.time()
            cue_ball_traject.respond_event(find_traject=True, check_event=True)
            start_time_respond = start_time_respond + (time.time() - tmp_time_respond)

            for ball in calculate.PoolBall.instances_traject:
                ball.state_to_render.append(ball.present_state)
                ball.heading_angle_to_render.append(ball.heading_angle)
                ball.heading_angle_changed_to_render.append(ball.heading_angle_changed)

            traject_list.append((cue_ball_traject.r.real[common.X_AXIS]*common.DIM_RATIO,
                                 cue_ball_traject.r.real[common.Z_AXIS]*common.DIM_RATIO,
                                 cue_ball_traject.r.real[common.Y_AXIS]*common.DIM_RATIO))
            if remainder_time:
                frame_to_render.append(common.NOR_FRAME_PER_SEC * (common.NOR_SAMP_PERIOD/remainder_time))
            else:
                frame_to_render.append(common.NOR_FRAME_PER_SEC)

            if all(ball_obj.present_state == calculate.STATIONARY_STATE
                   for ball_obj in calculate.PoolBall.instances_traject):
                break
            else:
                tmp_time_predict = time.time()
                time_to_event = cue_ball_traject.find_time_to_collision(find_traject=True)
                normal_loop = int(time_to_event/common.NOR_SAMP_PERIOD)
                remainder_time = time_to_event % common.NOR_SAMP_PERIOD
                start_time_predict = start_time_predict + (time.time() - tmp_time_predict)

        if start_shot == StartShot.SHOT_INITIATED:
            start_shot = StartShot.SHOT_READY
        elif start_shot == StartShot.AIMING_INITIATED:
            start_shot = StartShot.AIMING_READY
            if len(traject_list) < 500:  # only need to do this once per f press
                for i in range(500 - len(traject_list)):
                    traject_list.append(traject_list[-1])
            elif len(traject_list) >= 500:
                traject_list = traject_list[:500]
            track.buf[0].re_init(traject_list)
        render_index = 0

        if len(traject_list) <= 1:
            del traject_list[:]

        print("*****Time for response:******************", start_time_respond)
        print("*****Time for prediction:******************", start_time_predict)
        print("*****Time for calculation:******************", time.time() - start_time_over)
        # with open('temp.txt', 'w') as f:
        #     f.write('number={} list={}'.format(len(frame_to_render), frame_to_render))
