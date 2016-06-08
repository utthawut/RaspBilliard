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

from math import sin, cos, radians
import sys
sys.path.insert(1,'/home/patrick/pi3d')
import pi3d
import common
import table
import calculate
import time
import numpy as np


class StartShot(object):
    WAITING = 0
    AIMING_INITIATED = 1
    AIMING_READY = 2
    SHOT_INITIATED = 3
    SHOT_READY = 4

# Constant/Maximum value
V_CUE_DECIMAL_PLACE = 10
MAX_CAM_ROTATION_R = 360
MAX_CAM_ROTATION_L = -360
MAX_CAM_TILT = 90
MAX_V_CUE = 40

# Load display screen
DISPLAY = pi3d.Display.create(x=100, y=100, samples=2)

# Initial Game Type
table.BilliardTable.set_detail_auto(table_type=table.TableType.POOL, table_size=table.TableSize.EIGHT_FT,
                                    game_type=table.GameType.POOL_9_BALL)
calculate.BilliardBall.set_r(table.BilliardTable.r)
calculate.BilliardBall.set_mass(common.DEF_BALL_MASS, common.DEF_CUE_MASS)
calculate.CalConst.initial_constant()

# Create Shader
BallShader = pi3d.Shader("mat_reflect")
TableShader = BallShader
ShadowShader = pi3d.Shader("uv_flat")
Normtex = pi3d.Texture("media/textures/grasstile_n.jpg")
Shinetex = pi3d.Texture("media/textures/photosphere_small.jpg")
Shadowtex = pi3d.Texture("media/textures/shadow.png")

# Create Light
light_source = pi3d.Light(
    lightpos=(10, -(table.BilliardTable.table_height+calculate.BilliardBall.r)*common.DIM_RATIO*5.2, 1),
    lightcol=(0.9, 0.9, 0.8), lightamb=(0.3, 0.3, 0.3), is_point=False)

# Create Table
TableModel = pi3d.Model(file_string='media/models/Pool_Table_8ft.obj', name='Table',
                        sx = common.DIM_RATIO, sy=common.DIM_RATIO, sz=common.DIM_RATIO, light=light_source)
TableModel.set_shader(TableShader)
TableModel.set_normal_shine(Normtex, 500.0, Shinetex, 0.05, bump_factor=0.1)

# Create Trajectories
traject_list = [(i*0.1, i*0.1, i*0.1) for i in range(500)]
traject_empty_list = [(i*0, i*0, i*0) for i in range(500)]
tracksh = pi3d.Shader("mat_flat")
track = pi3d.Lines(vertices=traject_list, material=(1.0, 0.0, 1.0), z=0, line_width=1)
track.set_shader(tracksh)

# Create Ball
r_breaking = table.BilliardTable.r
r_epsil_breaking = table.BilliardTable.r_epsil
DIAMETER = r_epsil_breaking + r_epsil_breaking

cue_ball = calculate.PoolBall(name="Cue_Ball", ball_index=0, state=calculate.STATIONARY_STATE,
                                  r=np.array(
                                      [0, table.BilliardTable.bot_rail_r+DIAMETER*10,
                                       table.BilliardTable.table_height+table.BilliardTable.r]),
                                  v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                  traject_instance=False, light=light_source)
pool_ball_5 = calculate.PoolBall(name="Pool_Ball_5", ball_index=5, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [0, table.BilliardTable.bot_rail_r+DIAMETER*30,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_6 = calculate.PoolBall(name="Pool_Ball_6", ball_index=6, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] + DIAMETER, pool_ball_5.r[common.Y_AXIS],
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_4 = calculate.PoolBall(name="Pool_Ball_4", ball_index=4, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] - DIAMETER, pool_ball_5.r[common.Y_AXIS],
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_3 = calculate.PoolBall(name="Pool_Ball_3", ball_index=3, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] + r_epsil_breaking,
                                          pool_ball_5.r[common.Y_AXIS]-common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_2 = calculate.PoolBall(name="Pool_Ball_2", ball_index=2, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] - r_epsil_breaking,
                                          pool_ball_5.r[common.Y_AXIS]-common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_1 = calculate.PoolBall(name="Pool_Ball_1", ball_index=1, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS],
                                          pool_ball_3.r[common.Y_AXIS]-common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_8 = calculate.PoolBall(name="Pool_Ball_8", ball_index=8, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] + r_epsil_breaking,
                                          pool_ball_5.r[common.Y_AXIS]+common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_7 = calculate.PoolBall(name="Pool_Ball_7", ball_index=7, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS] - r_epsil_breaking,
                                          pool_ball_5.r[common.Y_AXIS]+common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)
pool_ball_9 = calculate.PoolBall(name="Pool_Ball_9", ball_index=9, state=calculate.STATIONARY_STATE,
                                     r=np.array(
                                         [pool_ball_5.r[common.X_AXIS],
                                          pool_ball_8.r[common.Y_AXIS]+common.COS_30*DIAMETER,
                                          table.BilliardTable.table_height+table.BilliardTable.r]),
                                     v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, heading_angle=0,
                                     traject_instance=False, light=light_source)

for ball_obj in calculate.PoolBall.instances:
    ball_obj.ball_model.set_shader(BallShader)
    ball_obj.ball_model.set_normal_shine(Normtex, 0.0, Shinetex, 0.1, is_uv=False, bump_factor=0.0)
    ball_obj.shadow.set_draw_details(ShadowShader, [Shadowtex])


# Initial Input of Cue Stick parameters
start_shot = StartShot.WAITING
v_cue = top_back_spin = left_right_spin = 0
cue_angle = 5
cue_stick_changed = True

# Initial Frame Render
frame_to_render = []
render_index = 0
DrawBallFirstTime = True  # Need to fix this
DISPLAY.frames_per_second = common.NOR_FRAME_PER_SEC

# Create Camera
CAMERA = pi3d.Camera.instance()
CamRadius = 45.0  # radius of camera position
CamRotation = 0.0  # rotation of camera
CamTilt = 0.0  # CamTilt of camera
CamEnable = True

# Create key presses (keyboard)
MyKeys = pi3d.Keyboard()
camera_position = None
while DISPLAY.loop_running():
    
    # move camera circularly around cue ball 
    if CamEnable:
        CAMERA.reset()
        CAMERA.rotateX(-CamTilt)
        CAMERA.rotateY(CamRotation)
        camera_target = [(cue_ball.r[common.X_AXIS] * common.DIM_RATIO) +
                         (CamRadius * sin(radians(CamRotation)) * cos(radians(CamTilt))),
                         (cue_ball.r[common.Z_AXIS] * common.DIM_RATIO) + (CamRadius * sin(radians(CamTilt))),
                         (cue_ball.r[common.Y_AXIS] * common.DIM_RATIO) -
                         (CamRadius * cos(radians(CamRotation)) * cos(radians(CamTilt)))]
        if camera_position is None:
          camera_position = camera_target
        else:
          camera_position = [camera_position[i] * 0.85 + camera_target[i] * 0.15 for i in range(3)]
        CAMERA.position(camera_position)

    TableModel.draw()
    
    # Draw Trajectories
    if start_shot == StartShot.AIMING_READY and len(traject_list) > 0:
        track.draw() 
        if len(traject_list) < 500:
            for i in range(500 - len(traject_list)):
                traject_list.append(traject_list[-1])
        elif len(traject_list) >= 500:
            traject_list = traject_list[:500]

        track.buf[0].re_init(traject_list)
    else:
        track.buf[0].re_init(traject_empty_list)
    
    if start_shot == StartShot.SHOT_READY:
        for ball_obj_traject in calculate.PoolBall.instances_traject:
            for ball_obj in calculate.PoolBall.instances:
                if ball_obj_traject.ball_index == ball_obj.ball_index:
                    previous_r = ball_obj.r
                    ball_obj.r = ball_obj_traject.r_to_render[render_index]
                    # ball_obj.w_roll = ball_obj_traject.w_to_render[render_index]
                    ball_obj.present_state = ball_obj_traject.state_to_render[render_index]
                    ball_obj.heading_angle = ball_obj_traject.heading_angle_to_render[render_index]
                    ball_obj.heading_angle_changed = ball_obj_traject.heading_angle_changed_to_render[render_index]
                    ball_obj.move_rotate_draw(t=1/frame_to_render[render_index], prev_posit=previous_r)
                    break

        DISPLAY.frames_per_second = frame_to_render[render_index]
        render_index += 1
        if render_index >= len(frame_to_render):         
            start_shot = StartShot.WAITING
    else:
        if DrawBallFirstTime:
            for ball_obj in calculate.PoolBall.instances:
                ball_obj.move_draw()
                ball_obj.shadow.draw()
            DrawBallFirstTime = True
        else:
            for ball_obj in calculate.PoolBall.instances:
                ball_obj.empty1.draw()
                ball_obj.shadow.draw()

    # Check Keyboard
    k = MyKeys.read()
    if k > -1:
        if k == 111:          # key 'O' to zoom out
            if CamRadius < 120:
                CamRadius += 5
        elif k == 105:        # key 'I' to zoom in
            if CamRadius >= 5:
                CamRadius -= 5
        elif k == 119:        # key 'W' to move camera up
            if CamTilt < MAX_CAM_TILT:
                CamTilt += 1
        elif k == 115:        # key 'S' to move camera down
            if CamTilt > -90:
                CamTilt -= 1
        elif k == 97:        # key 'A' to move camera rotate left
            if CamRotation == MAX_CAM_ROTATION_L:
                CamRotation = 1
            else:
                CamRotation -= .5
            cue_stick_changed = True
        elif k == 100:        # key 'd' to move rotate right
            if CamRotation == MAX_CAM_ROTATION_R:
                CamRotation = -1
            else:
                CamRotation += .5
            cue_stick_changed = True
        elif k == 27:        # key ESC to terminate
            MyKeys.close()
            DISPLAY.destroy()
            break
        elif k == 107:      # key 'K'
            tmp_v_cue = v_cue + 1
            if tmp_v_cue <= MAX_V_CUE:
                v_cue = tmp_v_cue
                cue_stick_changed = True
                if start_shot != StartShot.SHOT_READY:
                    start_shot = StartShot.WAITING
        elif k == 108:      # key 'L'
            tmp_v_cue = v_cue - 1
            if tmp_v_cue >= 0:
                v_cue = tmp_v_cue
                cue_stick_changed = True
                if start_shot != StartShot.SHOT_READY:
                    start_shot = StartShot.WAITING
        elif k == 117:      # key 'U'
            tmp_cue_angle = cue_angle + 1
            if tmp_cue_angle <= 90:
                cue_angle = tmp_cue_angle
                cue_stick_changed = True
            if start_shot != StartShot.SHOT_READY:
                start_shot = StartShot.WAITING
        elif k == 106:      # key 'J'
            tmp_cue_angle = cue_angle - 1
            if tmp_cue_angle >= 0:
                cue_angle = tmp_cue_angle
                cue_stick_changed = True
            if start_shot != StartShot.SHOT_READY:
                start_shot = StartShot.WAITING
        # Ball-Centric
        elif k == 134:      # key up
            tmp_top_back_spin = top_back_spin + 2
            if tmp_top_back_spin <= 60:
                if (((table.BilliardTable.r*tmp_top_back_spin/100)**2 +
                        (table.BilliardTable.r*left_right_spin/100)**2) < calculate.BilliardBall.r_square):
                    top_back_spin = tmp_top_back_spin
                    cue_stick_changed = True
                    if start_shot != StartShot.SHOT_READY:
                        start_shot = StartShot.WAITING
        elif k == 135:      # key down
            tmp_top_back_spin = top_back_spin - 2
            if tmp_top_back_spin >= -60:
                if (((table.BilliardTable.r*tmp_top_back_spin/100)**2 +
                        (table.BilliardTable.r*left_right_spin/100)**2) < calculate.BilliardBall.r_square):
                    top_back_spin = tmp_top_back_spin
                    cue_stick_changed = True
                    if start_shot != StartShot.SHOT_READY:
                        start_shot = StartShot.WAITING
        elif k == 136:      # key left
            tmp_left_right_spin = left_right_spin + 2
            if tmp_left_right_spin <= 60:
                if (((table.BilliardTable.r*top_back_spin/100)**2 +
                        (table.BilliardTable.r*tmp_left_right_spin/100)**2) < calculate.BilliardBall.r_square):
                    left_right_spin = tmp_left_right_spin
                    cue_stick_changed = True
                    if start_shot != StartShot.SHOT_READY:
                        start_shot = StartShot.WAITING
        elif k == 137:      # key right
            tmp_left_right_spin = left_right_spin - 2
            if tmp_left_right_spin >= -60:
                if (((table.BilliardTable.r*top_back_spin/100)**2 +
                        (table.BilliardTable.r*tmp_left_right_spin/100)**2) < calculate.BilliardBall.r_square):
                    left_right_spin = tmp_left_right_spin
                    cue_stick_changed = True
                    if start_shot != StartShot.SHOT_READY:
                        start_shot = StartShot.WAITING
        elif k == 102:       # key 'F' for aiming
            start_shot = StartShot.AIMING_INITIATED
        elif k == 103:       # key 'G' for shooting
            if start_shot == StartShot.AIMING_READY:
                start_shot = StartShot.SHOT_READY
                render_index = 0 
            else:
                start_shot = StartShot.SHOT_INITIATED
        
    if cue_stick_changed:
        print("English", table.BilliardTable.r*left_right_spin/100,
              "Draw/Follow", table.BilliardTable.r*top_back_spin/100,
              "Cue_Velo", v_cue,
              "Cue_Angle", cue_angle,
              "Heading", CamRotation)
        cue_stick_changed = False

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
                
        v, w = calculate.cal_cue_impact(a=table.BilliardTable.r*left_right_spin/100,
                                            b=table.BilliardTable.r*top_back_spin/100,
                                            theta=cue_angle,
                                            v_cue=v_cue/V_CUE_DECIMAL_PLACE)
        cue_ball_traject.init_collide_outcome(state=calculate.STATIONARY_STATE,
                                              heading_angle=(CamRotation+360) % 360,
                                              v=v, w=w, u=calculate.cal_relative_velo_impact(v, w),
                                              cue_stick_collide=True)
        cue_ball_traject.update_state_collide()
        # cue_ball_traject.respond_event(find_traject=True, check_event=False)
        traject_list.append((cue_ball_traject.r.real[common.X_AXIS]*common.DIM_RATIO,
                             cue_ball_traject.r.real[common.Z_AXIS]*common.DIM_RATIO,
                             cue_ball_traject.r.real[common.Y_AXIS]*common.DIM_RATIO))
        print ("Start v impact***", v)
        print ("Start w impact***", w)
        print ("Start r***", cue_ball_traject.r)
        print ("Start v***", cue_ball_traject.v)
        print ("Start v0***", cue_ball_traject.v0)
        print ("Start u***", cue_ball_traject.u)
        print ("Start u0***", cue_ball_traject.u0)
        print ("Start u0_unit***", cue_ball_traject.u0_unit)
        print ("Heading Angle", cue_ball_traject.heading_angle)
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
        render_index = 0  
        
        if len(traject_list) <= 1:
            del traject_list[:]

        print("*****Time for response:******************", start_time_respond)
        print("*****Time for prediction:******************", start_time_predict)
        print("*****Time for calculation:******************", time.time() - start_time_over)
