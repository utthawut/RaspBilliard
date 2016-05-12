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

import numpy as np
import pi3d
import cmath
import common
import table
import speedup.solve as solve

TimeSpeedUp = 0.0
G = 9.8

# Motion State
STATIONARY_STATE = 0
SLIDING_STATE = 1
ROLLING_STATE = 2
SPINNING_STATE = 3

# Sign of SPIN
POSITIVE_SIGN = 1
ZERO_SIGN = 0
NEGATIVE_SIGN = -1


class BilliardBall(object):
    @classmethod
    def set_r(cls, radius_ball):
        cls.r = radius_ball
        cls.two_r = 2*cls.r
        cls.r_epsilon = cls.r + common.POS_EPSIL
        cls.r_square = cls.r * cls.r
        cls.r_square_epsilon = cls.r * cls.r + common.POS_EPSIL
        cls.sum_r = cls.r + cls.r
        cls.sum_r_epsilon = cls.sum_r + common.POS_EPSIL
        cls.sum_r_square_epsilon = cls.sum_r_epsilon * cls.sum_r_epsilon
        cls.r_predict = cls.r + common.POS_EPSIL_PREDICT
        cls.r_square_predict = cls.r_predict * cls.r_predict
        cls.sum_r_predict = cls.sum_r + common.POS_EPSIL_PREDICT
        cls.sum_r_square_predict = cls.sum_r_predict * cls.sum_r_predict

        cls.k_vector = np.array([0, 0, 1], dtype = float)
        cls.r_k_vector = np.array([0, 0, cls.r], dtype = float)
        cls.abs_r = np.linalg.norm(cls.r_k_vector)
        cls.abs_r_square = cls.abs_r * cls.abs_r
        
        if 'ball_mass' in locals():
            cls.inertia_sphere = (2/5)*cls.ball_mass*cls.r_square
        else:
            cls.inertia_sphere = (2/5)*common.DEF_BALL_MASS*cls.r_square
        
    @classmethod
    def set_mass(cls, ball_mass, cue_mass):
        cls.ball_mass = ball_mass
        cls.cue_mass = cue_mass
        cls.inertia_sphere = (2/5)*cls.ball_mass*cls.r_square


class MotionFrict(object):
    slide_frict = 0.2
    roll_frict  = 0.016
    spin_frict  = 0.044


class BallCollideCoef(object):
    restitution_coef_ball = 0.92
    restitution_coef_cushion = 0.65


class BallFrict(object):
    friction_coef_ball = 0.04
    friction_coef_rail = 0.14


class CalConst(object):
    @classmethod
    def initial_constant(cls):
        # Sliding Motion
        cls.SEVEN_SLIDE_FRICT_G_DIV_TWO = (7/2)*MotionFrict.slide_frict*G
        cls.TWO_DIV_SEVEN_SLIDE_FRICT_G = 2/(7*MotionFrict.slide_frict*G)
        cls.ZERO_P_FIVE_SLIDE_FRICT_G = 0.5*MotionFrict.slide_frict*G
        cls.SLIDE_FRICT_G = MotionFrict.slide_frict*G

        # Rolling Motion
        cls.ZERO_P_FIVE_ROLL_FRICT_G = 0.5*MotionFrict.roll_frict*G
        cls.ROLL_FRICT_G = MotionFrict.roll_frict*G

        # Spin Motion
        cls.FIVE_SPIN_FRICT_G_DIV_TWO_R = (5*MotionFrict.spin_frict*G)/BilliardBall.two_r
        cls.TWO_R_DIV_FIVE_SPIN_FRICT_G = BilliardBall.two_r/(5*MotionFrict.spin_frict*G)

        # Predict Collision
        cls.G_DIV_TWO = G/2


# common functions
def cal_unit_vector(vector):
    magnitude = np.linalg.norm(vector)
    if magnitude > 0:
        return vector/magnitude
    else:
        return np.copy(vector)


def cal_angular_velo_vertical(w0_vertical, t):
    sign_before = check_sign(w0_vertical)
    if sign_before == POSITIVE_SIGN:
        w0_vertical_after = (w0_vertical - CalConst.FIVE_SPIN_FRICT_G_DIV_TWO_R*t)
    elif sign_before == NEGATIVE_SIGN:
        w0_vertical_after = (w0_vertical + CalConst.FIVE_SPIN_FRICT_G_DIV_TWO_R*t)
    else:
        return 0
        
    if check_sign(w0_vertical_after) != sign_before:
        return 0
    else:
        return w0_vertical_after


def counter_clockwise_vector(vector, heading_angle):
    heading_radians = np.radians(heading_angle)
    cos_data = np.cos(heading_radians)
    sin_data = np.sin(heading_radians)
    
    # if cos_data < common.POS_COSINE_EPSIL and cos_data > common.NEG_COSINE_EPSIL:
    #     cos_data = 0
    # if sin_data < common.POS_COSINE_EPSIL and sin_data > common.NEG_COSINE_EPSIL:
    #     sin_data = 0
 
    rot_matrix = np.array([[cos_data, -sin_data], [sin_data, cos_data]], dtype=float)

    return np.dot(rot_matrix, vector.real)


def clockwise_vector(vector, heading_angle):
    heading_radians = np.radians(heading_angle)
    cos_data = np.cos(heading_radians)
    sin_data = np.sin(heading_radians)
    
    # if cos_data < common.POS_COSINE_EPSIL and cos_data > common.NEG_COSINE_EPSIL:
    #     cos_data = 0
    # if sin_data < common.POS_COSINE_EPSIL and sin_data > common.NEG_COSINE_EPSIL:
    #     sin_data = 0
    
    rot_matrix = np.array([[cos_data, sin_data], [-sin_data, cos_data]], dtype=float)
    
    return np.dot(rot_matrix, vector.real)


def abs_exclude_z(vector):
    return np.linalg.norm(np.delete(vector, common.Z_AXIS))


def check_sign(value):
    if value > 0:
        return POSITIVE_SIGN
    elif value < 0:
        return NEGATIVE_SIGN
    else:
        return ZERO_SIGN


# Sliding State
def cal_cue_impact(a=0, b=0, theta=0, v_cue=1):
    radian_of_theta = np.radians(theta)
    sin_theta = np.sin(radian_of_theta)
    cos_theta = np.cos(radian_of_theta)
    cos_2theta = np.cos(np.radians(2*theta))
    
    # if sin_theta < common.POS_COSINE_EPSIL and sin_theta > common.NEG_COSINE_EPSIL:
    #     sin_theta = 0
    # if cos_theta < common.POS_COSINE_EPSIL and cos_theta > common.NEG_COSINE_EPSIL:
    #     cos_theta = 0
    # if cos_2theta < common.POS_COSINE_EPSIL and cos_2theta > common.NEG_COSINE_EPSIL:
    #     cos_2theta = 0
    
    a_square = a * a
    b_square = b * b
    a_square_plus_b_square = a_square + b_square
    if a_square_plus_b_square > BilliardBall.r_square:
        a = b = a_square = b_square = a_square_plus_b_square = 0
    
    c = np.sqrt(BilliardBall.r_square - a_square_plus_b_square)

    f = ((2*BilliardBall.ball_mass*v_cue) /
         (1 + (BilliardBall.ball_mass/BilliardBall.cue_mass) +
          ((5/(2*BilliardBall.r_square)) * (a_square + b_square*(0.5 + 0.5*cos_2theta) +
                                            (c*c*(0.5 - 0.5*cos_2theta)) - (2*b*c*cos_theta*sin_theta)))))
    vx = 0.0
    vy = -(f/BilliardBall.ball_mass)*cos_theta
    vz = -(f/BilliardBall.ball_mass)*sin_theta
    
    wx = ((-c*f*sin_theta) + (b*f*cos_theta))/BilliardBall.inertia_sphere
    wy = (a*f*sin_theta)/BilliardBall.inertia_sphere
    wz = (-a*f*cos_theta)/BilliardBall.inertia_sphere
    
    # return (np.array([vx, vy, vz], float), np.array([wx, wy, wz], float))
    # convert axis of Event Base (Paper) to game ball-center axis
    return np.array([vx, -vy, 0], dtype=float), np.array([-wx, -wy, wz], dtype=float)


def cal_relative_velo_impact(v, w):
    return v + np.cross(BilliardBall.r_k_vector, w)


def cal_relative_velo(u0, u0_unit, t):
    return u0 - ((CalConst.SEVEN_SLIDE_FRICT_G_DIV_TWO*t)*u0_unit)


def cal_time_slide_end(u0):
    return np.linalg.norm(u0)*CalConst.TWO_DIV_SEVEN_SLIDE_FRICT_G


def cal_slide_posit(r0, heading_angle, v0, u0_unit, t):
    v0 = np.delete(v0.real, common.Z_AXIS)
    u0_unit = np.delete(u0_unit.real, common.Z_AXIS)
    v0 = v0.real * t
    u0_unit *= (CalConst.ZERO_P_FIVE_SLIDE_FRICT_G*t*t)
    rb = v0 - u0_unit
    rt = counter_clockwise_vector(rb, heading_angle)
    
    return r0 + np.concatenate((rt, [0]))


def cal_slide_velo(v0, u0_unit, t):
    return v0 - ((CalConst.SLIDE_FRICT_G*t)*u0_unit)


def cal_slide_angular_velo_plane(w0_plane, u0_unit, t):
    if np.linalg.norm(w0_plane) > 0:
        sign_x_before = check_sign(w0_plane.real[common.X_AXIS])
        sign_y_before = check_sign(w0_plane.real[common.Y_AXIS])
        
        w0_plane_after = w0_plane - ((CalConst.FIVE_SPIN_FRICT_G_DIV_TWO_R*t)*np.cross(BilliardBall.k_vector, u0_unit))
        
        if check_sign(w0_plane_after.real[common.X_AXIS]) != sign_x_before:
            w0_plane_after[common.X_AXIS] = 0
        if check_sign(w0_plane_after.real[common.Y_AXIS]) != sign_y_before:
            w0_plane_after[common.Y_AXIS] = 0
        return w0_plane_after
    return np.copy(w0_plane)


# Rolling State
def cal_roll_posit(r0, heading_angle, v0, v0_unit, t):
    return r0 + np.concatenate(
        (counter_clockwise_vector(
            np.delete(
                ((v0*t) - (CalConst.ZERO_P_FIVE_ROLL_FRICT_G*t*t*v0_unit)),
                common.Z_AXIS), heading_angle), [0]))


def cal_roll_velo(v0, v0_unit, t):
    sign_x_before = check_sign(v0.real[common.X_AXIS])
    sign_y_before = check_sign(v0.real[common.Y_AXIS])
    
    v0_after = v0 - (CalConst.ROLL_FRICT_G*t*v0_unit)
    
    if check_sign(v0_after.real[common.X_AXIS]) != sign_x_before:
        v0_after[common.X_AXIS] = 0
    if check_sign(v0_after.real[common.Y_AXIS]) != sign_y_before:
        v0_after[common.Y_AXIS] = 0
    return v0_after


# def cal_roll_angular_velo_plane (v, angle):
#     # return np.linalg.norm(v)/BilliardBall.r
#     v_direction = np.concatenate((counter_clockwise_vector(np.delete(v, common.Z_AXIS), angle), [0]))
#     return np.cross(BilliardBall.r_k_vector, v_direction)/BilliardBall.abs_r_square


def cal_time_roll_end(v0):
    return abs_exclude_z(v0)/CalConst.ROLL_FRICT_G


# Spinning State
def cal_time_spin_end(w0_vertical):
    return abs(w0_vertical*CalConst.TWO_R_DIV_FIVE_SPIN_FRICT_G)


# Intersection
def is_circle_line_segment_intersect(circle_center, start_line, end_line, r):
    line_vector = end_line - start_line
    relative_vector = circle_center - start_line
    line_vector_unit = cal_unit_vector(line_vector)
    project_relative_abs = np.dot(relative_vector, line_vector_unit)
    if project_relative_abs <= 0:
        closet_point_adjust = closet_point = start_line
    elif project_relative_abs >= np.linalg.norm(line_vector):
        closet_point = end_line
        closet_point_adjust = (line_vector*0.90) + start_line
    else:
        closet_point = (line_vector_unit*project_relative_abs) + start_line
        closet_point_adjust = (line_vector_unit*project_relative_abs*0.90) + start_line
    
    # if epsilon:
    #     # r_tmp = BilliardBall.r_epsilon
    #     r_tmp = BilliardBall.r + 0.0001
    #
    # else:
    #     # r_tmp = BilliardBall.r
    #     r_tmp = BilliardBall.r + 0.00005

    if np.linalg.norm(circle_center - closet_point) <= r:
        return (np.array(
            [closet_point[common.X_AXIS],
             closet_point[common.Y_AXIS],
             BilliardBall.r], dtype=float),
                np.array(
                    [closet_point_adjust[common.X_AXIS],
                     closet_point_adjust[common.Y_AXIS],
                     table.BilliardTable.table_height+BilliardBall.r], dtype=float))
    else:
        return None, None


def is_circle_line_segment_intersect2(circle_center, start_line, end_line, epsilon=False):
    if epsilon:
        # r_square_tmp = BilliardBall.r_square_epsilon
        r_square_tmp = BilliardBall.r_square_epsilon + 0.0001
    else:
        # r_square_tmp = BilliardBall.r_square
        r_square_tmp = BilliardBall.r_square_epsilon + 0.00005
    d = end_line - start_line
    f = start_line - circle_center
    d_unit = cal_unit_vector(d)
    d_abs = np.linalg.norm(d)
    
    a = np.dot(d, d)
    b = 2*np.dot(d, f)
    c = np.dot(f, f) - r_square_tmp
    
    root = solve.cython_solve_quadratic(a, b, c)
    if 0 >= root <= 1:
        # intersect_point = start_line + roots[i]*d
        intersect_point = start_line + (root*d_abs)*d_unit
        return np.array([intersect_point[common.X_AXIS], intersect_point[common.Y_AXIS], BilliardBall.r], dtype=float)
                
    return None


# Check lying on the table/desire spot
def is_outside_table_area(position):
    if ((position[common.X_AXIS] < table.BilliardTable.left_rail_r_predict_outer) or
            (position[common.X_AXIS] > table.BilliardTable.right_rail_r_predict_outer)):
        return True
    if ((position[common.Y_AXIS] > table.BilliardTable.top_rail_r_predict_outer) or
            (position[common.Y_AXIS] < table.BilliardTable.bot_rail_r_predict_outer)):
        return True
    return False


def is_outside_pocket_area(position):
    billiard_table = table.BilliardTable
    if ((billiard_table.left_rail < position[common.X_AXIS] < billiard_table.left_rail_r_predict_outer) or
            (billiard_table.right_rail_r_predict_outer < position[common.X_AXIS] < billiard_table.right_rail)):
        if (billiard_table.rail_border_y_posit_upper[common.POSIT_LOWER] < position[common.Y_AXIS] <
                billiard_table.rail_border_y_posit_upper[common.POSIT_UPPER]):
            return True
        elif (billiard_table.rail_border_y_posit_lower[common.POSIT_LOWER] < position[common.Y_AXIS] <
                billiard_table.rail_border_y_posit_lower[common.POSIT_UPPER]):
            return True
        elif (position[common.Y_AXIS] < billiard_table.bot_rail) or (position[common.Y_AXIS] > billiard_table.top_rail):
            return True
        else:
            return False
    if ((billiard_table.top_rail_r_predict_outer < position[common.Y_AXIS] < billiard_table.top_rail) or
            (billiard_table.bot_rail < position[common.Y_AXIS] < billiard_table.bot_rail_r_predict_outer)):
        if (billiard_table.rail_border_x_posit[common.POSIT_LEFT] < position[common.X_AXIS] <
                billiard_table.rail_border_x_posit[common.POSIT_RIGHT]):
            return True
        elif ((position[common.X_AXIS] < billiard_table.left_rail) or
                (position[common.X_AXIS] > billiard_table.right_rail)):
            return True
        else:
            return False

    return True


class PoolBall(object):
    instances = []
    instances_traject = []
    table = None
    t_table = 0
    """ This class is used to create billiard ball"""
    def __init__(self, name="", ball_index=0, camera=None, light=None, state=STATIONARY_STATE, heading_angle=0,
                 r=common.ZERO_VECTOR, v=common.ZERO_VECTOR, w=common.ZERO_VECTOR, u=common.ZERO_VECTOR,
                 traject_instance=False):
    
        if traject_instance:
            PoolBall.instances_traject.append(self)  # Register traject_instance for calculating trajectories
            self.r_to_render = []
            # self.w_to_render = []
            self.state_to_render = []
            self.heading_angle_to_render = []
            self.heading_angle_changed_to_render = []
        else:
            x_render = r[common.X_AXIS]*common.DIM_RATIO
            y_render = r[common.Z_AXIS]*common.DIM_RATIO
            z_render = r[common.Y_AXIS]*common.DIM_RATIO
            
            self.ball_model = pi3d.Model(
                self, name=name, file_string='media/models/' + name + '.obj', x=0, y=0, z=0,
                sx=common.DIM_RATIO, sy=common.DIM_RATIO, sz=common.DIM_RATIO,
                light=pi3d.Light(
                    lightpos=(0, (table.BilliardTable.table_height+BilliardBall.r)*common.DIM_RATIO*1.5, 0),
                    lightcol=(0.5, 0.5, 0.5), lightamb=(0.5, 0.5, 0.5), is_point=False))

            PoolBall.instances.append(self)  # Register actual instance on the table
            
            # Create Empty for rotation
            self.empty1 = pi3d.Triangle(corners=((-0.01, 0.0), (0.0, 0.01), (0.01, 0.0)), z=z_render)
            self.empty2 = pi3d.Triangle(corners=((-0.01, 0.0), (0.0, 0.01), (0.01, 0.0)))
            self.empty1.add_child(self.empty2)
            self.empty2.add_child(self.ball_model)
            
            self.empty1.positionX(x_render)
            self.empty1.positionY(y_render)
            
        # create new instance variable only on __init__ (Simple is better than complex)
        self.name = name
        self.ball_index = ball_index
        self.previous_state = self.present_state = state
        
        self.r0 = np.copy(r)
        self.r = np.copy(self.r0)
        
        self.v0 = np.copy(v)
        self.v = np.copy(self.v0)
        
        self.w0_plane = np.copy(w)
        self.w_plane = np.copy(self.w0_plane)
        
        self.w0_vertical = self.w_vertical   = self.w0_plane[common.Z_AXIS]
        
        self.u0 = np.copy(u)
        self.u = np.copy(self.u0)
        
        self.u0_unit = cal_unit_vector(self.u0)
        self.v0_unit = cal_unit_vector(self.v0)

        # self.w_roll = np.copy(common.ZERO_VECTOR)

        self.t = 0
        self.heading_angle = heading_angle
        self.heading_angle_changed = False
        
        self.previous_r = np.copy(self.r)
        self.u_direction = np.copy(common.ZERO_VECTOR)
    
        self.previous_sign_w_vertical = self.present_sign_w_vertical = check_sign(self.w0_vertical)
        
        # Collision response (per frame)    
        self.ball_collide_remember = []
        self.allow_collide_rail_per_frame = True
        self.potential_ball_collide = False
        
        # Respond sliding (spin effect) only on cue shot
        self.allow_response_sliding = False
        
        # Event Response
        self.event_motion_change = False
        self.event_rail_collide = False
        self.event_pock_rail_collide = False
        self.event_pock_collide = False
        self.event_ball_collide = [False]*table.BilliardTable.num_of_ball

        # prediction available
        self.allow_predict_rail_collide = [True]*len(table.BilliardTable.rail_r_all)
        self.allow_predict_pock_rail_collide = [True]*len(table.BilliardTable.actual_pock_rail_all)
        self.allow_predict_pock_collide = True
        self.allow_predict_ball_collide = [True]*table.BilliardTable.num_of_ball

    def init_collide_outcome(self, state=STATIONARY_STATE, heading_angle=0, v=common.ZERO_VECTOR,
                             w=common.ZERO_VECTOR, u=common.ZERO_VECTOR, cue_stick_collide=False):
        self.previous_state = self.present_state = state
        self.r0 = np.copy(self.r)
        
        self.v0 = np.copy(v)
        self.v = np.copy(self.v0)
        
        self.w0_plane = np.copy(w)
        self.w_plane = np.copy(self.w0_plane)
        
        self.w0_vertical = self.w_vertical = self.w0_plane[common.Z_AXIS]
        
        self.u0 = np.copy(u)
        self.u = np.copy(self.u0)
        
        self.u0_unit = cal_unit_vector(self.u0)
        self.v0_unit = cal_unit_vector(self.v0)

        # self.w_roll  = np.copy(common.ZERO_VECTOR)
    
        self.t = 0
        self.heading_angle = heading_angle
        
        self.previous_r = np.copy(self.r)
        self.u_direction = np.copy(common.ZERO_VECTOR)
    
        self.previous_sign_w_vertical = self.present_sign_w_vertical = check_sign(self.w0_vertical)
        
        # Respond sliding (spin effect) only on cue shot
        if cue_stick_collide:
            self.allow_response_sliding = True
        else:
            self.allow_response_sliding = False
        
    def copy_ball_to_traject(self):
        del self.__class__.instances_traject[:]
        for ball_obj in self.__class__.instances:
            PoolBall(name=ball_obj.name, ball_index=ball_obj.ball_index, r=ball_obj.r, traject_instance=True)
    
    def move_draw(self):
        self.empty1.positionX(self.r.real[common.X_AXIS]*common.DIM_RATIO)
        self.empty1.positionZ(self.r.real[common.Y_AXIS]*common.DIM_RATIO)
        self.empty1.draw()
    
    def move_rotate_draw(self, t, prev_posit):
        self.empty1.positionX(self.r.real[common.X_AXIS]*common.DIM_RATIO)
        self.empty1.positionZ(self.r.real[common.Y_AXIS]*common.DIM_RATIO)
        
        if self.present_state == ROLLING_STATE:
            dif = self.r - prev_posit  # Current position - Previous position
            distance = np.linalg.norm(dif)  # Magnitude
            if distance:
                angle = distance/(2*np.pi*BilliardBall.r)  # Angle (radian) to be rotated
                # dif_vector_normalized = dif/distance  # Normalization

                # This rotation axis can be arbitrary axis on x-y plne for game's logic or x-z plane
                # when rendered by pi3d
                # rotation_axis = np.cross(dif_vector_normalized, common.Z_UP_VECTOR)

                # Convert radian to degree and then scale it to be rendered on pi3d
                angle = np.degrees(angle)*common.DIM_RATIO

                # self.empty1.draw()
                # self.empty1.translateX((self.r.real[common.X_AXIS] - prev_posit.real[common.X_AXIS])*common.DIM_RATIO)
                # self.empty1.translateZ((self.r.real[common.Y_AXIS] - prev_posit.real[common.Y_AXIS])*common.DIM_RATIO)
                    
                if self.heading_angle_changed:
                    self.heading_angle_changed = False
                    self.empty1.rotateToY(-self.heading_angle)
                    self.empty2.rotateIncY(self.heading_angle)  # It need to have the Euler angles worked
                    
                self.empty2.rotateIncX(angle)

        self.empty1.draw()

    def find_time_to_collision(self, find_traject=False):
        global TimeSpeedUp
        if find_traject:
            ball_instances = self.__class__.instances_traject
        else:
            ball_instances = self.__class__.instances
        
        # Initial list variable for storing time prediction
        num_of_ball = len(ball_instances)
        list_time_motion = [common.MAX_TIME]*num_of_ball
        list_time_rail_collide = list_time_motion[:]
        list_time_pock_rail_collide = list_time_motion[:]
        list_time_pock_collide = list_time_motion[:]
        list_time_ball_collide = [list_time_motion[:] for _ in range(num_of_ball)] 
            
        t_smallest = common.MAX_TIME
        # Motion-transition
        for ball in ball_instances:
            if ball.present_state != STATIONARY_STATE:
                list_time_motion[ball.ball_index] = ball.time_of_motion()
                t_smallest = min(list_time_motion[ball.ball_index], t_smallest)

        for i, ball_1 in enumerate(ball_instances):
            for ball_2 in ball_instances[0:i]:
                # Ball Collision
                if (ball_1.allow_predict_ball_collide[ball_2.ball_index] and
                        ball_2.allow_predict_ball_collide[ball_1.ball_index]):
                    # Check both balls are rest?
                    if ((ball_1.present_state != SPINNING_STATE and ball_1.present_state != STATIONARY_STATE) or
                            (ball_2.present_state != SPINNING_STATE and ball_2.present_state != STATIONARY_STATE)):
                        list_time_ball_collide[ball_1.ball_index][ball_2.ball_index] = ball_1.time_of_ball_collision(
                            ball_2, t_smallest)
                        t_smallest = min(list_time_ball_collide[ball_1.ball_index][ball_2.ball_index], t_smallest)

        for ball in ball_instances:
            # Rail, Pocket-Rail and Pocket Collision
            if ball.present_state != SPINNING_STATE and ball.present_state != STATIONARY_STATE:
                # Rail
                list_time_rail_collide[ball.ball_index] = ball.time_of_rail_collision(t_smallest)
                t_smallest = min(list_time_rail_collide[ball.ball_index], t_smallest)

                # Pocket Rail
                list_time_pock_rail_collide[ball.ball_index] = ball.time_of_pock_rail_collision(t_smallest)
                t_smallest = min(list_time_pock_rail_collide[ball.ball_index], t_smallest)

                # Pocket
                list_time_pock_collide[ball.ball_index] = ball.time_of_pock_collision(t_smallest)
                t_smallest = min(list_time_pock_collide[ball.ball_index], t_smallest)

            # Allow all prediction again after prediction completed
            # ball.allow_predict_rail_collide         = [True]*len(table.BilliardTable.rail_r_all)
            # ball.allow_predict_pock_rail_collide    = [True]*len(table.BilliardTable.actual_pock_rail_all)
            # ball.allow_predict_pock_collide         = True
            # ball.allow_predict_ball_collide         = [True]*table.BilliardTable.num_of_ball

        if t_smallest < common.MAX_EVENT_TIME:
            # Assign Motion Transition, Rail, Pocket-Rail, pocket Events
            for i, ball_1 in enumerate(ball_instances):
                if (list_time_motion[ball_1.ball_index] - t_smallest) < common.SAMP_PERIOD_EPSIL:
                    ball_1.event_motion_change = True
                    # print("Motion Transition Event", ball_1.name)
                if (list_time_rail_collide[ball_1.ball_index] - t_smallest) < common.SAMP_PERIOD_EPSIL:
                    ball_1.event_rail_collide = True
                    # print("Rail Collision Event", ball_1.name)
                if (list_time_pock_rail_collide[ball_1.ball_index] - t_smallest) < common.SAMP_PERIOD_EPSIL:
                    ball_1.event_pock_rail_collide = True
                    # print("PockRail Collision Event", ball_1.name)
                if (list_time_pock_collide[ball_1.ball_index] - t_smallest) < common.SAMP_PERIOD_EPSIL:
                    ball_1.event_pock_collide = True
                    # print("Pocket Collision Event", ball_1.name)

                # Assign Ball Collisions Event
                for ball_2 in ball_instances[0:i]:
                    if ball_1.ball_index != ball_2.ball_index:
                        if ((list_time_ball_collide[ball_1.ball_index][ball_2.ball_index] - t_smallest) <
                                common.SAMP_PERIOD_EPSIL):
                            ball_1.event_ball_collide[ball_2.ball_index] = True
                            # print("Ball Collision Event", ball_1.name, ball_2.name)
                        elif ((list_time_ball_collide[ball_2.ball_index][ball_1.ball_index] - t_smallest) <
                                common.SAMP_PERIOD_EPSIL):
                            ball_2.event_ball_collide[ball_1.ball_index] = True
                            # print("Ball Collision Event", ball_1.name, ball_2.name)

        # print (t_smallest)
        if t_smallest < common.MAX_EVENT_TIME:
            return t_smallest

        return common.NOR_SAMP_PERIOD

    def time_of_motion(self):       
        t = common.MAX_TIME
        if self.present_state == SLIDING_STATE:
            t = cal_time_slide_end(self.u0) - self.t
        elif self.present_state == ROLLING_STATE:
            t = cal_time_roll_end(self.v0) - self.t
        elif self.present_state == SPINNING_STATE:
            t = cal_time_spin_end(self.w0_vertical) - self.t
            
        if t > 0:
            return t
        else:
            return common.MAX_TIME

    def time_of_ball_collision(self, ball_2, t_smallest):
        t = t_smallest
        find_root = False
        dynamic_cal = True

        if (self.present_state == SPINNING_STATE) or (self.present_state == STATIONARY_STATE):
            ball_t_min = ball_2.t
            dynamic_cal = False
        elif (ball_2.present_state == SPINNING_STATE) or (ball_2.present_state == STATIONARY_STATE):
            ball_t_min = self.t
            dynamic_cal = False
        else:
            ball_t_min = 0

        if dynamic_cal:
            friction1, ux1_unit, uy1_unit, v1, rx1, ry1, theta1 = self.prepare_eq_for_solve_dynamic()
            friction2, ux2_unit, uy2_unit, v2, rx2, ry2, theta2 = ball_2.prepare_eq_for_solve_dynamic()
        else:
            friction1, ux1_unit, uy1_unit, v1, rx1, ry1, theta1 = self.prepare_eq_for_solve()
            friction2, ux2_unit, uy2_unit, v2, rx2, ry2, theta2 = ball_2.prepare_eq_for_solve()

        sin_theta1 = np.sin(np.radians(theta1))
        cos_theta1 = np.cos(np.radians(theta1))

        sin_theta2 = np.sin(np.radians(theta2))
        cos_theta2 = np.cos(np.radians(theta2))

        ax = (-CalConst.G_DIV_TWO)*((friction2*(ux2_unit*cos_theta2 - uy2_unit*sin_theta2)) -
                                    (friction1*(ux1_unit*cos_theta1 - uy1_unit*sin_theta1)))
        ay = (-CalConst.G_DIV_TWO)*((friction2*(ux2_unit*sin_theta2 + uy2_unit*cos_theta2)) -
                                    (friction1*(ux1_unit*sin_theta1 + uy1_unit*cos_theta1)))
        bx = -v2*sin_theta2 + v1*sin_theta1
        by = v2*cos_theta2 - v1*cos_theta1
        cx = rx2 - rx1
        cy = ry2 - ry1

        x4 = ax*ax + ay*ay
        x3 = 2*ax*bx + 2*ay*by
        x2 = bx*bx + 2*ax*cx + 2*ay*cy + by*by
        x1 = 2*bx*cx + 2*by*cy
        x0 = (cx*cx + cy*cy) - (4*BilliardBall.r_square)

        if x4:
            root = solve.cython_solve_quartic(x4, x3, x2, x1, x0)
        elif x3:
            root = solve.cython_solve_cubic(x3, x2, x1, x0)
        elif x2:
            root = solve.cython_solve_quadratic(x2, x1, x0)
        else:
            return common.MAX_TIME

        t_target = root - ball_t_min
        if t_target > 0:
            if t_target <= t:
                if dynamic_cal:
                    if self.is_real_physics_time_ball_collide_dynamic(ball_2, root):
                        t = root
                        find_root = True
                else:
                    if self.is_real_physics_time_ball_collide(ball_2, root):
                        t = root
                        find_root = True

        if find_root:
            if dynamic_cal:
                return t
            else:
                if t - ball_t_min > 0:
                    return  t - ball_t_min
                else:
                    print("something wrong, maybe the ball collision response is not performed", self.name, ball_2.name)
                    return t  # No choice (If get this, should find and fix the problem)
        else:
            return common.MAX_TIME

    def time_of_rail_collision(self, t_smallest):
        t = t_smallest
        find_root = False
        friction, ux_unit, uy_unit, v, rx, ry, theta = self.prepare_eq_for_solve()
        sin_theta = np.sin(np.radians(theta))
        cos_theta = np.cos(np.radians(theta))

        # Rail collision for horizontal
        x2 = (-friction*CalConst.G_DIV_TWO)*(ux_unit*cos_theta - uy_unit*sin_theta)
        x1 = -v*sin_theta

        # Left Rail
        x0 = rx - table.BilliardTable.left_rail_r
        root = solve.cython_solve_quadratic(x2, x1, x0)
        t_target = root - self.t
        if t_target > 0:
            if t_target <= t:
                if self.is_real_physics_time_rail_collide(root, table.RailIndex.LEFT):
                    t = root
                    find_root = True

        # Right Rail
        x0 = rx - table.BilliardTable.right_rail_r
        root = solve.cython_solve_quadratic(x2, x1, x0)
        t_target = root - self.t
        if t_target > 0:
            if t_target <= t:
                if self.is_real_physics_time_rail_collide(root, table.RailIndex.RIGHT):
                    t = root
                    find_root = True

        # Rail collision for vertical
        x2 = (-friction*G/2)*(ux_unit*sin_theta + uy_unit*cos_theta)
        x1 = v*cos_theta

        # Top Rail
        x0 = ry - table.BilliardTable.top_rail_r
        root = solve.cython_solve_quadratic(x2, x1, x0)
        t_target = root - self.t
        if t_target > 0:
            if t_target <= t:
                if self.is_real_physics_time_rail_collide(root, table.RailIndex.TOP):
                    t = root
                    find_root = True

        # Bottom Rail
        x0 = ry - table.BilliardTable.bot_rail_r
        root = solve.cython_solve_quadratic(x2, x1, x0)
        t_target = root - self.t
        if t_target > 0:
            if t_target <= t:
                if self.is_real_physics_time_rail_collide(root, table.RailIndex.BOTTOM):
                    t = root
                    find_root = True

        if find_root:
            if t-self.t > 0:
                return t - self.t
            else:
                print("something wrong, maybe the rail collision response is not performed", self.name)
                return t        # No choice (should find the problem)
        else:
            return common.MAX_TIME
        
    def time_of_pock_rail_collision(self, t_smallest):
        t = t_smallest
        find_root = False
        friction, ux_unit, uy_unit, v, rx, ry, theta = self.prepare_eq_for_solve()
        sin_theta = np.sin(np.radians(theta))
        cos_theta = np.cos(np.radians(theta))
        ux_cos_minus_uy_sin = (ux_unit*cos_theta - uy_unit*sin_theta)
        ux_sin_plus_uy_cos = (ux_unit*sin_theta + uy_unit*cos_theta)
        f_g_divide_2 = friction*CalConst.G_DIV_TWO

        for rail_index, eq in enumerate(table.BilliardTable.virtual_eq_pock_rail_all):
            x2 = f_g_divide_2*((eq[table.LineEq.A_COEF]*ux_cos_minus_uy_sin) +
                               (eq[table.LineEq.B_COEF]*ux_sin_plus_uy_cos))
            x1 = v*(eq[table.LineEq.A_COEF]*sin_theta - eq[table.LineEq.B_COEF]*cos_theta)
            x0 = -(eq[table.LineEq.A_COEF]*rx + eq[table.LineEq.B_COEF]*ry + eq[table.LineEq.C_COEF])

            root = solve.cython_solve_quadratic(x2, x1, x0)
            t_target = root - self.t
            if t_target > 0:
                if t_target <= t:
                    if self.is_real_physics_time_pock_rail_collide(root, rail_index):
                        t = root
                        find_root = True
        
        for corner_index, corner_point in enumerate(table.BilliardTable.actual_end_point_pock_rail_all):
            ax = -f_g_divide_2*ux_cos_minus_uy_sin
            ay = -f_g_divide_2*ux_sin_plus_uy_cos
            bx = -v*sin_theta
            by = v*cos_theta
            cx = rx - corner_point[common.X_AXIS]
            cy = ry - corner_point[common.Y_AXIS]

            x4 = ax*ax + ay*ay
            x3 = 2*ax*bx + 2*ay*by
            x2 = bx*bx + 2*ax*cx + 2*ay*cy + by*by
            x1 = 2*bx*cx + 2*by*cy
            x0 = (cx*cx + cy*cy) - BilliardBall.r_square

            if x4:
                root = solve.cython_solve_quartic(x4, x3, x2, x1, x0)
            elif x3:
                root = solve.cython_solve_cubic(x3, x2, x1, x0)
            elif x2:
                root = solve.cython_solve_quadratic(x2, x1, x0)

            t_target = root - self.t
            if t_target > 0:
                if t_target <= t:
                    if self.is_real_physics_time_pock_rail_point_collide(root, corner_point):
                        t = root
                        find_root = True

        if find_root:
            if t - self.t > 0:
                return t - self.t
            else:
                print("something wrong, maybe the pock-rail collision response is not performed", self.name)
                return t
        else:
            return common.MAX_TIME

    def time_of_pock_collision(self, t_smallest):
        t = t_smallest
        find_root = False
        friction, ux_unit, uy_unit, v, rx, ry, theta = self.prepare_eq_for_solve()
        sin_theta = np.sin(np.radians(theta))
        cos_theta = np.cos(np.radians(theta))
        ux_cos_minus_uy_sin = (ux_unit*cos_theta - uy_unit*sin_theta)
        ux_sin_plus_uy_cos = (ux_unit*sin_theta + uy_unit*cos_theta)
        f_g_divide_2 = friction*CalConst.G_DIV_TWO
        
        for pock_index, pock_center in enumerate(table.BilliardTable.actual_pock_center_all):
            ax = -f_g_divide_2*ux_cos_minus_uy_sin
            ay = -f_g_divide_2*ux_sin_plus_uy_cos
            bx = -v*sin_theta
            by = v*cos_theta
            cx = rx - pock_center[common.X_AXIS]
            cy = ry - pock_center[common.Y_AXIS]

            x4 = ax*ax + ay*ay
            x3 = 2*ax*bx + 2*ay*by
            x2 = bx*bx + 2*ax*cx + 2*ay*cy + by*by
            x1 = 2*bx*cx + 2*by*cy
            x0 = (cx*cx + cy*cy) - table.BilliardTable.r_of_pocket_width_square

            if x4:
                root = solve.cython_solve_quartic(x4, x3, x2, x1, x0)
            elif x3:
                root = solve.cython_solve_cubic(x3, x2, x1, x0)
            elif x2:
                root = solve.cython_solve_quadratic(x2, x1, x0)

            t_target = root - self.t
            if t_target > 0:
                if t_target <= t:
                    if self.is_real_physics_time_pock_collide(root, pock_center):
                        t = root
                        find_root = True

        if find_root:
            if t - self.t > 0:
                return t - self.t
            else:
                print("something wrong, maybe the pocket collision response is not performed", self.name)
                return t
        else:
            return common.MAX_TIME
    
    def prepare_eq_for_solve(self):
        if self.present_state == SLIDING_STATE:
            u_unit = self.u0_unit
            return (MotionFrict.slide_frict, u_unit[common.X_AXIS], u_unit[common.Y_AXIS], self.v0[common.Y_AXIS],
                    self.r0[common.X_AXIS], self.r0[common.Y_AXIS], self.heading_angle)
        elif self.present_state == ROLLING_STATE:
            v_unit = self.v0_unit
            return (MotionFrict.roll_frict, v_unit[common.X_AXIS], v_unit[common.Y_AXIS], self.v0[common.Y_AXIS],
                    self.r0[common.X_AXIS], self.r0[common.Y_AXIS], self.heading_angle)
        else:
            return 0, 0, 0, 0, self.r0[common.X_AXIS], self.r0[common.Y_AXIS], 0

    def prepare_eq_for_solve_dynamic(self):
        if self.present_state == SLIDING_STATE:
            u_unit = cal_unit_vector(self.u)
            return (MotionFrict.slide_frict, u_unit[common.X_AXIS], u_unit[common.Y_AXIS], self.v[common.Y_AXIS],
                    self.r[common.X_AXIS], self.r[common.Y_AXIS], self.heading_angle)
        elif self.present_state == ROLLING_STATE:
            v_unit = self.v0_unit
            return (MotionFrict.roll_frict, v_unit[common.X_AXIS], v_unit[common.Y_AXIS], self.v[common.Y_AXIS],
                    self.r[common.X_AXIS], self.r[common.Y_AXIS], self.heading_angle)
        else:
            return 0, 0, 0, 0, self.r[common.X_AXIS], self.r[common.Y_AXIS], 0

    def is_real_physics_time_ball_collide(self, other, t):
        # Check Life time
        position_future_1 = self.check_life_time_expire(t)
        position_future_2 = other.check_life_time_expire(t)
        if (position_future_1 is None) or (position_future_2 is None):
            return False

        # Check whether ball will be lying on the table or not
        if is_outside_table_area(position_future_1):
            if is_outside_pocket_area(position_future_1):
                return False
        if is_outside_table_area(position_future_2):
            if is_outside_pocket_area(position_future_2):
                return False

        # Check the ball collision will be definitely occurred
        collision_normal = position_future_1 - position_future_2
        distance_square = collision_normal.dot(collision_normal)

        if distance_square <= BilliardBall.sum_r_square_predict:
            collision_normal_unit = cal_unit_vector(collision_normal)
            collision_point = (position_future_1 + position_future_2)/2

            # Compute relative velocity
            r1 = collision_point - position_future_1
            r2 = collision_point - position_future_2

            incoming_velo1, incoming_angular_velo1 = self.prepare_future_incoming_collide(t)
            incoming_velo2, incoming_angular_velo2 = other.prepare_future_incoming_collide(t)

            relative_velo = ((incoming_velo1 + np.cross(incoming_angular_velo1, r1)) -
                             (incoming_velo2 + np.cross(incoming_angular_velo2, r2)))
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

            if relative_v_dot_n_unit < 0:
                return True

        return False

    def is_real_physics_time_ball_collide_dynamic(self, other, t):
        # Check Life time
        position_future_1 = self.check_life_time_expire_dynamic(t)
        position_future_2 = other.check_life_time_expire_dynamic(t)
        if (position_future_1 is None) or (position_future_2 is None):
            return False

        # Check whether ball will be lying on the table or not
        if is_outside_table_area(position_future_1):
            if is_outside_pocket_area(position_future_1):
                return False
        if is_outside_table_area(position_future_2):
            if is_outside_pocket_area(position_future_2):
                return False

        # Check the ball collision will be definitely occurred
        collision_normal = position_future_1 - position_future_2
        distance_square = collision_normal.dot(collision_normal)

        if distance_square <= BilliardBall.sum_r_square_predict:
            collision_normal_unit = cal_unit_vector(collision_normal)
            collision_point = (position_future_1 + position_future_2)/2

            # Compute relative velocity
            r1 = collision_point - position_future_1
            r2 = collision_point - position_future_2

            incoming_velo1, incoming_angular_velo1 = self.prepare_future_incoming_collide_dynamic(t)
            incoming_velo2, incoming_angular_velo2 = other.prepare_future_incoming_collide_dynamic(t)

            relative_velo = ((incoming_velo1 + np.cross(incoming_angular_velo1, r1)) -
                             (incoming_velo2 + np.cross(incoming_angular_velo2, r2)))
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

            if relative_v_dot_n_unit < 0:
                return True

        return False

    def is_real_physics_time_rail_collide(self, t, rail_index):
        # Check Life time
        bt = table.BilliardTable
        r_future = self.check_life_time_expire(t)
        if r_future is None:
            return False

        # Check whether ball will be lying on the table or not
        if is_outside_table_area(r_future):
            if is_outside_pocket_area(r_future):
                return False

        # Check the rail collision will be definitely occurred
        if rail_index == table.RailIndex.LEFT:
            if ((r_future[common.X_AXIS] < bt.left_rail_r_predict) and
                    ((bt.rail_border_y_posit_upper[common.POSIT_LOWER] < r_future[common.Y_AXIS] <
                        bt.rail_border_y_posit_upper[common.POSIT_UPPER]) or
                        (bt.rail_border_y_posit_lower[common.POSIT_LOWER] < r_future[common.Y_AXIS] <
                            bt.rail_border_y_posit_lower[common.POSIT_UPPER]))):
                    collision_point = np.array(
                        [r_future[common.X_AXIS] - BilliardBall.r, r_future[common.Y_AXIS], r_future[common.Z_AXIS]],
                        dtype=float)
                    collision_normal_unit = np.array([1, 0, 0], dtype=float)
            else:
                return False
        elif rail_index == table.RailIndex.RIGHT:
            if ((r_future[common.X_AXIS] > bt.right_rail_r_predict) and
                    ((bt.rail_border_y_posit_upper[common.POSIT_LOWER] < r_future[common.Y_AXIS] <
                        bt.rail_border_y_posit_upper[common.POSIT_UPPER]) or
                        (bt.rail_border_y_posit_lower[common.POSIT_LOWER] < r_future[common.Y_AXIS] <
                            bt.rail_border_y_posit_lower[common.POSIT_UPPER]))):
                collision_point = np.array(
                    [r_future[common.X_AXIS] + BilliardBall.r, r_future[common.Y_AXIS], r_future[common.Z_AXIS]],
                    dtype=float)
                collision_normal_unit = np.array([-1, 0, 0], dtype=float)
            else:
                return False
        elif rail_index == table.RailIndex.TOP:
            if ((r_future[common.Y_AXIS] > bt.top_rail_r_predict) and
                    (bt.rail_border_x_posit[common.POSIT_LEFT] < r_future[common.X_AXIS] <
                        bt.rail_border_x_posit[common.POSIT_RIGHT])):
                collision_point = np.array(
                    [r_future[common.X_AXIS], r_future[common.Y_AXIS] + BilliardBall.r, r_future[common.Z_AXIS]],
                    dtype=float)
                collision_normal_unit = np.array([0, -1, 0], dtype=float)
            else:
                return False
        elif rail_index == table.RailIndex.BOTTOM:
            if ((r_future[common.Y_AXIS] < bt.bot_rail_r_predict) and
                    (bt.rail_border_x_posit[common.POSIT_LEFT] < r_future[common.X_AXIS] <
                        bt.rail_border_x_posit[common.POSIT_RIGHT])):
                collision_point = np.array(
                    [r_future[common.X_AXIS], r_future[common.Y_AXIS] - BilliardBall.r, r_future[common.Z_AXIS]],
                    dtype=float)
                collision_normal_unit = np.array([0, 1, 0], dtype=float)
            else:
                return False
        else:
            return False

        r = collision_point - r_future
        incoming_velo, incoming_angular_velo = self.prepare_future_incoming_collide(t)
        relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)

        relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

        if relative_v_dot_n_unit < 0:
            return True

        return False

    def is_real_physics_time_pock_rail_collide(self, t, line_index):
        # Check Life time
        position_future = self.check_life_time_expire(t)
        if position_future is None:
            return False

        # Check whether ball will be lying on the table or not
        # if is_outside_table_area(position_future):
        #     if is_outside_pocket_area(position_future):
        #         return False

        # Check Circle and Line_Segment intersection
        collision_point, collision_point_adjust = is_circle_line_segment_intersect(
            np.array([position_future[common.X_AXIS], position_future[common.Y_AXIS]], dtype=float),
            np.array(
                [table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.START][common.X_AXIS],
                 table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.START][common.Y_AXIS]],
                dtype=float),
            np.array(
                [table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.END][common.X_AXIS],
                 table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.END][common.Y_AXIS]],
                dtype=float), BilliardBall.r_predict)

        if collision_point is None:
            return False

        collision_normal = position_future - collision_point
        collision_normal_unit = cal_unit_vector(collision_normal)

        r = collision_point - position_future
        incoming_velo, incoming_angular_velo = self.prepare_future_incoming_collide(t)
        relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)

        relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

        if relative_v_dot_n_unit < 0:
            return True

        return False

    def is_real_physics_time_pock_rail_point_collide(self, t, corner_point):
        # Check Life time
        position_future = self.check_life_time_expire(t)
        if position_future is None:
            return False

        # Check whether ball will be lying on the table or not
        # if is_outside_table_area(position_future):
        #     if is_outside_pocket_area(position_future):
        #         return False

        collision_normal = position_future - corner_point
        distance_square = collision_normal.dot(collision_normal)

        if distance_square <= BilliardBall.r_square_predict:
            collision_normal_unit = cal_unit_vector(collision_normal)
            r = corner_point - position_future
            incoming_velo, incoming_angular_velo = self.prepare_future_incoming_collide(t)
            relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

            if relative_v_dot_n_unit < 0:
                return True

        return False

    def is_real_physics_time_pock_collide(self, t, pocket_center):
        # Check Life time
        position_future = self.check_life_time_expire(t)
        if position_future is None:
            return False

        # Check whether ball will be lying on the table or not
        # if is_outside_table_area(position_future):
        #     if is_outside_pocket_area(position_future):
        #         return False

        collision_normal = position_future - pocket_center
        distance_square = collision_normal.dot(collision_normal)

        if distance_square <= table.BilliardTable.r_of_pocket_width_square_predict:
            collision_normal_unit = cal_unit_vector(collision_normal)
            r = pocket_center - position_future
            incoming_velo, incoming_angular_velo = self.prepare_future_incoming_collide(t)
            relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)

            if relative_v_dot_n_unit < 0:
                return True

        return False

    def check_life_time_expire(self, t):
        if self.present_state == SLIDING_STATE:
            if (cal_time_slide_end(self.u0) < t) or (self.t > t):
                return None
            else:
                position_future = cal_slide_posit(self.r0, self.heading_angle, self.v0, self.u0_unit, t)
        elif self.present_state == ROLLING_STATE:
            if (cal_time_roll_end(self.v0) < t) or (self.t > t):
                return None
            else:
                position_future = cal_roll_posit(self.r0, self.heading_angle, self.v0, self.v0_unit, t)
        else:
            position_future = self.r

        return position_future

    def check_life_time_expire_dynamic(self, t):
        if self.present_state == SLIDING_STATE:
            if (cal_time_slide_end(self.u0) - self.t) < t:
                return None
            else:
                position_future = cal_slide_posit(self.r0, self.heading_angle, self.v0, self.u0_unit, self.t+t)
        elif self.present_state == ROLLING_STATE:
            if (cal_time_roll_end(self.v0) - self.t) < t:
                return None
            else:
                position_future = cal_roll_posit(self.r0, self.heading_angle, self.v0, self.v0_unit, self.t+t)
        else:
            position_future = self.r

        return position_future

    def prepare_future_incoming_collide(self, t):
        if self.present_state == SLIDING_STATE:
            v_ball_future = cal_slide_velo(self.v0, self.u0_unit, t)
            w_plane_ball_future = cal_slide_angular_velo_plane(self.w0_plane, self.u0_unit, t)
            w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t)
            v_table_future = np.concatenate((counter_clockwise_vector(np.delete(v_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [0]))
            w_table_future = np.concatenate((counter_clockwise_vector(np.delete(w_plane_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [w_vertical_future]))

        elif self.present_state == ROLLING_STATE:
            v_ball_future = cal_roll_velo(self.v0, self.v0_unit, t)
            w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t)
            v_table_future = np.concatenate((counter_clockwise_vector(np.delete(v_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [0]))
            w_table_future = np.concatenate((
                np.delete(np.cross(BilliardBall.r_k_vector, v_table_future)/BilliardBall.abs_r_square, common.Z_AXIS),
                [w_vertical_future]))
        else:
            if self.present_state == SPINNING_STATE:
                w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t)
                v_table_future = common.ZERO_VECTOR
                w_table_future = np.array([0, 0, w_vertical_future], dtype=float)
            else:
                v_table_future = common.ZERO_VECTOR
                w_table_future = common.ZERO_VECTOR

        return v_table_future, w_table_future
        
    def prepare_future_incoming_collide_dynamic(self, t):
        if self.present_state == SLIDING_STATE:
            t_to_collide = self.t + t
            v_ball_future = cal_slide_velo(self.v0, self.u0_unit, t_to_collide)
            w_plane_ball_future = cal_slide_angular_velo_plane(self.w0_plane, self.u0_unit, t_to_collide)
            w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t_to_collide)
            v_table_future = np.concatenate((counter_clockwise_vector(np.delete(v_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [0]))
            w_table_future = np.concatenate((counter_clockwise_vector(np.delete(w_plane_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [w_vertical_future]))

        elif self.present_state == ROLLING_STATE:
            t_to_collide = self.t + t
            v_ball_future = cal_roll_velo(self.v0, self.v0_unit, t_to_collide)
            w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t_to_collide)
            v_table_future = np.concatenate((counter_clockwise_vector(np.delete(v_ball_future, common.Z_AXIS),
                                                                      self.heading_angle), [0]))
            w_table_future = np.concatenate((
                np.delete(np.cross(BilliardBall.r_k_vector, v_table_future)/BilliardBall.abs_r_square, common.Z_AXIS),
                [w_vertical_future]))
        else:
            if self.present_state == SPINNING_STATE:
                t_to_collide = self.t + t
                w_vertical_future = cal_angular_velo_vertical(self.w0_vertical, t_to_collide)
                v_table_future = common.ZERO_VECTOR
                w_table_future = np.array([0, 0, w_vertical_future], dtype=float)
            else:
                v_table_future = common.ZERO_VECTOR
                w_table_future = common.ZERO_VECTOR

        return v_table_future, w_table_future

    def advance_state(self, t):
        if self.present_state == STATIONARY_STATE:
            return
    
        # Update r, v, w
        self.t = self.t + t
        if self.present_state == SLIDING_STATE:
            self.v = cal_slide_velo(self.v0, self.u0_unit, self.t)
            self.w_plane = cal_slide_angular_velo_plane(self.w0_plane, self.u0_unit, self.t)
            if self.present_sign_w_vertical != ZERO_SIGN:
                self.w_vertical = cal_angular_velo_vertical(self.w0_vertical, self.t)
                self.present_sign_w_vertical = check_sign(self.w_vertical)
            self.u = cal_relative_velo(self.u0, self.u0_unit, self.t)
            self.r = cal_slide_posit(self.r0, self.heading_angle, self.v0, self.u0_unit, self.t)
            self.u_direction = self.r - self.previous_r
            self.previous_r = np.copy(self.r)

        elif self.present_state == ROLLING_STATE:
            # Update from Initial value of Rolling State, not Sliding State.
            # *****Collision event must update the initial value of r0, v0 ,w0, t
            self.v = cal_roll_velo(self.v0, self.v0_unit, self.t)
            # self.w_roll = cal_roll_angular_velo_plane(self.v, self.heading_angle)
            if self.present_sign_w_vertical != ZERO_SIGN:
                self.w_vertical = cal_angular_velo_vertical(self.w0_vertical, self.t)
                self.present_sign_w_vertical = check_sign(self.w_vertical)
            self.r = cal_roll_posit(self.r0, self.heading_angle, self.v0, self.v0_unit, self.t)
            # self.w_roll[common.Z_AXIS] = self.w_vertical
        elif self.present_state == SPINNING_STATE:
            if self.present_sign_w_vertical != ZERO_SIGN:
                self.w_vertical = cal_angular_velo_vertical(self.w0_vertical, self.t)
                self.present_sign_w_vertical = check_sign(self.w_vertical)
            # self.w_roll[common.Z_AXIS] = self.w_vertical

    def respond_event(self, find_traject=False, check_event=False):
        if check_event:  
            if find_traject:
                ball_instances = self.__class__.instances_traject
            else:
                ball_instances = self.__class__.instances
            
            num_of_active_ball = len(ball_instances)
            
            # MOTION-TRANSITION
            for i, ball in enumerate(ball_instances):
                ball.allow_collide_rail_per_frame = True
                ball.potential_ball_collide = False
                del ball.ball_collide_remember[:]
                ball.ball_collide_remember = [False] * table.BilliardTable.num_of_ball
                if ball.event_motion_change:
                    ball.event_motion_change = False
                    ball.update_state_change()
                            
            # BALL-COLLISION
            if num_of_active_ball > 1:
                
                # Response of Event Prediction
                for i, ball_1 in enumerate(ball_instances):
                    for ball_2 in ball_instances[0:i]:
                        if ball_1.event_ball_collide[ball_2.ball_index] or ball_2.event_ball_collide[ball_1.ball_index]:
                            ball_1.event_ball_collide[ball_2.ball_index] = False
                            ball_2.event_ball_collide[ball_1.ball_index] = False
                            if ball_1.ball_index != ball_2.ball_index:
                                if not(ball_1.ball_collide_remember[ball_2.ball_index] or
                                        ball_2.ball_collide_remember[ball_1.ball_index]):
                                    if ball_1.update_ball_collide(ball_2):
                                        ball_1.ball_collide_remember[ball_2.ball_index] = True
                                        ball_2.ball_collide_remember[ball_1.ball_index] = True
                                        ball_1.potential_ball_collide = True
                                        ball_2.potential_ball_collide = True
                        
            # RAIL and POCKET-RAIL COLLISION
            for ball in ball_instances:
                if ball.event_rail_collide:
                    ball.event_rail_collide = False
                    if ball.allow_collide_rail_per_frame:
                        rail_collide_index = ball.update_rail_collide()
                        if rail_collide_index is not None:
                            ball.allow_collide_rail_per_frame = False
                            
                if ball.event_pock_rail_collide:
                    ball.event_pock_rail_collide = False
                    if ball.allow_collide_rail_per_frame:
                        pock_rail_collide_index = ball.update_pock_rail_collide()
                        if pock_rail_collide_index is not None:
                            ball.allow_collide_rail_per_frame = False

            # POCKETED?            
            for ball in ball_instances: 
                if ball.event_pock_collide:
                    ball.event_pock_collide = False  
                    ball.update_pock_collide()
                
        else:
            if find_traject:
                ball_instances = PoolBall.instances_traject
            else:
                ball_instances = PoolBall.instances
            
            num_of_active_ball = len(ball_instances)
            
            # MOTION-TRANSITION
            for i, ball in enumerate(ball_instances):
                ball.allow_collide_rail_per_frame = True
                del ball.ball_collide_remember[:]
                ball.ball_collide_remember = [False] * table.BilliardTable.num_of_ball
                ball.update_state_collide()

            # BALL-COLLISION
            if num_of_active_ball > 1:
                
                # Response of Event Prediction
                for ball_1 in ball_instances:
                    for ball_2 in ball_instances:
                            if ball_1.ball_index != ball_2.ball_index:
                                if not(ball_1.ball_collide_remember[ball_2.ball_index] or
                                        ball_2.ball_collide_remember[ball_1.ball_index]):
                                    if ball_1.update_ball_collide(ball_2):
                                        ball_1.ball_collide_remember[ball_2.ball_index] = True
                                        ball_2.ball_collide_remember[ball_1.ball_index] = True
                                        ball_1.potential_ball_collide = True
                                        ball_2.potential_ball_collide = True
                                        
                # Find response for potential
                no_new_potential = True
                while True:
                    for ball_1 in ball_instances:
                        if ball_1.potential_ball_collide:
                            ball_1.potential_ball_collide = False
                            for ball_2 in ball_instances:
                                if ball_1.ball_index != ball_2.ball_index:
                                    if not(ball_1.ball_collide_remember[ball_2.ball_index] or
                                            ball_2.ball_collide_remember[ball_1.ball_index]):
                                        if ball_1.update_ball_collide(ball_2):
                                            ball_1.ball_collide_remember[ball_2.ball_index] = True
                                            ball_2.ball_collide_remember[ball_1.ball_index] = True
                                            ball_2.potential_ball_collide = True
                                            no_new_potential = False
                                            
                    if no_new_potential:
                        break
                    else:
                        no_new_potential = True 
        
            # RAIL and POCKET-RAIL COLLISION
            for ball in ball_instances:
                if ball.allow_collide_rail_per_frame:
                    if ball.update_rail_collide() or ball.update_pock_rail_collide():
                        ball.allow_collide_rail_per_frame = False
            
            # POCKETED?            
            for ball in ball_instances:        
                ball.update_pock_collide()
            
    def is_slide(self, small_epsilon=False):
        if small_epsilon:
            if (abs_exclude_z(self.v) > common.SMALL_POS_EPSIL) and (abs_exclude_z(self.u) > common.SMALL_POS_EPSIL):
                return True
            return False
        else:
            if ((abs_exclude_z(self.v) > common.POS_SLIDE_VELO_EPSIL) and
                    (abs_exclude_z(self.u) > common.POS_SLIDE_VELO_EPSIL)):
                return True
            return False
    
    def is_roll(self, small_epsilon=False):
        if small_epsilon:
            if (abs_exclude_z(self.v) > common.SMALL_POS_EPSIL) and (abs_exclude_z(self.u) <= common.SMALL_POS_EPSIL):
                return True
            return False
        else:
            if ((abs_exclude_z(self.v) > common.POS_ROLL_VELO_EPSIL) and
                    (abs_exclude_z(self.u) <= common.POS_SLIDE_VELO_EPSIL)):
                return True
            return False
    
    def is_spin(self, small_epsilon=False):
        if self.is_slide() or self.is_roll():
            return False
        if self.present_sign_w_vertical == ZERO_SIGN:
            self.w_vertical = 0
            return False
        elif self.present_sign_w_vertical == check_sign(self.w_vertical):  # Sign is still not changed
            if self.present_sign_w_vertical == POSITIVE_SIGN:
                if small_epsilon:
                    if self.w_vertical > common.SMALL_POS_EPSIL:
                        return True
                    else:
                        self.w_vertical = 0
                        return False
                else:
                    if self.w_vertical > common.POS_VELO_EPSIL:
                        return True
                    else:
                        self.w_vertical = 0
                        return False
            elif self.present_sign_w_vertical == NEGATIVE_SIGN:
                if small_epsilon:
                    if self.w_vertical < common.SMALL_NEG_EPSIL:
                        return True
                    else:
                        self.w_vertical = 0
                        return False
                else:
                    if self.w_vertical < common.NEG_VELO_EPSIL:
                        return True
                    else:
                        self.w_vertical = 0
                        return False
            else:
                self.w_vertical = 0
                return False
        else:  # w_vertical already got 0 (Sign changed)
            self.w_vertical = 0
            return False
    
    def is_stationary(self, small_epsilon = False):
        if small_epsilon:
            if ((abs_exclude_z(self.v) <= common.SMALL_POS_EPSIL) and
                    (abs_exclude_z(self.u) <= common.SMALL_POS_EPSIL) and
                    (self.is_spin(True) == False)):
                return True
            return False

        else:
            if ((abs_exclude_z(self.v) <= common.POS_VELO_EPSIL) and
                    (abs_exclude_z(self.u) <= common.POS_VELO_EPSIL) and
                    (self.is_spin() == False)):
                return True
            return False
            
    def update_state_change(self):
        if self.present_state == SLIDING_STATE:
            if self.is_roll():
                self.previous_state = self.present_state
                self.present_state = ROLLING_STATE
                self.r0 = np.copy(self.r)
                heading_angle_changed = ((np.degrees(
                    np.arctan2([self.v[common.Y_AXIS].real], [self.v[common.X_AXIS].real])[0]) + 270) % 360)
                self.heading_angle = (self.heading_angle + heading_angle_changed)%360
                # v[common.X_AXIS] will be unpredictable when curve/spin is performed
                self.v0 = np.array([0, abs_exclude_z(self.v), 0], dtype=float)
                self.v = np.copy(self.v0)
                self.v0_unit = cal_unit_vector(self.v0)
                self.w0_vertical = self.w_vertical
                self.t = 0
                self.heading_angle_changed = True
                # print("Motion Response", self.name)
            elif self.is_spin():
                self.previous_state = self.present_state
                self.present_state = SPINNING_STATE
                self.r0 = np.copy(self.r)
                # Update Transition value
                self.w0_vertical = self.w_vertical
                self.t = 0
                # print("Motion Response", self.name)
            elif self.is_stationary():
                self.previous_state = self.present_state
                self.present_state = STATIONARY_STATE
                self.r0 = np.copy(self.r)
                self.t = 0
        elif self.present_state == ROLLING_STATE:
            if self.is_spin():
                self.previous_state = self.present_state
                self.present_state = SPINNING_STATE
                self.r0 = np.copy(self.r)
                # Update Transition value
                self.w0_vertical = self.w_vertical
                self.t = 0
                # print("Motion Response", self.name)
            elif self.is_stationary():
                self.previous_state = self.present_state
                self.present_state = STATIONARY_STATE
                self.r0 = np.copy(self.r)
                self.t = 0
                # print("Motion Response", self.name)
        elif self.present_state == SPINNING_STATE:
            if self.is_stationary():
                self.previous_state = self.present_state
                self.present_state = STATIONARY_STATE
                self.r0 = np.copy(self.r)
                self.t = 0
                # print("Motion Response", self.name)
    
    def update_state_collide(self):
        if self.is_slide():
            if self.present_state != SLIDING_STATE:
                self.previous_state = self.present_state
                self.present_state = SLIDING_STATE
                # Update Transition value
                self.t = 0
        elif self.is_roll():
            if self.present_state != ROLLING_STATE:
                self.previous_state = self.present_state
                self.present_state = ROLLING_STATE
                # Update Transition value
                self.t = 0
                self.heading_angle_changed = True
        elif self.is_spin():
            if self.present_state != SPINNING_STATE:
                self.previous_state = self.present_state
                self.present_state = SPINNING_STATE
                # Update Transition value
                self.w0_vertical = self.w_vertical
                self.t = 0
        else:
            if self.present_state != STATIONARY_STATE:
                self.previous_state = self.present_state
                self.present_state = STATIONARY_STATE
                self.t = 0

    def update_ball_collide(self, other):
        collision_normal = self.r - other.r
        distance_square = collision_normal.dot(collision_normal)

        if distance_square <= BilliardBall.sum_r_square_epsilon:
            # Handle collision
            # Find penetration  # no need because It's already perform collision prediction
            # distance = np.sqrt(distance_square)
            # penetration = BilliardBall.sum_r - distance
                
            collision_normal_unit = cal_unit_vector(collision_normal)

            # collision point is average of penetration
            # collision_point = 0.5*(self.r + BilliardBall.r*collision_normal_unit) +
            # 0.5*(other.r - BilliardBall.r*collision_normal_unit)
            collision_point = (self.r + other.r)/2
        
            # if penetration > common.POS_EPSIL:
            # Push out by penetration * no need because we already predict collision
            #   self.r  = self.r + (0.5*penetration*collision_normal_unit)
            #   other.r = other.r - (0.5*penetration*collision_normal_unit)

            # Compute relative velocity
            r1 = collision_point - self.r
            r2 = collision_point - other.r

            incoming_velo1, incoming_angular_velo1 = self.prepare_incoming_ball_collide()
            incoming_velo2, incoming_angular_velo2 = other.prepare_incoming_ball_collide()

            relative_velo = ((incoming_velo1 + np.cross(incoming_angular_velo1, r1)) -
                             (incoming_velo2 + np.cross(incoming_angular_velo2, r2)))
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)
            
            if relative_v_dot_n_unit < 0:
                # Compute Impulse Factor
                denominator = ((1.0/BilliardBall.ball_mass + 1.0/BilliardBall.ball_mass) *
                               (collision_normal_unit.dot(collision_normal_unit)))
                # Compute Angular Factor
                cross1 = np.cross(r1, collision_normal_unit)/BilliardBall.inertia_sphere
                cross2 = np.cross(r2, collision_normal_unit)/BilliardBall.inertia_sphere
                sum_cross = np.cross(cross1, r1) + np.cross(cross2, r2)
                denominator = denominator + sum_cross.dot(collision_normal_unit)
                modified_velocity = relative_v_dot_n_unit/denominator
                j = -(1.0 + BallCollideCoef.restitution_coef_ball)*modified_velocity
                
                # Find Tangential vector
                collision_tangent = np.cross(np.cross(collision_normal_unit, relative_velo), collision_normal_unit)
                collision_tangent_unit = cal_unit_vector(collision_tangent)
                         
                if abs(relative_velo.dot(collision_tangent_unit)) > 0:
                    # Tangent dominated
                    # Calculate commonly used data
                    plus_impulse = ((j*collision_normal_unit) +
                                    ((BallFrict.friction_coef_ball*j)*collision_tangent_unit))
                    minus_impulse = ((-j*collision_normal_unit) +
                                     ((BallFrict.friction_coef_ball*j)*collision_tangent_unit))
                    
                    # Update linear velocity 
                    incoming_velo1 = incoming_velo1 + (plus_impulse/BilliardBall.ball_mass)
                    incoming_velo2 = incoming_velo2 + (minus_impulse/BilliardBall.ball_mass)
                    
                    # Update angular velocity
                    incoming_angular_velo1 = (incoming_angular_velo1 +
                                              (np.cross(r1, plus_impulse)/BilliardBall.inertia_sphere))
                    incoming_angular_velo2 = (incoming_angular_velo2 +
                                              (np.cross(r2, minus_impulse)/BilliardBall.inertia_sphere))
                else:
                    # No tangent
                    # Update linear velocity 
                    incoming_velo1 = incoming_velo1 + ((j*collision_normal_unit)/BilliardBall.ball_mass)
                    incoming_velo2 = incoming_velo2 - ((j*collision_normal_unit)/BilliardBall.ball_mass)
                    
                    # Update angular velocity
                    incoming_angular_velo1 = (incoming_angular_velo1 +
                                              (np.cross(r1, j*collision_normal_unit)/BilliardBall.inertia_sphere))
                    incoming_angular_velo2 = (incoming_angular_velo2 -
                                              (np.cross(r2, j*collision_normal_unit)/BilliardBall.inertia_sphere))

                # Update outcome
                self.update_outgoing_ball_collide(incoming_velo1, incoming_angular_velo1)
                other.update_outgoing_ball_collide(incoming_velo2, incoming_angular_velo2)
                return True
                
        return False
                
    def update_rail_collide(self):
        collision_index = None
        if ((self.r[common.X_AXIS] < table.BilliardTable.left_rail_r_epsil) and
                ((table.BilliardTable.rail_border_y_posit_upper[common.POSIT_LOWER] < self.r[common.Y_AXIS] <
                    table.BilliardTable.rail_border_y_posit_upper[common.POSIT_UPPER]) or
                    (table.BilliardTable.rail_border_y_posit_lower[common.POSIT_LOWER] < self.r[common.Y_AXIS] <
                        table.BilliardTable.rail_border_y_posit_lower[common.POSIT_UPPER]))):
            collision_point = np.array(
                [self.r[common.X_AXIS] - BilliardBall.r, self.r[common.Y_AXIS], self.r[common.Z_AXIS]], dtype=float)
            collision_normal_unit = np.array([1, 0, 0], dtype=float)
            collision_index = table.RailIndex.LEFT
        elif ((self.r[common.X_AXIS] > table.BilliardTable.right_rail_r_epsil) and
                ((table.BilliardTable.rail_border_y_posit_upper[common.POSIT_LOWER] < self.r[common.Y_AXIS] <
                    table.BilliardTable.rail_border_y_posit_upper[common.POSIT_UPPER]) or
                    (table.BilliardTable.rail_border_y_posit_lower[common.POSIT_LOWER] < self.r[common.Y_AXIS] <
                        table.BilliardTable.rail_border_y_posit_lower[common.POSIT_UPPER]))):
            collision_point = np.array(
                [self.r[common.X_AXIS] + BilliardBall.r, self.r[common.Y_AXIS], self.r[common.Z_AXIS]], dtype=float)
            collision_normal_unit = np.array([-1, 0, 0], dtype=float)
            collision_index = table.RailIndex.RIGHT
        elif ((self.r[common.Y_AXIS] > table.BilliardTable.top_rail_r_epsil) and
                (table.BilliardTable.rail_border_x_posit[common.POSIT_LEFT] < self.r[common.X_AXIS] <
                    table.BilliardTable.rail_border_x_posit[common.POSIT_RIGHT])):
            collision_point = np.array(
                [self.r[common.X_AXIS], self.r[common.Y_AXIS] + BilliardBall.r, self.r[common.Z_AXIS]], dtype=float)
            collision_normal_unit = np.array([0, -1, 0], dtype=float)
            collision_index = table.RailIndex.TOP
        elif ((self.r[common.Y_AXIS] < table.BilliardTable.bot_rail_r_epsil) and
                (table.BilliardTable.rail_border_x_posit[common.POSIT_LEFT] < self.r[common.X_AXIS] <
                    table.BilliardTable.rail_border_x_posit[common.POSIT_RIGHT])):
            collision_point = np.array(
                [self.r[common.X_AXIS], self.r[common.Y_AXIS] - BilliardBall.r, self.r[common.Z_AXIS]], dtype=float)
            collision_normal_unit = np.array([0, 1, 0], dtype=float)
            collision_index = table.RailIndex.BOTTOM
        else:
            return collision_index
            
        r = collision_point - self.r    
        incoming_velo, incoming_angular_velo = self.prepare_incoming_rail_collide()
        relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)

        relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)
            
        if relative_v_dot_n_unit < 0:
            # Compute Impulse
            if self.present_state == ROLLING_STATE or (not self.allow_response_sliding):
                # Ignore Angular
                relative_v_dot_n_unit_for_j = incoming_velo.dot(collision_normal_unit)
            else:
                relative_v_dot_n_unit_for_j = relative_v_dot_n_unit

            j = ((-(1.0 + BallCollideCoef.restitution_coef_cushion) * relative_v_dot_n_unit_for_j) /
                 ((1.0/BilliardBall.ball_mass) +
                  collision_normal_unit.dot(
                      np.cross(np.cross(r, collision_normal_unit)/BilliardBall.inertia_sphere, r))))
            
            # Find Tangential vector
            collision_tangent = np.cross(np.cross(collision_normal_unit, relative_velo), collision_normal_unit)

            # Reverse vector
            collision_tangent[common.X_AXIS] = -collision_tangent[common.X_AXIS]
            collision_tangent[common.Y_AXIS] = -collision_tangent[common.Y_AXIS]
            collision_tangent[common.Z_AXIS] = -collision_tangent[common.Z_AXIS]
            collision_tangent_unit = cal_unit_vector(collision_tangent)
            
            if abs(relative_velo.dot(collision_tangent_unit)) > 0:
                # Tangent dominated
                # Calculate commonly used data
                plus_impulse = (j*collision_normal_unit) + ((BallFrict.friction_coef_rail*j)*collision_tangent_unit)
                # Update linear velocity 
                incoming_velo = incoming_velo + (plus_impulse/BilliardBall.ball_mass)
                # Update angular velocity
                incoming_angular_velo = incoming_angular_velo + (np.cross(r, plus_impulse)/BilliardBall.inertia_sphere)
            else:
                # No tangent
                # Update linear velocity 
                incoming_velo = incoming_velo + ((j*collision_normal_unit)/BilliardBall.ball_mass)
                # Update angular velocity
                incoming_angular_velo = (incoming_angular_velo +
                                         (np.cross(r, j*collision_normal_unit)/BilliardBall.inertia_sphere))
                    
            # Update outcome
            self.update_outgoing_rail_collide(incoming_velo, incoming_angular_velo)

            return collision_index

        collision_index = None
        return collision_index
            
    def update_pock_rail_collide(self):
        collision_index = None
        collision_point = None
        collision_point_adjust = None
        for line_index, pock_rail in enumerate(table.BilliardTable.actual_pock_rail_all):
            collision_point, collision_point_adjust = is_circle_line_segment_intersect(
                np.delete(self.r, common.Z_AXIS),
                np.delete(table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.START], common.Z_AXIS),
                np.delete(table.BilliardTable.actual_pock_rail_all[line_index][table.LineSeg.END], common.Z_AXIS),
                BilliardBall.r_epsilon)
            if collision_point is not None:
                collision_index = line_index
                break
        if collision_point is None:
            return collision_index
        
        collision_normal = self.r - collision_point
        collision_normal_unit = cal_unit_vector(collision_normal)

        r = collision_point - self.r    
        incoming_velo, incoming_angular_velo = self.prepare_incoming_rail_collide()
        relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)

        relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)
        
        if relative_v_dot_n_unit < 0:
            collision_normal = self.r - collision_point_adjust
            collision_normal_unit = cal_unit_vector(collision_normal)
            r = collision_point_adjust - self.r 
            relative_velo = incoming_velo + np.cross(incoming_angular_velo, r)
            relative_v_dot_n_unit = relative_velo.dot(collision_normal_unit)
            
            # Compute Impulse
            j = ((-(1.0 + BallCollideCoef.restitution_coef_cushion) * relative_v_dot_n_unit) /
                 ((1.0/BilliardBall.ball_mass) +
                  collision_normal_unit.dot(
                      np.cross(np.cross(r, collision_normal_unit)/BilliardBall.inertia_sphere, r))))
            
            # Find Tangential vector
            collision_tangent = np.cross(np.cross(collision_normal_unit, relative_velo), collision_normal_unit)
            # Reverse vector is no need
            # collision_tangent[common.X_AXIS], collision_tangent[common.Y_AXIS], collision_tangent[common.Z_AXIS] =
            # -collision_tangent[common.X_AXIS], -collision_tangent[common.Y_AXIS], -collision_tangent[common.Z_AXIS]
            collision_tangent_unit = cal_unit_vector(collision_tangent)
            
            if abs(relative_velo.dot(collision_tangent_unit)) > 0:
                # Tangent dominated
                # Calculate commonly used data
                plus_impulse = (j*collision_normal_unit) + ((BallFrict.friction_coef_rail*j)*collision_tangent_unit)
                # Update linear velocity 
                incoming_velo = incoming_velo + (plus_impulse/BilliardBall.ball_mass)
                # Update angular velocity
                incoming_angular_velo = incoming_angular_velo + (np.cross(r, plus_impulse)/BilliardBall.inertia_sphere)
            else:
                # No tangent
                # Update linear velocity 
                incoming_velo = incoming_velo + ((j*collision_normal_unit)/BilliardBall.ball_mass)
                # Update angular velocity
                incoming_angular_velo = incoming_angular_velo + (np.cross(r, j*collision_normal_unit) /
                                                                 BilliardBall.inertia_sphere)
                
            # Update outcome
            self.update_outgoing_rail_collide(incoming_velo, incoming_angular_velo)
            return collision_index

        collision_index = None
        return collision_index
            
    def update_pock_collide(self):
        for pock_center in table.BilliardTable.actual_pock_center_all:
            collision_normal = self.r - pock_center
            distance_square = collision_normal.dot(collision_normal)

            if distance_square <= table.BilliardTable.r_of_pocket_width_square_epsilon:
                # Clear parameter to Zero
                self.init_collide_outcome()
                # Update position of this ball
                self.r[common.X_AXIS], self.r[common.Y_AXIS], self.r[common.Z_AXIS] = (pock_center[common.X_AXIS],
                                                                                       pock_center[common.Y_AXIS],
                                                                                       BilliardBall.r)
                # print("Pocket Response Ball Name:", self.name)
                return True
                
        return False
            
    def prepare_incoming_ball_collide(self):
        if self.present_state == SLIDING_STATE:
            # return (np.concatenate((counter_clockwise_vector(np.delete(self.u, common.Z_AXIS), self.heading_angle),
            # [0])), np.concatenate((counter_clockwise_vector(np.delete(self.w_plane, common.Z_AXIS),
            # self.heading_angle), [self.w_vertical])))

            return (np.concatenate(
                (counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle), [0])),
                    np.concatenate(
                        (counter_clockwise_vector(np.delete(self.w_plane, common.Z_AXIS), self.heading_angle),
                         [self.w_vertical])))
        elif self.present_state == ROLLING_STATE:
            # return (np.concatenate((counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle),
            # [0])), common.ZERO_VECTOR)
            v_table = np.concatenate(
                (counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle), [0]))
            return (v_table,
                    np.concatenate((np.delete(
                        np.cross(BilliardBall.r_k_vector, v_table)/BilliardBall.abs_r_square, common.Z_AXIS),
                                    [self.w_vertical])))

        elif self.present_state == SPINNING_STATE:
            return common.ZERO_VECTOR, np.array([0, 0, self.w_vertical], dtype=float)
        else:
            return common.ZERO_VECTOR, common.ZERO_VECTOR

    def prepare_incoming_rail_collide(self):
        # if (self.present_state == SLIDING_STATE and self.allow_response_sliding):
        if self.present_state == SLIDING_STATE:
            # return (np.concatenate((counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle),
            # [0])),  np.array([0, 0, self.w_vertical], dtype=float))

            return (np.concatenate(
                (counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle), [0])),
                    np.concatenate(
                        (counter_clockwise_vector(np.delete(self.w_plane, common.Z_AXIS), self.heading_angle),
                         [self.w_vertical])))
            # v_table = (np.concatenate((counter_clockwise_vector(np.delete(self.v, common.Z_AXIS),
            # self.heading_angle), [0])))
            # r = np.array([0, 0, BilliardBall.r], dtype=float)
            # abs_r = np.linalg.norm(r)
            # return (v_table, np.concatenate((np.delete(np.cross(r, v_table)/(abs_r*abs_r)*0.5, common.Z_AXIS),
            # [self.w_vertical])))

        elif (self.present_state == ROLLING_STATE or
                (self.present_state == SLIDING_STATE and (not self.allow_response_sliding))):
            # return (np.concatenate((counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle),
            # [0])), np.array([0, 0, self.w_vertical], dtype=float))
            v_table = np.concatenate((counter_clockwise_vector(np.delete(self.v, common.Z_AXIS), self.heading_angle),
                                      [0]))
            # Decrease Energy from angular effect should be performed?
            return (v_table,
                    np.concatenate(
                        (np.delete(np.cross(BilliardBall.r_k_vector, v_table)/BilliardBall.abs_r_square, common.Z_AXIS),
                         [self.w_vertical])))
        elif self.present_state == SPINNING_STATE:
            return common.ZERO_VECTOR, np.array([0, 0, self.w_vertical], dtype=float)
        else:
            return common.ZERO_VECTOR, common.ZERO_VECTOR

    def update_outgoing_ball_collide(self, velo, angular_velo):
        # collide_heading_angle = ((((np.degrees(np.arctan2([velo.real[common.Y_AXIS]],
        # [velo.real[common.X_AXIS.real]])[0]) +360) % 360)) + 270) % 360

        # collide_v = np.array([0, abs_exclude_z(velo), 0], dtype=float)
        # collide_w = (np.concatenate((clockwise_vector(np.delete(angular_velo, common.Z_AXIS),
        # collide_heading_angle)*0.7, [angular_velo[common.Z_AXIS]])))

        # collide_u = cal_relative_velo_impact(collide_v, collide_w)
        # self.init_collide_outcome(state = STATIONARY_STATE, heading_angle = collide_heading_angle, v = collide_v,
        # w = collide_w, u = collide_u)

        # self.update_state_collide()
        
        collide_heading_angle = ((np.degrees(np.arctan2([velo[common.Y_AXIS].real], [velo[common.X_AXIS].real])[0]) +
                                  270) % 360)
        collide_v = np.array([0, abs_exclude_z(velo), 0], dtype=float)
        # Decrease English effect should be performed?
        collide_w = np.concatenate(
            (clockwise_vector(np.delete(angular_velo, common.Z_AXIS), collide_heading_angle),
             [angular_velo[common.Z_AXIS]]))

        # collide_w = np.concatenate((clockwise_vector(np.delete(angular_velo, common.Z_AXIS),
        # collide_heading_angle)*0.7, [angular_velo[common.Z_AXIS]]))   # Decrease English effect should be performed?
        
        if self.present_state == SLIDING_STATE and self.allow_response_sliding:
            self.allow_response_sliding = False
            # collide_u = cal_relative_velo_impact(velo, angular_velo)
            # collide_u = np.concatenate((clockwise_vector(np.delete(collide_u, common.Z_AXIS), collide_heading_angle),
            # [collide_u[common.Z_AXIS]]))

            collide_u = cal_relative_velo_impact(collide_v, collide_w)
            # collide_w[common.Z_AXIS] = angular_velo[common.Z_AXIS]
        else:
            collide_w = common.ZERO_VECTOR
            collide_u = common.ZERO_VECTOR
        
        self.init_collide_outcome(state=STATIONARY_STATE, heading_angle=collide_heading_angle, v=collide_v,
                                  w=collide_w, u=collide_u)
        self.update_state_collide()

        # print("Collision Response Ball Name:", self.name)
        # print("Collision Response r:", self.r)
        # print("Collision Response u:", self.u)
        # print("Collision Response v:", self.v)
        # print("Collision Response w:", self.w_plane)
        # print("Collision Response head:", self.heading_angle)
        # if self.ball_index == 0:
        #     print("Collision Response t table:", self.t_table)

    def update_outgoing_rail_collide(self, velo, angular_velo):
        collide_heading_angle = ((np.degrees(np.arctan2([velo[common.Y_AXIS].real], [velo[common.X_AXIS].real])[0]) +
                                  270) % 360)
        collide_v = np.array([0, abs_exclude_z(velo), 0], dtype=float)
        collide_w = np.concatenate((clockwise_vector(np.delete(angular_velo, common.Z_AXIS), collide_heading_angle),
                                    [angular_velo[common.Z_AXIS]]))

        if self.present_state == SLIDING_STATE and self.allow_response_sliding:
            self.allow_response_sliding = False

            # collide_u = cal_relative_velo_impact(velo, angular_velo)
            # collide_u = np.concatenate((clockwise_vector(np.delete(collide_u, common.Z_AXIS), collide_heading_angle),
            # [collide_u[common.Z_AXIS]]))

            collide_u = cal_relative_velo_impact(collide_v, collide_w)

            # collide_w = common.ZERO_VECTOR
            # collide_w[common.Z_AXIS] = angular_velo[common.Z_AXIS]
        else:
            # collide_v *= 0.85
            collide_w = common.ZERO_VECTOR
            # collide_w[common.Z_AXIS] = angular_velo[common.Z_AXIS]
            collide_u = common.ZERO_VECTOR
            
        self.init_collide_outcome(state=STATIONARY_STATE, heading_angle=collide_heading_angle, v=collide_v,
                                  w=collide_w, u=collide_u)
        self.update_state_collide()
