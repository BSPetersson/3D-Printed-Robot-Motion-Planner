import numpy as np
import math
import matplotlib.pyplot as plt
import time as timeimp
import datetime
import RPi.GPIO as GPIO

ENA = 3
J1_DIR = 5
J1_PUL = 7
J2_DIR = 11
J2_PUL = 13
J3_DIR = 15
J3_PUL = 19
J4_DIR = 33
J4_PUL = 35
J5_DIR = 29
J5_PUL = 31
J6_DIR = 21
J6_PUL = 23

R1 = 47.0
R2 = 110.0
R3 = 26.0
D1 = 133.0
D3 = 0.0
D4 = 117.50
D6 = 28.0

J1_gear_ratio = 4.8
J2_gear_ratio = 4
J3_gear_ratio = 5
J4_gear_ratio = 2.8
J5_gear_ratio = 2.1
J6_gear_ratio = 1

J1_microstep = 32
J2_microstep = 32
J3_microstep = 32
J4_microstep = 16
J5_microstep = 16
J6_microstep = 16

J1_limit = [-math.radians(120), math.radians(120)]
J2_limit = [-math.radians(77), math.radians(136)]
J3_limit = [-math.radians(180), math.radians(60)]
J4_limit = [-math.radians(300), math.radians(300)]
J5_limit = [-math.radians(135), math.radians(135)]
J6_limit = [-math.radians(360), math.radians(360)]

L5 = math.sqrt(math.pow(R3, 2) + math.pow(D4, 2))

max_arm_length = R2 + L5
min_arm_length = R2 - L5

radian_pr_step = (2*math.pi)/200

GPIO.setmode(GPIO.BOARD)

def rad2deg(rad):
    return [math.degrees(rad[0]), math.degrees(rad[1]), math.degrees(rad[2]), math.degrees(rad[3]), math.degrees(rad[4]), math.degrees(rad[5])]

end_effector_frame = np.matrix([[1, 0, 0,  0],
                               [0, 1, 0,  0],
                               [0, 0, 1, 30],
                               [0, 0, 0,  1]])

fold_pose = np.matrix([[-0.2923717,  0, 0.95630476,  44.58343738],
                       [         0, -1,          0,            0],
                       [0.95630476,  0,  0.2923717, 216.96221494],
                       [         0,  0,          0,            1]]).dot(end_effector_frame)

fold_motor_steps = [0, -5550, 6650, 0, 0, 0]

current_motor_steps = [0, -5550, 6650, 0, 0, 0]

fold_joint_angles = [((fold_motor_steps[0]/J1_gear_ratio)*radian_pr_step)/J1_microstep,\
                     ((fold_motor_steps[1]/J2_gear_ratio)*radian_pr_step)/J2_microstep,\
                     ((fold_motor_steps[2]/J3_gear_ratio)*radian_pr_step)/J3_microstep,\
                     ((fold_motor_steps[3]/J4_gear_ratio)*radian_pr_step)/J4_microstep,\
                     ((fold_motor_steps[4]/J5_gear_ratio)*radian_pr_step)/J5_microstep,\
                     ((fold_motor_steps[5]/J6_gear_ratio)*radian_pr_step)/J6_microstep]

current_joint_angles = [((current_motor_steps[0]/J1_gear_ratio)*radian_pr_step)/J1_microstep,\
                        ((current_motor_steps[1]/J2_gear_ratio)*radian_pr_step)/J2_microstep,\
                        ((current_motor_steps[2]/J3_gear_ratio)*radian_pr_step)/J3_microstep,\
                        ((current_motor_steps[3]/J4_gear_ratio)*radian_pr_step)/J4_microstep,\
                        ((current_motor_steps[4]/J5_gear_ratio)*radian_pr_step)/J5_microstep,\
                        ((current_motor_steps[5]/J6_gear_ratio)*radian_pr_step)/J6_microstep]

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(J1_DIR, GPIO.OUT)
GPIO.setup(J1_PUL, GPIO.OUT)
GPIO.setup(J2_DIR, GPIO.OUT)
GPIO.setup(J2_PUL, GPIO.OUT)
GPIO.setup(J3_DIR, GPIO.OUT)
GPIO.setup(J3_PUL, GPIO.OUT)
GPIO.setup(J4_DIR, GPIO.OUT)
GPIO.setup(J4_PUL, GPIO.OUT)
GPIO.setup(J5_DIR, GPIO.OUT)
GPIO.setup(J5_PUL, GPIO.OUT)
GPIO.setup(J6_DIR, GPIO.OUT)
GPIO.setup(J6_PUL, GPIO.OUT)

GPIO.output(ENA, GPIO.LOW)

def forward_kinematics(J1, J2, J3, J4, J5, J6):
    i_11 = math.sin(J6)*(math.cos(J4)*math.sin(J1) + math.sin(J4)*(math.cos(J1)*math.sin(J3)*math.sin(J2 + math.pi/2) - math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2))) + math.cos(J6)*(math.cos(J5)*(math.sin(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2) - math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2))) - math.sin(J5)*(math.cos(J1)*math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J1)*math.cos(J2 - math.pi/2)*math.sin(J3)))
    i_12 = math.cos(J6)*(math.cos(J4)*math.sin(J1) + math.sin(J4)*(math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2) - math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2))) - math.sin(J6)*(math.cos(J5)*(math.sin(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2) - math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2))) - math.sin(J5)*(math.cos(J1)*math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J1)*math.cos(J2 - math.pi/2)*math.sin(J3)))
    i_13 = -math.sin(J5)*(math.sin(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2) - math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2))) - math.cos(J5)*(math.cos(J1)*math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J1)*math.cos(J2 - math.pi/2)*math.sin(J3))
    i_14 = 47*math.cos(J1) + 110*math.cos(J1)*math.cos(J2 - math.pi/2) - (235*math.cos(J1)*math.cos(J3)*math.sin(J2 - math.pi/2))/2 - (235*math.cos(J1)*math.cos(J2 - math.pi/2)*math.sin(J3))/2 - 26*math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2) + 26*math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2)
    i_21 = -math.sin(J6)*(math.cos(J1)*math.cos(J4) + math.sin(J4)*(math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2))) - math.cos(J6)*(math.cos(J5)*(math.cos(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2))) + math.sin(J5)*(math.cos(J3)*math.sin(J1)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J1)*math.sin(J3)))
    i_22 = math.sin(J6)*(math.cos(J5)*(math.cos(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2))) + math.sin(J5)*(math.cos(J3)*math.sin(J1)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J1)*math.sin(J3))) - math.cos(J6)*(math.cos(J1)*math.cos(J4) + math.sin(J4)*(math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2)))
    i_23 = math.sin(J5)*(math.cos(J1)*math.sin(J4) - math.cos(J4)*(math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2))) - math.cos(J5)*(math.cos(J3)*math.sin(J1)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J1)*math.sin(J3))
    i_24 = 47*math.sin(J1) + 110*math.cos(J2 - math.pi/2)*math.sin(J1) + 26*math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - (235*math.cos(J3)*math.sin(J1)*math.sin(J2 - math.pi/2))/2 - (235*math.cos(J2 - math.pi/2)*math.sin(J1)*math.sin(J3))/2 - 26*math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2)
    i_31 = math.sin(J4)*math.sin(J6)*(math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J3)) - math.cos(J6)*(math.sin(J5)*(math.cos(J3)*math.cos(J2 - math.pi/2) - math.sin(J3)*math.sin(J2 - math.pi/2)) + math.cos(J4)*math.cos(J5)*(math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J3)))
    i_32 = math.sin(J6)*(math.sin(J5)*(math.cos(J3)*math.cos(J2 - math.pi/2) - math.sin(J3)*math.sin(J2 - math.pi/2)) + math.cos(J4)*math.cos(J5)*(math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J3))) + math.cos(J6)*math.sin(J4)*(math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J3))
    i_33 = math.cos(J4)*math.sin(J5)*(math.cos(J3)*math.sin(J2 - math.pi/2) + math.cos(J2 - math.pi/2)*math.sin(J3)) - math.cos(J5)*(math.cos(J3)*math.cos(J2 - math.pi/2) - math.sin(J3)*math.sin(J2 - math.pi/2))
    i_34 = (235*math.sin(J3)*math.sin(J2 - math.pi/2))/2 - (235*math.cos(J3)*math.cos(J2 - math.pi/2))/2 - 26*math.cos(J3)*math.sin(J2 - math.pi/2) - 26*math.cos(J2 - math.pi/2)*math.sin(J3) - 110*math.sin(J2 - math.pi/2) + 133

    forward_kinematics_matrix = np.matrix([[i_11, i_12, i_13, i_14],
                                           [i_21, i_22, i_23, i_24],
                                           [i_31, i_32, i_33, i_34],
                                           [   0,    0,    0,    1]])

    return forward_kinematics_matrix


def inverse_kinematics_XYZ_fixed_angles(pos_x, pos_y, pos_z, roll, pitch, yaw):

    geometric_solutions = []
    solutions = []

    J1_1 = None
    J1_2 = None
    J2_1_1 = None
    J2_1_2 = None
    J2_2_1 = None
    J2_2_2 = None
    J3_1_1 = None
    J3_1_2 = None
    J3_2_1 = None
    J3_2_2 = None

    J1_1 = math.atan2(pos_y, pos_x)

    if J1_1 <= 0:
        J1_2 = J1_1 + math.pi
    else:
        J1_2 = J1_1 - math.pi

    L1 = math.sqrt(math.pow(pos_x, 2) + math.pow(pos_y, 2))
    L2 = L1 - R1
    L3 = pos_z - D1
    L4 = math.sqrt(math.pow(L2, 2) + math.pow(L3, 2))
    L5 = math.sqrt(math.pow(R3, 2) + math.pow(D4, 2))
    L6 = L1 + R1
    L7 = math.sqrt(math.pow(L6, 2) + math.pow(L3, 2))

    PHI_3 = math.asin(D4/L5)

    if J1_1 >= J1_limit[0] and J1_1 <= J1_limit[1]:
        if L4 <= max_arm_length and L4 >= min_arm_length:
            #PHI_1 = math.asin(L3/L4)
            PHI_1 = math.atan2(L3, L2)
            PHI_2 = math.acos((math.pow(L4, 2) + math.pow(R2, 2) - math.pow(L5, 2))/(2*L4*R2))
            PHI_4 = math.acos((math.pow(L5, 2) + math.pow(R2, 2) - math.pow(L4, 2))/(2*L5*R2))
            J2_1_1 = math.pi/2 - PHI_1 - PHI_2
            J2_1_2 = math.pi/2 - PHI_1 + PHI_2
            J3_1_1 = math.pi - PHI_3 - PHI_4
            J3_1_2 = -math.pi - PHI_3 + PHI_4
            geometric_solutions.append([J1_1, J2_1_1, J3_1_1])
            geometric_solutions.append([J1_1, J2_1_2, J3_1_2])

    if J1_2 >= J1_limit[0] and J1_2 <= J1_limit[1]:
        if L7 <= max_arm_length and L7 >= min_arm_length:
            #PHI_5 = math.asin(L3/L7)
            PHI_5 = math.atan2(L3, L6)
            PHI_6 = math.acos((math.pow(L7, 2) + math.pow(R2, 2) - math.pow(L5, 2))/(2*L7*R2))
            PHI_7 = math.acos((math.pow(L5, 2) + math.pow(R2, 2) - math.pow(L7, 2))/(2*L5*R2))
            J2_2_1 = -math.pi/2 + PHI_5 + PHI_6
            J2_2_2 = -math.pi/2 + PHI_5 - PHI_6
            J3_2_1 = -math.pi + PHI_7 - PHI_3
            J3_2_2 = +math.pi - PHI_7 - PHI_3
            geometric_solutions.append([J1_2, J2_2_1, J3_2_1])
            geometric_solutions.append([J1_2, J2_2_2, J3_2_2])

    if len(geometric_solutions) > 0:
        transformation_0_to_6_desired = XYZ_fixed_angles_to_transformation_matrix(pos_x, pos_y, pos_z, roll, pitch, yaw)

        for geometric_solution_index in range(len(geometric_solutions)):
            if geometric_solutions[geometric_solution_index][1] >= J2_limit[0] and\
               geometric_solutions[geometric_solution_index][1] <= J2_limit[1] and\
               geometric_solutions[geometric_solution_index][2] >= J3_limit[0] and\
               geometric_solutions[geometric_solution_index][2] <= J3_limit[1]:

                transformation_0_to_3 = find_transformation_matrix_0_to_3(geometric_solutions[geometric_solution_index][0],
                                                                          geometric_solutions[geometric_solution_index][1],
                                                                          geometric_solutions[geometric_solution_index][2])

                transformation_0_to_3_inv = np.linalg.inv(transformation_0_to_3)

                transformation_3_to_6_desired = transformation_0_to_3_inv.dot(transformation_0_to_6_desired)

                J5_1 = math.acos(transformation_3_to_6_desired[1, 2])
                J4_1 = math.atan2(transformation_3_to_6_desired[2, 2]*math.sin(J5_1), -transformation_3_to_6_desired[0, 2]*math.sin(J5_1))
                J6_1 = math.atan2(-transformation_3_to_6_desired[1, 1]*math.sin(J5_1), transformation_3_to_6_desired[1, 0]*math.sin(J5_1))

                J4_2_positive = J4_1 + math.pi
                J4_2_negative = J4_1 - math.pi

                J5_2 = -J5_1

                J6_2_positive = J6_1 + math.pi
                J6_2_negative = J6_1 - math.pi

                if J4_1 >= J4_limit[0] and J4_1 <= J4_limit[1] and\
                   J5_1 >= J5_limit[0] and J5_1 <= J5_limit[1] and\
                   J6_1 >= J6_limit[0] and J6_1 <= J6_limit[1]:
                    solutions.append(geometric_solutions[geometric_solution_index] + [J4_1, J5_1, J6_1])

                if J5_2 >= J5_limit[0] and J5_2 <= J5_limit[1]:
                    if J4_2_positive >= J4_limit[0] and J4_2_positive <= J4_limit[1]:
                        if J6_2_positive >= J6_limit[0] and J6_2_positive <= J6_limit[1]:
                            solutions.append(geometric_solutions[geometric_solution_index] + [J4_2_positive, J5_2, J6_2_positive])
                        if J6_2_negative >= J6_limit[0] and J6_2_negative <= J6_limit[1]:
                            solutions.append(geometric_solutions[geometric_solution_index] + [J4_2_positive, J5_2, J6_2_negative])
                    if J4_2_negative >= J4_limit[0] and J4_2_negative <= J4_limit[1]:
                        if J6_2_positive >= J6_limit[0] and J6_2_positive <= J6_limit[1]:
                            solutions.append(geometric_solutions[geometric_solution_index] + [J4_2_negative, J5_2, J6_2_positive])
                        if J6_2_negative >= J6_limit[0] and J6_2_negative <= J6_limit[1]:
                            solutions.append(geometric_solutions[geometric_solution_index] + [J4_2_negative, J5_2, J6_2_negative])

    return solutions


def fixed_angles_to_rotation_matrix(roll, pitch, yaw):
    rot_z = np.matrix([[math.cos(roll), -math.sin(roll), 0],
                       [math.sin(roll),  math.cos(roll), 0],
                       [0             ,               0, 1]])

    rot_y = np.matrix([[ math.cos(pitch), 0, math.sin(pitch)],
                       [               0, 1,               0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])

    rot_x = np.matrix([[1,             0,              0],
                       [0, math.cos(yaw), -math.sin(yaw)],
                       [0, math.sin(yaw),  math.cos(yaw)]])

    rot_XYZ_fixed_angles = rot_z.dot(rot_y).dot(rot_x)

    return rot_XYZ_fixed_angles


def rotation_matrix_to_fixed_angles(rotation_matrix):
    roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(math.pow(rotation_matrix[2, 1], 2) + math.pow(rotation_matrix[2, 2], 2)))
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return [roll, pitch, yaw]


def XYZ_fixed_angles_to_transformation_matrix(pos_x, pos_y, pos_z, roll, pitch, yaw):
    rotation_matrix = fixed_angles_to_rotation_matrix(roll, pitch, yaw)
    return np.matrix([[rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], pos_x],
                      [rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], pos_y],
                      [rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], pos_z],
                      [                    0,                     0,                     0,     1]])


def transformation_matrix_to_XYZ_fixed_angles(transformation_matrix):
    fixed_angles = rotation_matrix_to_fixed_angles(np.matrix([[transformation_matrix[0, 0], transformation_matrix[0, 1], transformation_matrix[0, 2]],
                                                                 [transformation_matrix[1, 0], transformation_matrix[1, 1], transformation_matrix[1, 2]],
                                                                 [transformation_matrix[2, 0], transformation_matrix[2, 1], transformation_matrix[2, 2]]]))
    return [transformation_matrix[0, 3], transformation_matrix[1, 3], transformation_matrix[2, 3]] + fixed_angles


def find_transformation_matrix_0_to_3(J1, J2, J3):
    i_11 = math.cos(J1)*math.cos(J3)*math.cos(J2 - math.pi/2) - math.cos(J1)*math.sin(J3)*math.sin(J2 - math.pi/2)
    i_12 = -math.cos(J1)*math.cos(J3)*math.sin(J2 - math.pi/2) - math.cos(J1)*math.cos(J2 - math.pi/2)*math.sin(J3)
    i_13 = -math.sin(J1)
    i_14 = 47*math.cos(J1) + 110*math.cos(J1)*math.cos(J2 - math.pi/2)
    i_21 = math.cos(J3)*math.cos(J2 - math.pi/2)*math.sin(J1) - math.sin(J1)*math.sin(J3)*math.sin(J2 - math.pi/2)
    i_22 = -math.cos(J3)*math.sin(J1)*math.sin(J2 - math.pi/2) - math.cos(J2 - math.pi/2)*math.sin(J1)*math.sin(J3)
    i_23 = math.cos(J1)
    i_24 = 47*math.sin(J1) + 110*math.cos(J2 - math.pi/2)*math.sin(J1)
    i_31 = -math.cos(J3)*math.sin(J2 - math.pi/2) - math.cos(J2 - math.pi/2)*math.sin(J3)
    i_32 = math.sin(J3)*math.sin(J2 - math.pi/2) - math.cos(J3)*math.cos(J2 - math.pi/2)
    i_33 = 0
    i_34 = 133 - 110*math.sin(J2 - math.pi/2)

    return np.matrix([[i_11, i_12, i_13, i_14],
                      [i_21, i_22, i_23, i_24],
                      [i_31, i_32, i_33, i_34],
                      [   0,    0,    0,    1]])


def find_cubic_polynomial_coefficients(position_start, position_final, time_final, vel_start=0, vel_final=0):
    a0 = position_start
    a1 = vel_final
    if vel_final == 0 and vel_start == 0:
        a2 = (3/math.pow(time_final, 2))*(position_final - position_start)
        a3 = -(2/math.pow(time_final, 3))*(position_final - position_start)
    else:
        a2 = (3/math.pow(time_final, 2))*(position_final - position_start) - (2/time_final)*vel_start - (1/time_final)*vel_final
        a3 = -(2/math.pow(time_final, 3))*(position_final - position_start) + (1/math.pow(time_final, 2))*(vel_final + vel_final)

    return [a0, a1, a2, a3]


def find_six_cubic_polynomial_coefficients(pose_start, pose_end, time):
    cartesian_cubic_polynomial_coefficients = []
    for i in range(6):
        cartesian_cubic_polynomial_coefficients.append(find_cubic_polynomial_coefficients(position_start=pose_start[i],
                                                                                          position_final=pose_end[i],
                                                                                          time_final=time))
    return cartesian_cubic_polynomial_coefficients


def find_cartesian_via_points_straight_line(pose_start, pose_end, time, time_increments):
    cartesian_cubic_polynomial_coefficients = find_six_cubic_polynomial_coefficients(pose_start, pose_end, time)
    number_of_via_points = int(time/time_increments) #change to distance/distance_increments
    via_points = []
    for i in range(number_of_via_points):
        time = time_increments*i
        via_points.append([cartesian_cubic_polynomial_coefficients[0][0] + cartesian_cubic_polynomial_coefficients[0][1]*time + cartesian_cubic_polynomial_coefficients[0][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[0][3]*math.pow(time, 3),
                           cartesian_cubic_polynomial_coefficients[1][0] + cartesian_cubic_polynomial_coefficients[1][1]*time + cartesian_cubic_polynomial_coefficients[1][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[1][3]*math.pow(time, 3),
                           cartesian_cubic_polynomial_coefficients[2][0] + cartesian_cubic_polynomial_coefficients[2][1]*time + cartesian_cubic_polynomial_coefficients[2][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[2][3]*math.pow(time, 3),
                           cartesian_cubic_polynomial_coefficients[3][0] + cartesian_cubic_polynomial_coefficients[3][1]*time + cartesian_cubic_polynomial_coefficients[3][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[3][3]*math.pow(time, 3),
                           cartesian_cubic_polynomial_coefficients[4][0] + cartesian_cubic_polynomial_coefficients[4][1]*time + cartesian_cubic_polynomial_coefficients[4][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[4][3]*math.pow(time, 3),
                           cartesian_cubic_polynomial_coefficients[5][0] + cartesian_cubic_polynomial_coefficients[5][1]*time + cartesian_cubic_polynomial_coefficients[5][2]*math.pow(time, 2) + cartesian_cubic_polynomial_coefficients[5][3]*math.pow(time, 3)])
    return via_points






def move_to_zero_position():
    delta_angles_to_fold = [-current_joint_angles[0],\
                            -current_joint_angles[1],\
                            -current_joint_angles[2],\
                            -current_joint_angles[3],\
                            -current_joint_angles[4],\
                            -current_joint_angles[5]]
    max_delta_angle = abs(max(delta_angles_to_fold, key=abs))
    time = max_delta_angle*3
    turn_motors(radians=delta_angles_to_fold, time_s=time)

def move_to_fold_position():
    delta_angles_to_fold = [fold_joint_angles[0] - current_joint_angles[0],\
                            fold_joint_angles[1] - current_joint_angles[1],\
                            fold_joint_angles[2] - current_joint_angles[2],\
                            fold_joint_angles[3] - current_joint_angles[3],\
                            fold_joint_angles[4] - current_joint_angles[4],\
                            fold_joint_angles[5] - current_joint_angles[5]]
    max_delta_angle = abs(max(delta_angles_to_fold, key=abs))
    time = max_delta_angle*3
    turn_motors(radians=delta_angles_to_fold, time_s=time)

def move_to_pose_straight_line(goal_pose, speed_mm_pr_s, time_increments=0.02):
    goal_pose = [goal_pose[0], goal_pose[1], goal_pose[2], math.radians(goal_pose[3]), math.radians(goal_pose[4]), math.radians(goal_pose[5])]
    goal_end_effector_pose = XYZ_fixed_angles_to_transformation_matrix(goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5])
    current_end_effector_pose = forward_kinematics(current_joint_angles[0], current_joint_angles[1], current_joint_angles[2], current_joint_angles[3], current_joint_angles[4], current_joint_angles[5]).dot(end_effector_frame)
    travel_distance_mm = math.sqrt(math.pow(goal_end_effector_pose[0, 3] - current_end_effector_pose[0, 3], 2) +\
                                   math.pow(goal_end_effector_pose[1, 3] - current_end_effector_pose[1, 3], 2) +\
                                   math.pow(goal_end_effector_pose[2, 3] - current_end_effector_pose[2, 3], 2))
    travel_time_s = travel_distance_mm/speed_mm_pr_s
    end_effector_frame_inv = np.linalg.inv(end_effector_frame)
    current_wrist_pose = transformation_matrix_to_XYZ_fixed_angles(current_end_effector_pose.dot(end_effector_frame_inv))
    goal_wrist_pose = transformation_matrix_to_XYZ_fixed_angles(goal_end_effector_pose.dot(end_effector_frame_inv))

    cartesian_via_points = find_cartesian_via_points_straight_line(pose_start=current_wrist_pose, pose_end=goal_wrist_pose, time=travel_time_s, time_increments=time_increments)

    for via_point_number in range(len(cartesian_via_points)):
        via_point_solutions = inverse_kinematics_XYZ_fixed_angles(pos_x=cartesian_via_points[via_point_number][0],\
                                                                  pos_y=cartesian_via_points[via_point_number][1],\
                                                                  pos_z=cartesian_via_points[via_point_number][2],\
                                                                  roll=cartesian_via_points[via_point_number][3],\
                                                                  pitch=cartesian_via_points[via_point_number][4],\
                                                                  yaw=cartesian_via_points[via_point_number][5])
        if not len(via_point_solutions) == 0:
            best_solution = None
            best_solution_largest_radian_to_move = None
            for via_point_solution in via_point_solutions:
                largest_radian_to_move = max([abs(via_point_solution[0] - current_joint_angles[0]),
                                            abs(via_point_solution[1] - current_joint_angles[1]),
                                            abs(via_point_solution[2] - current_joint_angles[2]),
                                            abs(via_point_solution[3] - current_joint_angles[3]),
                                            abs(via_point_solution[4] - current_joint_angles[4]),
                                            abs(via_point_solution[5] - current_joint_angles[5])])
                if best_solution == None or largest_radian_to_move < best_solution_largest_radian_to_move:
                    best_solution = via_point_solution
                    best_solution_largest_radian_to_move = largest_radian_to_move

            delta_angles = [best_solution[0] - current_joint_angles[0],\
                            best_solution[1] - current_joint_angles[1],\
                            best_solution[2] - current_joint_angles[2],\
                            best_solution[3] - current_joint_angles[3],\
                            best_solution[4] - current_joint_angles[4],\
                            best_solution[5] - current_joint_angles[5]]


            turn_motors(delta_angles, time_increments)
        else:
            print("No Solution")

def move_to_pose(goal_pose, time):
    goal_pose = [goal_pose[0], goal_pose[1], goal_pose[2], math.radians(goal_pose[3]), math.radians(goal_pose[4]), math.radians(goal_pose[5])]

    solutions = inverse_kinematics_XYZ_fixed_angles(pos_x=goal_pose[0],\
                                                    pos_y=goal_pose[1],\
                                                    pos_z=goal_pose[2],\
                                                    roll=goal_pose[3],\
                                                    pitch=goal_pose[4],\
                                                    yaw=goal_pose[5])

    if not len(solutions) == 0:
        best_solution = None
        best_solution_largest_radian_to_move = None
        for solution in solutions:
            largest_radian_to_move = max([abs(solution[0] - current_joint_angles[0]),
                                          abs(solution[1] - current_joint_angles[1]),                                              abs(solution[2] - current_joint_angles[2]),
                                          abs(solution[3] - current_joint_angles[3]),
                                          abs(solution[4] - current_joint_angles[4]),
                                          abs(solution[5] - current_joint_angles[5])])

            if best_solution == None or largest_radian_to_move < best_solution_largest_radian_to_move:
                best_solution = solution
                best_solution_largest_radian_to_move = largest_radian_to_move


        delta_angles = [best_solution[0] - current_joint_angles[0],\
                        best_solution[1] - current_joint_angles[1],\
                        best_solution[2] - current_joint_angles[2],\
                        best_solution[3] - current_joint_angles[3],\
                        best_solution[4] - current_joint_angles[4],\
                        best_solution[5] - current_joint_angles[5]]

        turn_motors(delta_angles, time)
    else:
        print("no solution!")

def move_to_pose_all_solutions(goal_pose, time):
    goal_pose = [goal_pose[0], goal_pose[1], goal_pose[2], math.radians(goal_pose[3]), math.radians(goal_pose[4]), math.radians(goal_pose[5])]

    solutions = inverse_kinematics_XYZ_fixed_angles(pos_x=goal_pose[0],\
                                                    pos_y=goal_pose[1],\
                                                    pos_z=goal_pose[2],\
                                                    roll=goal_pose[3],\
                                                    pitch=goal_pose[4],\
                                                    yaw=goal_pose[5])

    if not len(solutions) == 0:
        for solution in solutions:

            delta_angles = [solution[0] - current_joint_angles[0],\
                            solution[1] - current_joint_angles[1],\
                            solution[2] - current_joint_angles[2],\
                            solution[3] - current_joint_angles[3],\
                            solution[4] - current_joint_angles[4],\
                            solution[5] - current_joint_angles[5]]

            turn_motors(delta_angles, time)

            timeimp.sleep(0.5)
    else:
        print("no solution!")





def turn_motors(radians, time_s):

    print(radians)

    max_speed = 4000

    if radians[0] > 0:
        GPIO.output(J1_DIR, GPIO.LOW)
        J1_dir_sign = 1
    elif radians[0] < 0:
        GPIO.output(J1_DIR, GPIO.HIGH)
        J1_dir_sign = -1

    if radians[1] > 0:
        GPIO.output(J2_DIR, GPIO.LOW)
        J2_dir_sign = 1
    elif radians[1] < 0:
        GPIO.output(J2_DIR, GPIO.HIGH)
        J2_dir_sign = -1

    if radians[2] > 0:
        GPIO.output(J3_DIR, GPIO.HIGH)
        J3_dir_sign = 1
    elif radians[2] < 0:
        GPIO.output(J3_DIR, GPIO.LOW)
        J3_dir_sign = -1

    if radians[3] > 0:
        GPIO.output(J4_DIR, GPIO.LOW)
        J4_dir_sign = 1
    elif radians[3] < 0:
        GPIO.output(J4_DIR, GPIO.HIGH)
        J4_dir_sign = -1

    if radians[4] > 0:
        GPIO.output(J5_DIR, GPIO.HIGH)
        J5_dir_sign = 1
    elif radians[4] < 0:
        GPIO.output(J5_DIR, GPIO.LOW)
        J5_dir_sign = -1

    if radians[5] > 0:
        GPIO.output(J6_DIR, GPIO.HIGH)
        J6_dir_sign = 1
    elif radians[5] < 0:
        GPIO.output(J6_DIR, GPIO.LOW)
        J6_dir_sign = -1

    time_ms = time_s*1000000
    J1_steps = ((abs(radians[0])*J1_gear_ratio)/radian_pr_step)*J1_microstep
    J2_steps = ((abs(radians[1])*J2_gear_ratio)/radian_pr_step)*J2_microstep
    J3_steps = ((abs(radians[2])*J3_gear_ratio)/radian_pr_step)*J3_microstep
    J4_steps = ((abs(radians[3])*J4_gear_ratio)/radian_pr_step)*J4_microstep
    J5_steps = ((abs(radians[4])*J5_gear_ratio)/radian_pr_step)*J5_microstep
    J6_steps = ((abs(radians[5])*J6_gear_ratio)/radian_pr_step)*J6_microstep

    J1_speed_steps_pr_sec = J1_steps/time_s
    J2_speed_steps_pr_sec = J2_steps/time_s
    J3_speed_steps_pr_sec = J3_steps/time_s
    J4_speed_steps_pr_sec = J4_steps/time_s
    J5_speed_steps_pr_sec = J5_steps/time_s
    J6_speed_steps_pr_sec = J6_steps/time_s

    print(J1_speed_steps_pr_sec)
    print(J2_speed_steps_pr_sec)
    print(J3_speed_steps_pr_sec)
    print(J4_speed_steps_pr_sec)
    print(J5_speed_steps_pr_sec)
    print(J6_speed_steps_pr_sec)

    if not J1_steps == 0:
        J1_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J1_steps)/2) #change to seconds
        J1_microsecond_delay_now = J1_microsecond_delay
    else:
        J1_microsecond_delay = None
        J1_microsecond_delay_now = J1_microsecond_delay
    if not J2_steps == 0:
        J2_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J2_steps)/2)
        J2_microsecond_delay_now = J2_microsecond_delay
    else:
        J2_microsecond_delay = None
        J2_microsecond_delay_now = J2_microsecond_delay
    if not J3_steps == 0:
        J3_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J3_steps)/2)
        J3_microsecond_delay_now = J3_microsecond_delay
    else:
        J3_microsecond_delay = None
        J3_microsecond_delay_now = J3_microsecond_delay
    if not J4_steps == 0:
        J4_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J4_steps)/2)
        J4_microsecond_delay_now = J4_microsecond_delay
    else:
        J4_microsecond_delay = None
        J4_microsecond_delay_now = J4_microsecond_delay
    if not J5_steps == 0:
        J5_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J5_steps)/2)
        J5_microsecond_delay_now = J5_microsecond_delay
    else:
        J5_microsecond_delay = None
        J5_microsecond_delay_now = J5_microsecond_delay
    if not J6_steps == 0:
        J6_microsecond_delay = datetime.timedelta(microseconds=(time_ms/J6_steps)/2)
        J6_microsecond_delay_now = J6_microsecond_delay
    else:
        J6_microsecond_delay = None
        J6_microsecond_delay_now = J6_microsecond_delay

    J1_pul_state = True
    J2_pul_state = True
    J3_pul_state = True
    J4_pul_state = True
    J5_pul_state = True
    J6_pul_state = True

    start_time = datetime.datetime.now()
    end_time = datetime.datetime.now() + datetime.timedelta(seconds=time_s)

    if J1_speed_steps_pr_sec < max_speed and\
       J2_speed_steps_pr_sec < max_speed and\
       J3_speed_steps_pr_sec < max_speed and\
       J4_speed_steps_pr_sec < max_speed and\
       J5_speed_steps_pr_sec < max_speed and\
       J6_speed_steps_pr_sec < max_speed:

        while datetime.datetime.now() < end_time:
            timer = datetime.datetime.now() - start_time

            if not J1_microsecond_delay_now == None:
                if timer > J1_microsecond_delay_now:
                    GPIO.output(J1_PUL, J1_pul_state)
                    J1_pul_state = not J1_pul_state
                    J1_microsecond_delay_now += J1_microsecond_delay
                    current_motor_steps[0] += 0.5*J1_dir_sign

            if not J2_microsecond_delay_now == None:
                if timer > J2_microsecond_delay_now:
                    GPIO.output(J2_PUL, J2_pul_state)
                    J2_pul_state = not J2_pul_state
                    J2_microsecond_delay_now += J2_microsecond_delay
                    current_motor_steps[1] += 0.5*J2_dir_sign

            if not J3_microsecond_delay_now == None:
                if timer > J3_microsecond_delay_now:
                    GPIO.output(J3_PUL, J3_pul_state)
                    J3_pul_state = not J3_pul_state
                    J3_microsecond_delay_now += J3_microsecond_delay
                    current_motor_steps[2] += 0.5*J3_dir_sign

            if not J4_microsecond_delay_now == None:
                if timer > J4_microsecond_delay_now:
                    GPIO.output(J4_PUL, J4_pul_state)
                    J4_pul_state = not J4_pul_state
                    J4_microsecond_delay_now += J4_microsecond_delay
                    current_motor_steps[3] += 0.5*J4_dir_sign

            if not J5_microsecond_delay_now == None:
                if timer > J5_microsecond_delay_now:
                    GPIO.output(J5_PUL, J5_pul_state)
                    J5_pul_state = not J5_pul_state
                    J5_microsecond_delay_now += J5_microsecond_delay
                    current_motor_steps[4] += 0.5*J5_dir_sign

            if not J6_microsecond_delay_now == None:
                if timer > J6_microsecond_delay_now:
                    GPIO.output(J6_PUL, J6_pul_state)
                    J6_pul_state = not J6_pul_state
                    J6_microsecond_delay_now += J6_microsecond_delay
                    current_motor_steps[5] += 0.5*J6_dir_sign

        if not J1_pul_state:
            GPIO.output(J1_PUL, J1_pul_state)
            current_motor_steps[0] += 0.5

        if not J2_pul_state:
            GPIO.output(J2_PUL, J2_pul_state)
            current_motor_steps[1] += 0.5

        if not J3_pul_state:
            GPIO.output(J3_PUL, J3_pul_state)
            current_motor_steps[2] += 0.5

        if not J4_pul_state:
            GPIO.output(J4_PUL, J4_pul_state)
            current_motor_steps[3] += 0.5

        if not J5_pul_state:
            GPIO.output(J5_PUL, J5_pul_state)
            current_motor_steps[4] += 0.5

        if not J6_pul_state:
            GPIO.output(J6_PUL, J6_pul_state)
            current_motor_steps[5] += 0.5

        current_joint_angles[0] = ((current_motor_steps[0]/J1_gear_ratio)*radian_pr_step)/J1_microstep
        current_joint_angles[1] = ((current_motor_steps[1]/J2_gear_ratio)*radian_pr_step)/J2_microstep
        current_joint_angles[2] = ((current_motor_steps[2]/J3_gear_ratio)*radian_pr_step)/J3_microstep
        current_joint_angles[3] = ((current_motor_steps[3]/J4_gear_ratio)*radian_pr_step)/J4_microstep
        current_joint_angles[4] = ((current_motor_steps[4]/J5_gear_ratio)*radian_pr_step)/J5_microstep
        current_joint_angles[5] = ((current_motor_steps[5]/J6_gear_ratio)*radian_pr_step)/J6_microstep
    else:
        print("To fast!")







move_to_zero_position()

move_to_pose([0, -170, 270, 0, 0, 90], 5)

move_to_pose([0, -170, 220, -45, 0, 90], 2)

move_to_pose([0, -170, 300, 45, 0, 90], 2)

move_to_pose([0, -170, 220, -45, 0, 90], 2)

move_to_pose([0, -170, 300, 45, 0, 90], 2)

move_to_pose([180, -120, 110, 0, 0, 90], 3)

move_to_pose([180, 120, 110, 0, 0, -90], 4)

move_to_pose_all_solutions([-5, 30, 320, 0, 0, 0], 4)

move_to_pose([-100, 0, 300, 0, 90, 0], 5)

move_to_fold_position()
