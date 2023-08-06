import numpy as np
import math

J1 = math.radians(0);
J2 = math.radians(0);
J3 = math.radians(0);
J4 = math.radians(0);
J5 = math.radians(0);
J6 = math.radians(0);

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

print(J1)
print(J2)
print(J3)
print(J4)
print(J5)
print(J6)

print(forward_kinematics(J1, J2, J3, J4, J5, J6))
