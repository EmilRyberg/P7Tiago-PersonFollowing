import numpy as np
import math
import glob
import cv2

intrinsics = np.array([[503.97061269, 0., 305.35963149],
                       [0., 503.74823596, 242.63972991],
                       [0., 0., 1.]]) #found through getIntrinsics/main.py

def transform(x, y, depth, rotx, roty, head_height):
    rot_vector = np.array([[rotx, roty, 0]]).T
    translation_vector = np.array([[0, head_height, 0]]).T
    rot_matrix, _ = cv2.Rodrigues(rot_vector)
    rot_matrix_inverse = np.linalg.inv(rot_matrix)
    intrinsic_matrix_inverse = np.linalg.inv(intrinsics)

    # finding correct scaling factor by adjusting it until the calculated Z is very close to the depth reading
    scaling_factor = 10
    index = 0
    while True:
        pixel_coordinates = np.array([[x, y, 1]]).T
        pixel_coordinates = scaling_factor*pixel_coordinates
        xyz_c = intrinsic_matrix_inverse.dot(pixel_coordinates)
        xyz_c = xyz_c - translation_vector
        world_coordinates = rot_matrix_inverse.dot(xyz_c)
        index += 1
        print(world_coordinates[2][0])
        if index > 1000:
            raise Exception("transformTo3D.py: scaling factor finding is taking longer than 1000 iterations")
        if world_coordinates[2][0] > depth + 0.05:
            scaling_factor -= 0.05
        elif world_coordinates[2][0] < depth - 0.05:
            scaling_factor += 0.05
        else:
            print(index)
            break
    print(world_coordinates)
    print(scaling_factor)
    return np.array([world_coordinates[0][0], world_coordinates[1][0], world_coordinates[2][0]])

transform(233, 122, 0.2, 0.23, 0.23, 1.25)
