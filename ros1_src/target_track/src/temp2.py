import numpy as np

np.set_printoptions(suppress=True)

def pose2mat(t, q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    Nq = w*w + x*x + y*y + z*z
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z


    return np.array(
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY, t[0]],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX, t[1]],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY), t[2]],
            [0, 0, 0, 1]])

t = np.array([0.004, 7.583, 0])
q = np.array([0, 0, 0.25, 0.96])

tar = np.array([2.4, 1.8, 1.5, 1])

T = pose2mat(t, q)

print(t, q)
print(np.dot(T, tar), tar)
