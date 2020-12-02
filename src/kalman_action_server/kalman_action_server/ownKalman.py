import numpy as np
from numpy import dot, zeros, eye

np.set_printoptions(suppress=True)

# Must do track() before adding new filters! 
class KfTracker:
    # The measurement matrix
    H = np.array([[1., 0., 0., 0.],
                  [0., 1., 0., 0.]])
    
    # Just preallocated
    _I = eye(4)

    # Used for adding the noise variances
    r = 5.
    q = 0
    R = np.eye(2) * r
    Q_base = _I * q

    def __init__(self, map_position, time_stamp):
        self.prevTime = time_stamp
        self.is_tracked = True
        self.x = np.array([[map_position[0]], [map_position[1]], [0.], [0.]])
        self.P = np.eye(4) * 1000.
        # The dynamics matrix. [0, 2] and [1, 3] will be replaced by dt (sampling period)
        self.F = np.array([[1., 0., 1., 0.],
                      [0., 1., 0., 1.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]])

    def predict(self, timeStamp):
        dt = timeStamp - self.prevTime
        self.prevTime = timeStamp
        self.F[0, 2] = dt
        self.F[1, 3] = dt

        # x = Fx
        self.x = dot(self.F, self.x)

        Q = self.Q_base * dt

        # P = FPF' + Q
        self.P = dot(dot(self.F, self.P), self.F.T) + Q


    def update(self, z):
        # y = z - Hx
        # error (residual) between measurement and prediction
        y = z - dot(self.H, self.x)

        # S = HPH' + R
        # project system uncertainty into measurement space
        PHT = dot(self.P, self.H.T) # Written here for optimization, used twice
        S = dot(self.H, PHT) + self.R
        # K = PH'inv(S)
        K = dot(PHT, np.linalg.inv(S))

        # x = x + Ky
        self.x += dot(K, y)

        # P = (I-KH)P
        #self.kf[i].P = dot(self._I - dot(K, self.H), self.kf[i].P) 
        # The quation below is supposed to be more numerically stable, but more expensive
        # P = (I-KH)P(I-KH)' + KRK'
        I_KH = self._I - dot(K, self.H)
        self.P = dot(dot(I_KH, self.P), I_KH.T) + dot(dot(K, self.R), K.T) 



#mPos = np.array([0, 0])
#kf = KfTracker(mPos, 0)

#print(kf.x)

#k = 0
#t = 0

#for i in range(10):
#    k = i
#    t = i
#    mPos = np.array([[k*2], [k]])
#    kf.track(mPos, t * 0.5)
#    print("state 1:")
#    print(kf.x)
#    print(mPos)

#for i in range(10):
#    t += 1
#    mPos = np.array([[k*2], [k]])
#    kf.track(mPos, t)
#    print("state 2:")
#    print(kf.x)
#    print(mPos)
