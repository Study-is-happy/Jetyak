#!/usr/bin/env python
import numpy as np
import scipy.linalg
import my_util


class CTRV:

    def __init__(self, x, P, R, Q, Q_aug, dt):

        self.x = x
        self.dim_x = len(x)
        self.P = P
        self.R = R
        self.dim_z = len(R)
        self.Q = Q
        self.Q_aug = Q_aug
        self.dim_x_aug = self.dim_x+len(Q_aug)
        self.H = np.array(
            [[1, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1], [0, 0, 0]])
        self.dt = dt

        self.weights = np.zeros(self.dim_x_aug*2+1)

        self.kalman_lambda = 3.-self.dim_x_aug

        self.weights[0] = self.kalman_lambda / \
            (self.kalman_lambda+self.dim_x_aug)
        for i in range(1, self.dim_x_aug*2+1):
            self.weights[i] = 1./(2.*(self.dim_x_aug+self.kalman_lambda))

        # self.weights[0] = 0.3
        # for i in range(1, self.dim_x_aug*2+1):
        #     self.weights[i] = (1-self.weights[0])/(2.*self.dim_x_aug)

    def fx(self, x_aug):

        if abs(my_util.normalize_radian(x_aug[4])) < np.pi/1800:
            deterministic = np.array([x_aug[2]*np.cos(x_aug[3])*self.dt,
                                      x_aug[2]*np.sin(x_aug[3])*self.dt, 0, 0, 0])
        else:
            deterministic = np.array([x_aug[2]/x_aug[4] *
                                      (np.sin(x_aug[3]+x_aug[4]
                                              * self.dt)-np.sin(x_aug[3])),
                                      x_aug[2]/x_aug[4] *
                                      (-np.cos(x_aug[3]+x_aug[4]
                                               * self.dt)+np.cos(x_aug[3])),
                                      0, x_aug[4]*self.dt, 0])
        stochastic = np.array([0.5*self.dt**2*np.cos(x_aug[3])*x_aug[5], 0.5 *
                               self.dt**2*np.sin(x_aug[3])*x_aug[5], self.dt*x_aug[5], 0.5*self.dt**2*x_aug[6], self.dt*x_aug[6]])
        return x_aug[:5] + deterministic + stochastic

    # def hx(self, x_pred):

    #     return np.array([np.sqrt(x_pred[0]**2+x_pred[1]**2), np.arctan2(x_pred[1], x_pred[0]),
    #                      (x_pred[0]*np.cos(x_pred[3])*x_pred[2]+x_pred[1]*np.sin(x_pred[3])*x_pred[2]) /
    #                      np.sqrt(x_pred[0]**2+x_pred[1]**2)])

    def predict(self):

        x_aug = np.append(self.x, np.zeros(2))
        P_aug = np.block(
            [[self.P, np.zeros((self.dim_x, self.dim_x_aug-self.dim_x))],
             [np.zeros((self.dim_x_aug-self.dim_x, self.dim_x)), self.Q_aug]])

        L = scipy.linalg.cholesky(P_aug)
        x_aug_sigma = np.zeros((self.dim_x_aug*2+1, self.dim_x_aug))
        x_aug_sigma[0] = x_aug
        for i in range(self.dim_x_aug):

            x_aug_sigma[i+1] = x_aug + \
                np.sqrt(self.kalman_lambda+self.dim_x_aug)*L[i]
            x_aug_sigma[i+1+self.dim_x_aug] = x_aug - \
                np.sqrt(self.kalman_lambda+self.dim_x_aug)*L[i]

            # x_aug_sigma[i+1] = x_aug + \
            #     np.sqrt(self.dim_x_aug/(1.-self.weights[0]))*L[i]
            # x_aug_sigma[i+1+self.dim_x_aug] = x_aug - \
            #     np.sqrt(self.dim_x_aug/(1.-self.weights[0]))*L[i]

        self.x_pred_sigma = np.zeros((self.dim_x_aug*2+1, self.dim_x))
        for i in range(self.dim_x_aug*2+1):
            self.x_pred_sigma[i] = self.fx(x_aug_sigma[i])

        self.x.fill(0)
        for i in range(self.dim_x_aug*2 + 1):
            self.x += self.weights[i] * self.x_pred_sigma[i]

        self.P.fill(0)
        for i in range(self.dim_x_aug*2 + 1):
            x_diff = self.x_pred_sigma[i]-self.x
            x_diff[3] = my_util.normalize_radian(x_diff[3])
            x_diff[4] = my_util.normalize_radian(x_diff[4])
            x_diff = np.array([x_diff])
            self.P += self.weights[i]*x_diff.T.dot(x_diff)

        self.P += self.Q

        # self.z_sigma = np.zeros((2 * self.dim_x_aug + 1, self.dim_z))
        # for i in range(2 * self.dim_x_aug + 1):
        #     self.z_sigma[i] = self.hx(self.x_pred_sigma[i])

        # self.z_pred = np.zeros(self.dim_z)
        # for i in range(2 * self.dim_x_aug + 1):
        #     self.z_pred += self.weights[i] * self.z_sigma[i]

        self.z_pred = self.x.dot(self.H)

    def update(self, z):

        y = z-self.z_pred

        # y[1] = my_util.normalize_radian(y[1])

        # S = np.zeros((self.dim_z, self.dim_z))
        # for i in range(self.dim_x_aug*2 + 1):
        #     z_diff = self.z_sigma[i]-self.z_pred
        #     z_diff[1] = my_util.normalize_radian(z_diff[1])
        #     z_diff = np.array([z_diff])
        #     S += self.weights[i]*z_diff.T.dot(z_diff)

        # S += self.R

        # Tc = np.zeros((self.dim_z, self.dim_x))
        # for i in range(self.dim_x_aug*2 + 1):
        #     z_diff = self.z_sigma[i]-self.z_pred
        #     z_diff[1] = my_util.normalize_radian(z_diff[1])
        #     z_diff = np.array([z_diff])
        #     x_diff = self.x_pred_sigma[i]-self.x
        #     x_diff[3] = my_util.normalize_radian(x_diff[3])
        #     x_diff = np.array([x_diff])
        #     Tc += self.weights[i]*z_diff.T.dot(x_diff)

        y[2] = my_util.normalize_radian(y[2])

        # S = np.zeros((self.dim_z, self.dim_z))
        # for i in range(self.dim_x_aug*2 + 1):
        #     z_diff = self.z_sigma[i]-self.z_pred
        #     z_diff[2] = my_util.normalize_radian(z_diff[2])
        #     z_diff = np.array([z_diff])
        #     S += self.weights[i]*z_diff.T.dot(z_diff)
        # S += self.R

        # Tc = np.zeros((self.dim_z, self.dim_x))
        # for i in range(self.dim_x_aug*2 + 1):
        #     z_diff = self.z_sigma[i]-self.z_pred
        #     z_diff[2] = my_util.normalize_radian(z_diff[2])
        #     z_diff = np.array([z_diff])
        #     x_diff = self.x_pred_sigma[i]-self.x
        #     x_diff[3] = my_util.normalize_radian(x_diff[3])
        #     x_diff[4] = my_util.normalize_radian(x_diff[4])
        #     x_diff = np.array([x_diff])
        #     Tc += self.weights[i]*z_diff.T.dot(x_diff)

        S = self.H.T.dot(self.P).dot(self.H)+self.R

        K = np.linalg.inv(S).dot(self.H.T).dot(self.P)

        self.x += y.dot(K)

        self.P -= K.T.dot(S).dot(K)


# np.set_printoptions(suppress=True)
# np.set_printoptions(precision=2)

# zs = [[0, -5, 0],
#       [5/np.sqrt(2), -5/np.sqrt(2), np.pi/4],
#       [5, 0, np.pi/2],
#       [5/np.sqrt(2), 5/np.sqrt(2), np.pi*3/4],
#       [0, 5, np.pi]]

# dt = 2.5
# CTRV_x = np.array(
#     [zs[0][0], zs[0][1], 0., zs[0][2], 0.])

# CTRV_P = np.diag(
#     [1.**2, 1.**2, 5.**2, (np.pi/8)**2, (np.pi/12)**2])
# CTRV_R = np.diag([1.**2, 1.**2, (np.pi/8)**2])
# CTRV_Q = np.diag(
#     [(0.5*0.5*dt**2)**2, (0.5*0.5*dt**2)**2, (0.5*dt)**2, (np.pi/48*dt)**2, (np.pi/48*dt)**2])
# CTRV_Q_aug = np.diag([0.5**2, (np.pi/48)**2])

# kalman = CTRV(CTRV_x, CTRV_P, CTRV_R, CTRV_Q, CTRV_Q_aug, dt)

# for i in range(4):
#     kalman.predict()
#     kalman.update(zs[i+1])

# for i in range(1):
#     kalman.predict()
#     print kalman.x
