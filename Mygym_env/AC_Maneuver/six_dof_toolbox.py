# -*- coding: utf-8 -*-
# six-dof computation toolbox

# author: tolshao
# 2020-09-10 20:14:13

import numpy as np


class Aircract(object):
    def __init__(self, states0, Tsc=0.02):
        # Aircraft parameters
        self.mass = 9259.44
        Ixx = 12874.8
        Iyy = 75673.6
        Izz = 85552.1
        Ixz = 1331.4
        Ixy = 0
        Iyz = 0
        self.inertia = np.matrix(
            [[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]])
        self.Inv_inertia = self.inertia.I

        # Env paprameters
        self.Tsc = Tsc

        # State variables of C.G. of AC
        x, vel, euler, ang_vel = states0
        self.x = np.matrix(x)
        self.vel = np.matrix(vel)
        self.euler = np.matrix(euler)
        self.ang_vel = np.matrix(ang_vel)
        self.forces = np.zeros((1, 3))
        self.moments = np.zeros((1.3))
        self.dcm = np.matrix(np.eye(3, dtype=float))

    def update_onestep(self, forces, moments):
        print("the input is forces{}, moments{}".format(forces, moments))
        # get the input variables

        # 计算各个向量的微分
        dot_vel = self.DotVel_get(forces)
        dot_ang_vel = self.DotAngVel_get(moments)
        dot_x = self.Vel_g_get()
        dot_euler = self.DotEuler_get()
        # 将变量积分
        self.vel = self.Integrator(self.vel, dot_vel)
        self.ang_vel = self.Integrator(self.ang_vel, dot_ang_vel)
        self.x = self.Integrator(self.x, dot_x)
        self.euler = self.Integrator(self.euler, dot_euler)
        self.DCM_get()

    def Integrator(self, input_X, input_dot_X):
        X = np.matrix(input_X)
        dot_X = np.matrix(input_dot_X)
        assert X.shape == dot_X.shape, 'the dimension is different'
        return X + self.Tsc * dot_X

    def DotAngVel_get(self, input_moments):
        moments = np.matrix(input_moments)
        assert moments.shape == self.ang_vel.shape, 'the dimension is different'
        Iw = np.matmul(self.inertia, self.ang_vel.T)
        w_x_Iw = np.cross(self.ang_vel, Iw.T)
        return np.matmul(self.Inv_inertia, (moments - w_x_Iw).T).T

    def DotVel_get(self, input_forces):
        forces = np.matrix(input_forces)
        assert forces.shape == self.ang_vel.shape, 'the dimension is different'
        assert forces.shape == self.vel.shape, 'the dimension is different'
        v_x_w = np.cross(self.vel, self.ang_vel)
        return forces/self.mass + v_x_w

    def DotEuler_get(self):
        u1, u2, u3 = self.ang_vel[0, 0], self.ang_vel[0, 1], self.ang_vel[0, 2]
        u4, u5, u6 = self.euler[0, 0], self.euler[0, 1], self.euler[0, 2]
        # test print
        # print(u1, u2, type(u1), type(u2))
        out1 = u1 + u2 * np.sin(u4) * np.tan(u5) + u3 * np.cos(u4) * np.tan(u5)
        out2 = u2 * np.cos(u4) - u3 * np.sin(u4)
        out3 = u2*np.sin(u4)/np.cos(u5)+u3*np.cos(u4)/np.cos(u5)
        return np.matrix([out1, out2, out3])

    def DCM_get(self):
        u1, u2, u3 = self.euler[0, 0], self.euler[0, 1], self.euler[0, 2]
        # test print
        # print(u1, u2, u3, type(u1), type(u2), type(u3))
        out = np.matrix(np.empty([3, 3], dtype=float))
        out[0, 0] = np.cos(u3)*np.cos(u2)
        out[1, 0] = np.cos(u3) * np.sin(u2) * np.sin(u1) - \
            np.sin(u3) * np.cos(u1)
        out[2, 0] = np.cos(u3) * np.sin(u2) * np.cos(u1) + \
            np.sin(u3) * np.sin(u1)
        out[0, 1] = np.sin(u3) * np.cos(u2)
        out[1, 1] = np.sin(u3)*np.sin(u2)*np.sin(u1) + np.cos(u3)*np.cos(u1)
        out[2, 1] = np.sin(u3)*np.sin(u2)*np.cos(u1) - np.cos(u3)*np.sin(u1)
        out[0, 2] = - np.sin(u2)
        out[1, 2] = np.cos(u2) * np.sin(u1)
        out[2, 2] = np.cos(u2) * np.cos(u1)
        self.DCM = out
        return out

    def Vel_g_get(self):
        tmp = np.matmul(self.dcm.T, self.vel.T)
        return tmp.T


def test():
    Tsc = 0.01
    print("testing----------------------------------------------------------------------------------------------------")

    x0 = [0, 0, 0]
    vel0 = [1.0, 2.0, 3.0]
    euler0 = [0.1, 0.2, 0.3]
    ang_vel0 = [0.1, 0.2, 0.3]

    initial_parameters = [x0, vel0, euler0, ang_vel0]
    ac = Aircract(initial_parameters, Tsc=Tsc)

    # print("inv_inertia is \n{}".format(ac.Inv_inertia))

    dot_x_temp = [0, 0, 1]
    temp = ac.Integrator(ac.x, dot_x_temp)
    print(temp, type(temp), temp.shape)
    print("1. Func Integrator success")
    moments = [3, 2, 1]

    temp = ac.DotAngVel_get(moments)
    print(temp, type(temp), temp.shape)
    print("2. Func DotAngVel_get success",)

    forces = [1, 2, 3]
    temp = ac.DotVel_get(forces)
    print(temp, type(temp), temp.shape)
    print("3. Func DotVel_get success")

    temp = ac.DotEuler_get()
    print(temp, type(temp), temp.shape)
    print("4. Func DotEuler_get success")

    temp = ac.DCM_get()
    print(temp, type(temp), temp.shape)
    print("5.Func DCM_get success")

    temp = ac.Vel_g_get()
    print(temp, type(temp), temp.shape)
    print("6.Func Vel_get success")


if __name__ == "__main__":
    test()
