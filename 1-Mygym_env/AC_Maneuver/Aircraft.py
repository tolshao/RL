# -*- coding: utf-8 -*-
# Airframe model

# author: tolshao
# 2020-09-10 20:14:13

import numpy as np


class Aircraft(object):
    def __init__(self, states0: float, Tsc=0.02, RK4=False):
        # Env paprameters
        self.Tsc = Tsc
        self.RK4 = RK4

        # Aircraft parameters
        self.mass = 9259.44
        Ixx = 12874.8
        Iyy = 75673.6
        Izz = 85552.1
        Ixz = 1331.4
        Ixy = 0.0
        Iyz = 0.0
        self.inertia = np.mat(
            [[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]])
        self.Inv_inertia = self.inertia.I

        # State variables of C.G. of AC
        x, vel, euler, ang_vel = states0
        self.x = np.mat(x)
        self.vel = np.mat(vel)
        self.euler = np.mat(euler)
        self.ang_vel = np.mat(ang_vel) 
        self.forces = np.mat([0, 0, 0], dtype=float)
        self.moments = np.mat([0, 0, 0], dtype=float)
        self.dcm = np.mat(np.eye(3, dtype=float))

    def update_onestep(self, forces, moments):
        # test print
        # print("the input is forces{}, moments{}".format(forces, moments))
        # get the input variables
        self.forces = np.mat(forces)
        self.moments = np.mat(moments)

        if self.RK4 == False:
            self.Int_0()
        else:
            self.IntRK4()

    def Int_0(self):
        # 刷新坐标系转换矩阵
        self.DCM_refresh()
        # 计算各个向量的微分 并 积分
        self.vel += self.DotVel_get() * self.Tsc
        self.ang_vel += self.DotAngVel_get() * self.Tsc
        self.x += self.Vel_g_get() * self.Tsc
        self.euler += self.DotEuler_get() * self.Tsc

    def IntRK4(self):
        # TODO: RK4 提高积分精度
        # self.RK4ForOne()
        return False

    # def RK4ForOne(self,Func,):

    def DotAngVel_get(self):
        moments = self.moments
        assert moments.shape == self.ang_vel.shape, 'the dimension is different'
        Iw = np.matmul(self.inertia, self.ang_vel.T)
        w_x_Iw = np.cross(self.ang_vel, Iw.T)
        return np.matmul(self.Inv_inertia, (moments - w_x_Iw).T).T

    def DotVel_get(self):
        forces = self.forces
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
        return np.mat([out1, out2, out3])

    def DCM_refresh(self):
        u1, u2, u3 = self.euler[0, 0], self.euler[0, 1], self.euler[0, 2]
        # test print
        # print(u1, u2, u3, type(u1), type(u2), type(u3))
        out = np.mat(np.empty([3, 3], dtype=float))
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

    x0 = [0.0, 0.0, 0.0]
    vel0 = [1.0, 2.0, 3.0]
    euler0 = [0.1, 0.2, 0.3]
    ang_vel0 = [0.1, 0.2, 0.3]

    State0 = (x0, vel0, euler0, ang_vel0)
    ac = Aircraft(State0, Tsc=Tsc)

    # print("inv_inertia is \n{}".format(ac.Inv_inertia))

    ac.Int_0()
    print("1. Func Int_0 success\n ")

    temp = ac.DotAngVel_get()
    print(temp, type(temp), temp.shape)
    print("2. Func DotAngVel_get success\n ",)

    temp = ac.DotVel_get()
    print(temp, type(temp), temp.shape)
    print("3. Func DotVel_get success\n ")

    temp = ac.DotEuler_get()
    print(temp, type(temp), temp.shape)
    print("4. Func DotEuler_get success\n ")

    temp = ac.DCM_refresh()
    print(temp, type(temp), temp.shape)
    print("5. Func DCM_refresh success\n ")

    temp = ac.Vel_g_get()
    print(temp, type(temp), temp.shape)
    print("6. Func Vel_get success\n ")

    moments = [1.0, 2.0, 3.0]
    forces = [1.0, 2.0, 3.0]
    ac.update_onestep(forces, moments)
    print("7. Func update_onestep success\n ")


if __name__ == "__main__":
    test()
