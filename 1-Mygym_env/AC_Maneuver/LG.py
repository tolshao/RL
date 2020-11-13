# -*- coding: utf-8 -*-
# LG model

# author: tolshao
# 2020-09-11 17:18:49

import numpy as np
import math


class LandingGear(object):
    # new the class LG object
    def __init__(self, Tsc=0.02, RK4=False):
        # Env paprameters
        self.Tsc = Tsc
        self.RK4 = RK4

        # Aircraft parameters
        self.pos_vec = np.matrix(np.zeros((1, 3)))

        # State variables of C.G. of AC
        self.vel = np.matrix(np.zeros((1, 3)))
        self.ang_vel = np.matrix(np.zeros((1, 3)))

        # State variables of LG
        self.x = np.matrix(np.zeros((1, 3)))
        self.rotzmatrix = np.matrix(np.eye(3, dtype=float))
        self.k_alpha = 1.0
        self.F_z = 150000.0

    # position from ac coordinate to LG cooordinate
    def Cal_vel_AC2LG(self, vel, ang_vel, x, dcm):
        # shape check
        assert vel.shape == (1, 3), 'dimension error'
        assert ang_vel.shape == (1, 3), 'dimension error'
        assert x.shape == (1, 3), 'dimension error'
        assert dcm.shape == (3, 3), 'dimension error'
        self.vel = np.cross(ang_vel, self.pos_vec) + vel
        self.x = (np.matmul(dcm.T, self.pos_vec.T)).T + x
        return self.vel, self.x

    # calculate the forces on the tire
    def Cal_SideSlipAngle(self):
        vel_temp = self.vel
        if vel_temp[0, 0] == 0:
            self.alpha = np.sign(vel_temp[0, 1]) * np.pi * 0.5
        else:
            self.alpha = math.atan2(
                vel_temp[0, 1], vel_temp[0, 0])
        return self.alpha

    def Force_x(self):
        return 0.0

    def Forces(self):
        ans = np.matrix([self.Force_x(), self.Cal_SideSlipAngle()
                         * self.k_alpha * self.F_z, self.F_z])
        self.forces = ans
        return ans

    def Moments(self):
        ans = np.cross(self.pos_vec, self.forces)
        return ans


class NoseLandingGear(LandingGear):
    # new the NLG object inherit from LG
    def __init__(self, Tsc=0.02, RK4=False):
        super().__init__(Tsc, RK4)
        self.pos_vec = np.matrix([8.5, 0.0, 2.0])
        self.delta = 0.0

    def Cal_SideSlipAngle(self, delta):
        self.delta = delta
        vel_temp = self.vel
        if vel_temp[0, 0] == 0:
            self.alpha = np.sign(vel_temp[0, 1]) * np.pi * 0.5 - self.delta
        else:
            self.alpha = math.atan2(
                vel_temp[0, 1], vel_temp[0, 0]) - self.delta
        return self.alpha

    def RotZ(self):
        ans = np.matrix([[math.cos(self.alpha), -math.sin(self.alpha), 0],
                         [math.sin(self.alpha), math.cos(self.alpha), 0], [0, 0, 1]])
        return ans

    def Forces(self):
        ans = np.matrix([self.Force_x(), self.Cal_SideSlipAngle(self.delta)
                         * self.k_alpha * self.F_z, self.F_z])
        self.rotzmatrix = self.RotZ()
        assert self.rotzmatrix.shape == (3, 3), "matrix shape error"
        assert ans.T.shape == (3, 1), "matrix shape erroe"

        self.forces = np.matmul(self.rotzmatrix, ans.T).T
        return ans


class MainLandingGear(LandingGear):
    # new the MLG object inherit from LG
    # 根据'L','R'决定位矢
    def __init__(self, direction, Tsc=0.02, RK4=False):
        super().__init__(Tsc, RK4)
        if direction == 'L':
            self.pos_vec = np.matrix([-1.5, -2, -2])
        elif direction == 'R':
            self.pos_vec = np.matrix([-1.5, 2, 2])
        else:
            raise "please define the LG to the L/R"


def test():
    # 测试函数
    Tsc = 0.01
    print("testing----------------------------------------------------------------------------------------------------")

    LG = LandingGear()
    vel_tmp = np.matrix([1, 0, 0])
    ang_vel_tmp = np.matrix([0, 0, 0.1])
    x_tmp = np.matrix([0, 0, 0])
    DCM_tmp = np.matrix(np.eye(3, dtype=float))
    print(vel_tmp, ang_vel_tmp, x_tmp, DCM_tmp)

    LG.Cal_vel_AC2LG(vel_tmp, ang_vel_tmp, x_tmp, DCM_tmp)
    print(type(LG.vel),  type(LG.x))
    print(LG.vel.shape,  LG.x.shape)
    print("1. Func Cal_vel_AC2LG success")

    NLG = NoseLandingGear(Tsc=Tsc)
    print(NLG.pos_vec)
    print("2. inherit NLG success")

    LMLG = MainLandingGear('L')
    print(LMLG.pos_vec)
    print("3. inherit LMLG success")

    RMLG = MainLandingGear('R')
    print(RMLG.pos_vec)
    print("4. inherit RMLG success")

    temp = np.matrix([1, 1, 1])
    temp_0 = np.matrix([0, 0, 0])
    a, b = NLG.Cal_vel_AC2LG(temp, temp / 1000, temp_0, DCM_tmp)
    LMLG.Cal_vel_AC2LG(temp, temp / 1000, temp_0, DCM_tmp)
    RMLG.Cal_vel_AC2LG(temp, temp / 1000, temp_0, DCM_tmp)
    print("vel is {}, angular vel is {}".format(a, b))
    print("5. Func Cal_vel_AC2LG success")

    temp0 = NLG.Cal_SideSlipAngle(0)
    temp1 = LMLG.Cal_SideSlipAngle()
    temp2 = RMLG.Cal_SideSlipAngle()
    print("Alpha are {},{},{}".format(temp0, temp1, temp2))
    print("6.Func Cal_SideSlipAngle success")

    NLG.alpha = math.pi / 2
    ans = NLG.RotZ()
    print(ans)
    print("7. Func RotZ success")

    ans = NLG.Forces()
    print(ans)
    ans = LMLG.Forces()
    print(ans)
    print("8. Func Forces success")


if __name__ == "__main__":
    test()
