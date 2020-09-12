# -*- coding: utf-8 -*-
# LG model

# author: tolshao
# 2020-09-11 17:18:49

import numpy as np


class LandingGear(object):
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

    def AC2LG(self, vel, ang_vel, x, dcm):
        assert vel.shape == (1, 3), 'dimension error'
        assert ang_vel.shape == (1, 3), 'dimension error'
        assert x.shape == (1, 3), 'dimension error'
        assert dcm.shape == (3, 3), 'dimension error'
        self.vel = np.cross(ang_vel, self.pos_vec) + vel
        self.x = (np.matmul(dcm.T, self.pos_vec.T)).T + x


class NoseLandingGear(LandingGear):
    def __init__(self, Tsc=0.02, RK4=False):
        super().__init__(Tsc, RK4)
        self.pos_vec = np.matrix([8.5, 0.0, 2.0])


class MainLandingGear(LandingGear):
    def __init__(self, direction, Tsc=0.02, RK4=False):
        super().__init__(Tsc, RK4)
        if direction == 'L':
            self.pos_vec = np.matrix([-1.5, -2, -2])
        elif direction == 'R':
            self.pos_vec = np.matrix([-1.5, 2, 2])
        else:
            raise "please define the LG to the L/R"


def test():
    Tsc = 0.01
    print("testing----------------------------------------------------------------------------------------------------")

    LG = LandingGear()
    vel_tmp = np.matrix([1, 0, 0])
    ang_vel_tmp = np.matrix([0, 0, 0.1])
    x_tmp = np.matrix([0, 0, 0])
    DCM_tmp = np.matrix(np.eye(3, dtype=float))
    print(vel_tmp, ang_vel_tmp, x_tmp, DCM_tmp)

    LG.AC2LG(vel_tmp, ang_vel_tmp, x_tmp, DCM_tmp)
    print(type(LG.vel),  type(LG.x))
    print(LG.vel.shape,  LG.x.shape)
    print("1. Func AC2LG success")

    NLG = NoseLandingGear(Tsc=Tsc)
    print(NLG.pos_vec)
    print("2. inherit NLG success")

    LMLG = MainLandingGear('L')
    print(LMLG.pos_vec)
    print("3. inherit LMLG success")

    RMLG = MainLandingGear('R')
    print(RMLG.pos_vec)
    print("4. inherit RMLG success")


if __name__ == "__main__":
    test()
