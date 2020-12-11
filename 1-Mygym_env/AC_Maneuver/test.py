from Aircraft import Aircraft
import numpy as np
from LG import LandingGear, NoseLandingGear, MainLandingGear
import math
time_step = 0.01


def test():

    print("-------------------------------------Testing AC----------------------------------")

    x0 = [0.0, 0.0, 0.0]
    vel0 = [1.0, 2.0, 3.0]
    euler0 = [0.1, 0.2, 0.3]
    ang_vel0 = [0.1, 0.2, 0.3]

    initial_parameters = [x0, vel0, euler0, ang_vel0]
    ac = Aircraft(initial_parameters, Tsc=0.01, RK4=False)
    forces = [1, 1, 1]
    forces = [0, 0, 0]
    moments = [0, 0, 0]
    print(ac.RK4)
    print(ac.vel, ac.ang_vel)
    print(ac.DotAngVel_get())
    ac.update_onestep(forces, moments)
    print(ac.vel, ac.ang_vel)


def test2():
    # 测试函数
    Tsc = 0.01
    print("-------------------------------------Testing LG-------------------------------------------------")

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

    fourin1 = [temp, temp / 1000, temp_0, DCM_tmp]

    # ans = NLG.Forces(fourin1)
    ans = LMLG.Forces()
    print(ans)


if __name__ == "__main__":
    test2()
