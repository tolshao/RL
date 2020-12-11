import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np


class AC_GroundManeuver(gym.Env):
    """
    简述：


    """
    metadata = {
        'render.modes': ['human']  # , 'rgb_array'],
        #		'video.frames_per_second': 50
    }

    def __init__(self):
        # env_para
        self.step_count = 0
        self.min_state =
        self.max_state =
        self.min_action =
        self.max_action =

        self.action_space = spaces.Box(low, high[, shape[, dtype]])
        self.observation_space = spaces.Box(low, high[, shape[, dtype]])
        self.state = []

        # enviroment_para
        self.g = 9.81  # gravity, m/s2
        self.rad2phi = 57.29577951308232

        # AC_para
        self.m_ac = 9295.44  # 质量 kg
        self.b = 9.144  # 翼展 m
        self.S_ref = 27.87  # 参考面积 m2
        self.c = 3.45  # 平均气动弦长 m

        self.Fz_gravity = [0 0 m_ac * g]  # 重力，N
        # I_ac = eye(3)*100000;
        self.Ixx = 12874.8
        self.Iyy = 75673.6
        self.Izz = 85552.1
        self.Ixz = 1331.4
        self.Ixy = 0
        self.Iyz = 0
        self.I_ac = [Ixx - Ixy - Ixz
                     - Ixy Iyy - Iyz
                     - Ixz - Iyz Izz]

        # CG
        self.l_xN = 8.5  # length, m
        self.l_yN = 0  # length, m
        self.l_zN = 2  # length, m
        self.l_relative_N = [l_xN l_yN l_zN]
        self.l_xMl = -1.5  # length, m
        self.l_yMl = -2  # length, m
        self.l_zMl = 2  # length, m
        self.l_relative_Ml = [l_xMl l_yMl l_zMl]
        self.l_xMr = -1.5  # length, m
        self.l_yMr = 2  # length, m
        self.l_zMr = 2  # length, m
        self.l_relative_Mr = [l_xMr l_yMr l_zMr]

        # LG
        self.m_tire = 100
        self.c_oil_N = 600000  # NLG液压阻尼
        self.c_oil = 800000  # MLG液压阻尼

        self.k_tire_N = 157741  # 前轮胎刚度N/m
        self.k_tire = 877200  # 轮胎刚度N/m
        self.k_s_M = 126200  # 主起落架刚度N/m
        self.k_s_N = 50600  # 前起落架刚度N/m

        # para_friction
        self.k_alpha2mu_y = 0.38  # 角度侧向摩擦力系数

    def reset(self):
        self.step_count = 0

        # initial_condition
        self.Vm_0 = [30 0 0]  # 初始速度
        self.pm_0 = [0 0 30]  # 初始角速度，°/s
        self.eul_0 = [0 0 0]  # 初始角度
        self.xme_0 = [0 0 0-l_zN]  # 初始地球坐标系位置

        self.state =
        return self.state

    def step(self, action):

        reward =
        done =
        self.state =
        return np.array(self.state), reward, done, {}

    def close(self):
        print('{}'.format(self.step_count))

    def compu_angle(self):
        code
