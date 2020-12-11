"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""

import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np


class Ac2dEnv(gym.Env):
    """
    Description:
        An aircraft model with lumped parameters, only a linear velocity and an angular velocity considered

    Source:
        Made by Hao, Zhang

    Observation:
        Type: Box(5)
        Num	Observation                     Min             Max
        0	Linear velocity                 -10             10
        1	Angle                           -Inf            Inf
        2	Angular velocity                -24 deg         24 deg
        3	x difference from next point    -Inf            Inf
        4   y difference from next point    -Inf            Inf

    Actions:
        Type: Box(2)
        Num	Action                          Min             Max
        0	Accelerate or decelerate        -0.50           0.5
        1	Steering moment                 -Inf            Inf

        Note: the amount the velocity in X axis that is reduced or increased is not fixed;
        which depends on the yaw angle.
        and the amount of that of angular velocity is depends on the current linear velocity.


    Reward:
        A function of current state variables.

    Starting State:
        position
        velocity            [-0.5, 0.5]
        angle               [-pi/20, -pi/20]
        angular velocity    [-2,2]
        x       computed from the random initial x0 from [-3,3]
        y       computed from the random initial y0 from [-3,3]

    Episode Termination:

        Episode length is greater than 1000.

        Solved Requirements:
        AC reach the destination with ||(x,y)||2 < ||(1, 1)||2
    """

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 20
    }

    def __init__(self, tsc=0.05):
        # constant parameters
        self.tsc = tsc
        self.mass = 9259.  # kg
        self.inertia = 85552.  # kg.m2

        # variables constraint
        self.theta2rad = 12 * 2 * math.pi / 360
        self.x_threshold = 150
        self.y_threshold = 150

        # target point
        self.x_t = 100
        self.y_t = 0

        # variables
        self.x = 0
        self.y = 0
        self.xd = None
        self.yd = None
        self.distance = None
        self.lastDistance = None
        self.rwd_list = None

        # transition matrix
        self.dcm = np.mat(np.eye(2, dtype=np.float32))

        # spaces statement
        # State = (v, phi, phi_dot, x_d^b, y_d^b)
        # Action = (a1, a2) a1 = F/m, a2 - M
        self.obs_high = np.array([20,
                                  math.pi,
                                  math.pi / 20,
                                  200,
                                  200],
                                 dtype=np.float32)
        act_high = np.array([0.5, 0.5], dtype=np.float32)

        self.action_space = spaces.Box(-act_high, act_high, dtype=np.float32)
        self.observation_space = spaces.Box(-self.obs_high,
                                            self.obs_high, dtype=np.float32)

        self.seed()
        self.viewer = None
        self.state = None

        self.steps_beyond_done = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def dcm_mat(self, phi):
        self.dcm = np.array([[np.cos(phi), -np.sin(phi)],
                             [np.sin(phi), np.cos(phi)]])
        return self.dcm

    def reset(self):
        vel, phi, phi_dot, self.x, self.y = self.np_random.uniform(
            low=-0.05, high=0.05, size=(5,)) * self.obs_high

        xd, yd = np.dot(np.mat(self.dcm_mat(-phi)),
                        np.mat([self.x_t - self.x, self.y_t - self.y]).T)
        self.distance = math.sqrt(xd ** 2 + yd ** 2)
        self.lastDistance = self.distance

        self.state = (vel, phi, phi_dot, self.x_t - self.x, self.y_t - self.y)
        self.steps_beyond_done = None
        return np.array(self.state)

    def step(self, action):
        action = np.array(action)
        err_msg = "%r (%s) invalid" % (action, action.shape)
        assert self.action_space.shape == action.shape, err_msg

        # get states
        vel, phi, phi_dot, xd, yd = self.state
        a1, a2 = action

        # compute variables_dot
        vel_dot = a1 / 1  # self.mass
        phi_acc = a2 / vel  # self.inertia / vel  # todo 检查这个角加速度的计算公式
        self.dcm_mat(-phi)  # 计算变换矩阵

        # integral
        self.x = self.x + self.tsc * vel * math.cos(phi)
        self.y = self.y + self.tsc * vel * math.sin(phi)
        vel = vel + self.tsc * vel_dot
        phi_dot = 0.995 * phi_dot + self.tsc * phi_acc
        phi = phi + self.tsc * phi_dot

        # observe
        vec = np.array([self.x_t - self.x, self.y_t - self.y])
        # transformation phi into body coordinate
        xd = np.dot(self.dcm[0], vec)
        yd = np.dot(self.dcm[1], vec)
        self.distance = math.sqrt(xd ** 2 + yd ** 2)
        self.state = (vel, phi, phi_dot, xd, yd)

        # Done
        done = bool(
            self.x < -self.x_threshold
            or self.x > self.x_threshold
            or self.y < -self.y_threshold
            or self.y > self.y_threshold
            or phi < -math.pi
            or phi > math.pi
            or self.distance < 3
        )

        # Reward
        if not done:
            reward = self.reward()

        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            # reward = 1.0
            reward = self.reward()
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0

        return np.array(self.state), reward, done, {}

    def reward(self):
        rwd_distance = 1 if self.lastDistance > self.distance else - 1
        self.lastDistance = self.distance

        rwd_vel = math.exp(np.min([self.state[0], 10]))

        rwd_y = -100*np.arctan(self.state[4] / self.state[3]) ** 2
        # x_log_distance = np.log(np.abs(self.state[3]))
        # rwd_y =  - 0.1 *\
        #     self.state[4] ** 2 / x_log_distance if x_log_distance > 1 else - \
        #     0.05 * self.state[4] ** 2
        reward = rwd_distance + rwd_vel + rwd_y
        self.rwd_list = [rwd_distance, rwd_vel, rwd_y]
        return reward

    def pos(self):
        pos_vec = np.array([self.x, self.y])
        ans = np.concatenate((self.state, pos_vec), axis=0)
        # print(pos_vec[0], pos_vec[1], self.state[0], self.state[1])
        return ans

    def render(self, mode='human'):
        screen_width = 600
        screen_height = 400

        world_width = self.x_threshold * 2
        scale = screen_width / world_width
        carty = 100  # TOP OF CART
        polewidth = 10.0
        polelen = scale * (2 * self.length)
        cartwidth = 50.0
        cartheight = 30.0

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)
            l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
            axleoffset = cartheight / 4.0
            cart = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.carttrans = rendering.Transform()
            cart.add_attr(self.carttrans)
            self.viewer.add_geom(cart)
            l, r, t, b = -polewidth / 2, polewidth / \
                2, polelen - polewidth / 2, -polewidth / 2
            pole = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole.set_color(.8, .6, .4)
            self.poletrans = rendering.Transform(translation=(0, axleoffset))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.carttrans)
            self.viewer.add_geom(pole)
            self.axle = rendering.make_circle(polewidth / 2)
            self.axle.add_attr(self.poletrans)
            self.axle.add_attr(self.carttrans)
            self.axle.set_color(.5, .5, .8)
            self.viewer.add_geom(self.axle)
            self.track = rendering.Line((0, carty), (screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)

            self._pole_geom = pole

        if self.state is None:
            return None

        # Edit the pole polygon vertex
        pole = self._pole_geom
        l, r, t, b = -polewidth / 2, polewidth / \
            2, polelen - polewidth / 2, -polewidth / 2
        pole.v = [(l, b), (l, t), (r, t), (r, b)]

        x = self.state
        cartx = x[0] * scale + screen_width / 2.0  # MIDDLE OF CART
        self.carttrans.set_translation(cartx, carty)
        self.poletrans.set_rotation(-x[2])

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


def test():
    import matplotlib.pyplot as plt
    env = Ac2dEnv()
    print('states:\n', env.reset())
    print('action:\n', env.action_space.__repr__())
    print('obs:\n', env.observation_space.__repr__())
    action = [0, 0]
    data = []
    rwd_list = []
    t = 20
    for _ in range(int(t / env.tsc)):
        state, reward, done, _ = env.step(action)
        tmp = np.concatenate((env.pos(), [reward]), axis=0)
        data.append(tmp)
        rwd_list.append(env.rwd_list)
    # 获取数据plot
    data = np.array(data)
    rwd_list = np.array(rwd_list)
    t = np.linspace(0, t, np.shape(data)[0])
    vel = data[:, 0]
    phi = data[:, 1]
    phi_dot = data[:, 2]
    xd = data[:, 3]
    yd = data[:, 4]
    x = data[:, 5]
    y = data[:, 6]
    reward = data[:, 7]

    plt.figure(figsize=(10, 12))
    plt.subplot(3, 2, 1)
    plt.plot(x, y, label='position of aircraft')
    plt.xlabel('x position')
    plt.ylabel('y position')
    plt.legend(loc='upper right')

    plt.subplot(3, 2, 2)
    plt.plot(xd, yd, label='target position')
    plt.xlabel('x target')
    plt.ylabel('y target')
    plt.legend(loc='upper right')

    plt.subplot(3, 2, 3)
    plt.plot(t, vel, label='velocity')
    plt.xlabel('time')
    plt.ylabel('aircraft velocity')
    plt.legend(loc='upper right')

    plt.subplot(3, 2, 4)
    plt.plot(t, phi_dot, label='$\omega_{phi}$')
    plt.xlabel('time')
    plt.ylabel('angular velocity')
    plt.legend(loc='upper right')

    plt.subplot(3, 2, 5)
    plt.plot(t, phi, label='angle')
    plt.xlabel('time')
    plt.ylabel('angular')
    plt.legend(loc='upper right')

    plt.subplot(3, 2, 6)
    plt.plot(t, rwd_list[:, 0], 'b-', label='distance')
    plt.plot(t, rwd_list[:, 1], 'g', label='vel')
    plt.plot(t, rwd_list[:, 2], 'r', label='angle')
    plt.plot(t, reward, 'black', label='total')
    plt.xlabel('time')
    plt.ylabel('angular velocity')
    plt.legend(loc='upper right')

    plt.savefig('test.pdf')
    # plt.show()


if __name__ == "__main__":
    test()
