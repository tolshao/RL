import math
import numpy as np
import random
class Parametersetting():
    def __init__(self):

        self.g = 9.8 #G

        #路面参数结合系数
        self.b = 14.0326
        self.c = 1.5344
        self.d = 0.8   #最高点                                                   

        #刹车系统参数
        self.Fn = 60000          #载荷
        self.Mp = 18000          #飞机质量  #9000 - 18000
        self.J = 3.5             #克服几轮发散问题，提高机轮惯量
        self.r = 0.35
        self.Kb = 600  

        #噪声均值与方差
        self.roadmiu = 0     #路面噪声
        self.roadsigma = 0.0 

       
        self.wheelmiu = 0    #轮速噪声
        self.wheelsigma = 0  

        self.valvemiu = 0    #伺服阀噪声
        self.valvesigma = 0  

        self.Tsc = 0.0001  #更改为0.0001，克服机轮发散问题

        self.Pilotcommand = 0  #飞行员指令


class AircraftBrake():

    state_dim = 1                         
    action_dim = 1                        

    def __init__(self,parsettings):
        self.parsettings = parsettings
        self.Vp = 80.0   #机速暂时设定为80m/s  70-80
        self.w = self.Vp /self.parsettings.r     #轮速 rad/s
        self.W = self.w * self.parsettings.r  #轮速 m/s
        self.VpDot = 0.0
        self.wDot = 0.0
        self.rwdot = 0.0

        self.pb =  0.0  #刹车指令压力
        self.truepb = 0.0 #真实刹车压力

        self.lastpb = 0.0  #上一时刻刹车压力  
        self.pbdot = 0.0    #刹车压力变化率  输出量

        self.slip_ration = 0.0   #滑移率

        self.Ff = 0.0  #地面摩擦力
        self.u_out = 0.0 #地面摩擦系数 ，魔术方程确定
        
       
        #用以表示总时间
        self.T = 0.0

        #动作空间
        self.action_space = [1,-1]  #实际没用
        self.n_actions = len(self.action_space)

        #self.action = 0

        #状态空间

        self.steps_beyond_done = None

        #判断路轮速监控

        self.Wheel_variant_flag = True #  当轮胎抱死后变为false
        self.Wheel_monitor_flag = True  #打开轮速监控器
        self.w_record_ = 0
        self.Wheel_flag_number = 0

        #观测以识别
        self.ob = []

        self.tag = 0

        #pbm控制

        self.ar = 2.5 #参考减速率

        self.Ts_pbm = 0.005  #控制周期

        self.D_Vpt = 2.5
        self.D_Vdt = 6.0
        self.D_Vit1 = 2.5
        self.D_Vit2 = 10
        
        #PBM变量
        self.vr = 0 #参考速度
        self.wr_error = 0 #轮速差记录 m/s

        self.breakflag = 0 #第一次控制赋予参考速度标志

        self.temperror_vw = 0.0
        self.Vi_1 = 0.0
        self.NumAbs = 0

        #注意PBM参数随时调整
        self.Kp = 2.3
        self.Kd = 0
        self.ki_increase = 0.09
        self.ki_decreasepro = 7
        self.ki_decrease = 5

        #伺服阀参数
        self.valve_u = [0.0,0.0,0.0]
        self.valve_y = [0.0,0.0,0.0]
        

        #Self_optpbm参数
        self.Difftemp = [200.0,-1.0,0.0,0.0]

        self.Diffsr = [0.0,0.0]

        #self.Diffw = [0.0,0.0]


        self.PressureCmd = 0

      
        self.startoptimal = 0  #是否进行寻优操作标志

        self.CtrlCount = 0 
        
        self.count = 0 

        self.MaxPointdected = 0  #最大结合力检测标志
        self.OptimalstartPre = 0  #??????????????

        self.firstflag = 0   #第一次
        self.Fout = 0 

        self.TarDiffV = -30

        self.PlaneVelocity = 0.1 #g估计的飞机速度

        self.Sumerror = 0

        self.MaxPressure = 0

        self.pressureintergration = 0

        self.maxsr = 0

        self.RefV = 0
        
        self.SlipFlag = 0

        self.FirstSkipPressure = 0

        self.Recoverflag  = 0

        self.SpeedError = 0 #没用

        self.ob1 = 0.0
        self.ob2 = 0.0
        self.ob3 = 0.0
        self.ob4 = 0.0
        self.ob5 = 0.0
        self.ob6 = 0.0
        self.ob7 = 0.0

        #控制噪声计数

        self.roadcount = 0
        self.wheelcount = 0
        self.valvecount = 0

        #PPO

        self.ppogetpoint = False
        self.pporcount = 0
        self.ppoflag = False
        
        

    def step(self,action):   #刹车控制调用函数
        """"采取的动作"""
        #self.action = action
        
        #print(action)
        #self.lastpb = self.pb
        #self.pb = self.pb + action*self.parsettings.Tsc

        #self.pb = self.Pbm_control(action)

        if not self.ppoflag:

            self.pb = self.Self_optpbm(action,self.Vp)


        else:

            self.pb = action + self.opp_pd(self.parsettings.Pilotcommand)

            s = self.get_state()
            print("$$$$$$$$$")
            print(s)
        
            r = self.get_reward()
            print("$$$$$$$$$")
            print(r)

            return s,r,self.ppogetpoint

    def get_state(self):
        
        #pb_state = np.array(int(self.pb/0.5)*0.5)
        #print("*****")
        #print(pb_state)
        #print("#####")
        temp = self.slip_ration
     
        # print("testate:")

        #print(temp)
        
        np.set_printoptions(precision=2)
        sliprationstate = np.array([temp])
        #print(sliprationstate)
        #print("temp is %f"%sliprationstate)

        #print("getstate:")


        #print(sliprationstate)

        return np.hstack([sliprationstate])


    def get_reward(self):
        #达到maxsr奖励1 

        t =100

        r = 0
        
        distance = self.maxsr - self.slip_ration

        r = -(self.maxsr - self.slip_ration)*100
        

        if  distance > 0 and (not self.ppogetpoint):

            r += 0.5


            if distance < 0.05 :
                
                self.pporcount += 1

                r += 10

                if self.pporcount > t:
                    r += 20
                    
                    if self.Vp < 10:

                        self.ppogetpoint =True

        else:
            self.pporcount = 0
            r -=5
            self.ppogetpoint = False

        #print(self.slip_ration)

        return  r

    def rl_reset(self):

        self.ppogetpoint =False
        self.pporcount = 0

        return self.get_state()





       
    def env_reset(self):
        """episode重置"""
        self.Vp = 80.0   #机速暂时设定为80m/s  70-80
        self.w = self.Vp/self.parsettings.r     #轮速 rad/s
        self.W = self.w * self.parsettings.r
        self.VpDot = 0.0
        self.wDot = 0.0
        self.rwdot = 0.0

        self.pb =  0.0  #初始刹车压力
        self.truepb = 0.0 #真实刹车压力

        self.lastpb = 0.0
        self.pbdot = 0.0

        self.slip_ration = 0.0
 

        self.Ff = 0.0 #地面摩擦力
        self.u_out = 0.0 #地面摩擦系数 ，魔术方程确定 

        #用以表示总时间
        self.T = 0.0

        #动作空间
        self.action_space = [1,-1]  #实际没用
        self.n_actions = len(self.action_space)

        #self.action = 0

        #状态空间

        self.steps_beyond_done = None

        #判断路轮速监控

        self.Wheel_variant_flag = True #  当轮胎抱死后变为false
        self.Wheel_monitor_flag = True  #打开轮速监控器
        self.w_record_ = 0
        self.Wheel_flag_number = 0

        #观测以识别
        self.ob = []

        self.tag = 0

        #pbm控制

        self.breakflag = 0 #第一次控制赋予参考速度标志

        self.vr = 0 #参考速度
        self.wr_error = 0 #轮速差记录 m/s
        
        self.ar = 2.5 #参考减速率

        self.Ts_pbm = 0.005  #控制周期

        self.D_Vpt = 2.5
        self.D_Vdt = 6.0
        self.D_Vit1 = 2.5
        self.D_Vit2 = 10

        self.temperror_vw = 0.0
        self.Vi_1 = 0.0
        self.NumAbs = 0

        #注意PBM参数随时调整
        self.Kp = 2.3
        self.Kd = 0
        self.ki_increase = 0.09
        self.ki_decreasepro = 7
        self.ki_decrease = 5

        #伺服阀参数
        self.valve_u = [0,0,0]
        self.valve_y = [0,0,0]

        #Self_optpbm参数
        self.Difftemp = [200.0,-1.0,0.0,0.0]

        self.Diffsr = [0.0,0.0]

        #self.Diffw = [0.0,0.0]

        self.PressureCmd = 0

        self.startoptimal = 0  #是否进行寻优操作标志

        self.CtrlCount = 0 
        
        self.count = 0 

        self.MaxPointdected = 0  #最大结合力检测标志
        self.OptimalstartPre = 0  #??????????????

        self.firstflag = 0   #第一次
        self.Fout = 0 

        self.TarDiffV = -30

        self.PlaneVelocity = 0.1 #g估计的飞机速度

        self.Sumerror = 0

        self.MaxPressure = 0

        self.pressureintergration = 0

        self.maxsr = 0

        self.RefV = 0
        
        self.SlipFlag = 0

        self.FirstSkipPressure = 0

        self.Recoverflag  = 0

        self.SpeedError = 0 #没用

        self.ob1 = 0.0
        self.ob2 = 0.0
        self.ob3 = 0.0
        self.ob4 = 0.0
        self.ob5 = 0.0
        self.ob6 = 0.0
        self.ob7 = 0.0

        #控制噪声计数

        self.roadcount = 0
        self.wheelcount = 0
        self.valvecount = 0

        #PPO检测

        self.ppogetpoint = False
        self.pporcount = 0
        self.ppoflag = False


    def Self_optpbm(self,command,true_vp):
        """
        自适应PBM
        """

        kbc = 1000

        if self.breakflag == 0:  #做第一次参考速度设定

            self.RefV = self.w
            self.breakflag = 1
        
        if command - self.PressureCmd > 1:             #

            self.PressureCmd = self.PressureCmd + 0.03      #每个周期增加0.03
            
            self.startoptimal = 1  #寻优标志

            self.CtrlCount = 0 #

            if (self.count < 10):  #寻优次数
                self.count = self.count + 1

            if self.count == 1:   #第一次寻优进行初始化
                self.firstflag = 0     #第一次寻优标志

                self.MaxPointdected = 0  #最大结合力检测标志
                self.OptimalstartPre = self.Fout

                self.SumError = 0
                self.TarDiffV = -30
                self.PressureCmd = self.Fout 

        elif command >= self.PressureCmd:  #飞行员没给指令
            #print("flag")

            self.PlaneVelocity = self.RefV * 0.35 * 1.14 #******************* 1.14
            

            if command - self.PressureCmd < 0.2: #当达到0.2时停止寻优
                self.startoptimal = 0  #寻优标志置0，停止寻优

            self.count = 0  

        else:  #刹车压力超过飞行员指令

            self.PressureCmd = command
            self.startoptimal = 0  #寻优标志置0，停止寻优
            self.count = 0

        if command < 0.5 or command < self.Fout:

            self.startoptimal = 0

            self.firstflag = 0

            self.Sumerror = 0

            self.Recoverflag = 0

            self.MaxPointdected = 0  #检测到标志置0

            self.TarDiffV = -30


        #对轮速进行微分跟踪器操作
        dw = self.Diff_wheel(0.005,2000000,self.w)
        #print(dw)

        F = (dw * self.parsettings.J + self.Fout * self.parsettings.Kb)/self.parsettings.r   #地面摩擦力估计  Fout * kbc刹车力矩

        dF = self.Diff_F(0.005,2000000,F)

        dsr = ((-0.05 * self.Fout /self.parsettings.r) - (dw * self.parsettings.r))/self.PlaneVelocity   #滑移率的微
        #print("self.Fout:")
        #print(self.Fout)
        #print("dw:")
        #print(dw)
        #print("PlaneVelocity")
        #print(self.PlaneVelocity)
    

        if dsr == 0:
            dsr = 0.0000001

        dFdsr = dF/dsr
        if dFdsr > 1000000:
            dFdsr = 1000000

        if dFdsr < -1000000:
            dFdsr = -1000000

        if self.startoptimal == 1:  #开始寻优
            if (dFdsr < 10000) and (self.firstflag == 0) and (self.PressureCmd - 2.0 > self.OptimalstartPre):
                self.MaxPointdected = 1
                self.MaxPressure = self.Fout - 0.3
                self.firstflag = 1
                if self.Recoverflag == 1:
                    self.Recoverflag = 0
                
                self.SumError = -self.SpeedError * 0.15  #????????
                self.startoptimal = 0

                self.maxsr = 1 - self.w * self.parsettings.r / true_vp   #Vph 和 PlaneVelocity有什么区别0.01 

                Vi = command - self.MaxPressure 
                self.Vi_1 = Vi

            self.pressureintergration = self.pressureintergration + self.Fout *0.005
        else:
            if self.firstflag == 0:
                self.Maxpressure = self.PressureCmd


        error_vw = self.RefV - self.w


        if self.MaxPointdected == 1:
           
            self.RefV = true_vp * (1-self.maxsr)/self.parsettings.r + self.D_Vit1 # +0.05会致使

        else:
            self.MaxPressure = self.PressureCmd
            self.RefV = true_vp * (1-0.08) / self.parsettings.r
  

        if self.RefV < 0:
            self.RefV = 0
        
        self.vr = self.RefV * self.parsettings.r  #(可观察)
        
        if error_vw > 15:      #打滑判定
            if (self.SlipFlag == 0):
                self.FirstSkipPressure = self.Fout

            self.SlipFlag = 1

        if error_vw <= 0:
            error_vw = 0
        
        if self.MaxPointdected == 1:
           
            temperror_Vp = error_vw - self.D_Vpt

            if temperror_Vp > 0:
                Vp = temperror_Vp * self.Kp 
            else:
                Vp = 0
            
            temperror_Vd = error_vw - self.D_Vdt

            if temperror_Vd > 0:
                Vd = (error_vw - self.temperror_vw) * self.Kd / self.Ts_pbm
            else:
                Vd = 0

            if Vd < 0:
                Vd = 0

            self.temperror_vw = error_vw   #记录上一周期轮速

            temperror_Vi = error_vw - self.D_Vit1  #浅打滑判据

            if (temperror_Vi <= 0):
                self.NumAbs = self.NumAbs + 1
                Vi = self.Vi_1 - (self.Ts_pbm * self.Ts_pbm * self.NumAbs * self.NumAbs * self.ki_increase)

            elif( temperror_Vi > self.D_Vit2 ):
                self.NumAbs = 0
                Vi = self.Vi_1 + (self.Ts_pbm * self.ki_decreasepro)
                self.Vi_1 = Vi

            else:
                self.NumAbs = 0
                Vi = self.Vi_1 + (self.Ts_pbm * temperror_Vi * self.ki_decrease)
                self.Vi_1 = Vi

            if (self.Vi_1 > command ):
                self.Vi_1 = command

            if (Vi > command ):
                Vi = command 

            if Vi < 0:
                Vi = 0
                self.Vi_1 = 0
                self.NumAbs = 0

            Vp_pbm = Vp 
            Vd_pbm = Vd
            Vi_pbm = Vi 
              

            self.ob1 = Vp_pbm
            self.ob3 = Vd_pbm
           
            self.ob6 = Vi_pbm

            

            Vs = Vp + Vd + Vi
            Vs_pbm = Vs

            self.ob2 = Vs_pbm

            if Vs < 0:
                Vs = 0
            if Vs > command:
                Vs = command
            
            self.Fout = command - Vs
        else:
            self.Fout = self.MaxPressure

        if self.Fout > command:
            self.Fout = command

        Pb = self.Fout

        return Pb
    
    #占用全局变量
    
    def Diff_wheel(self,h,r,Whespd):  #轮速微分
        delta = h*r
        deltal = h*delta
        ek = self.Difftemp[0] - Whespd
        z1 = ek + h * self.Difftemp[1]
        if abs(z1) > deltal:
            gk = self.Difftemp[1] - self.sign(z1) * (delta - math.sqrt(delta *delta + 8* r * abs(z1))/2)
        else:
            gk = self.Difftemp[1] + z1/h

        if abs(gk) >= delta:
            fst = -r * self.sign(gk)
        else:
            fst = -r *gk /delta

        self.Difftemp[0] = self.Difftemp[0] + self.Difftemp[1] * 0.005
        self.Difftemp[1] = self.Difftemp[1] + fst * 0.005

        return self.Difftemp[1]

    def Diff_F(self,h,r,F):   #刹车压力微分
        delta = h*r
        deltal = h*delta
        ek = self.Difftemp[2] - F
        z1 = ek + h * self.Difftemp[3]
        if abs(z1) > deltal:
            gk = self.Difftemp[3] - self.sign(z1) * (delta - math.sqrt(delta *delta + 8* r * abs(z1))/2)
        else:
            gk = self.Difftemp[3] + z1/h

        if abs(gk) >= delta:
            fst = -r * self.sign(gk)
        else:
            fst = -r *gk /delta

        self.Difftemp[2] = self.Difftemp[2] + self.Difftemp[3] * 0.005  #时间延迟
        self.Difftemp[3] = self.Difftemp[3] + fst * 0.005
        return self.Difftemp[3]

    def Diff_sr(self,h,r,sr):   #路面dsr
        delta = h*r
        deltal = h*delta
        ek = self.Diffsr[0] - sr
        z1 = ek + h * self.Diffsr[1]
        if abs(z1) > deltal:
            gk = self.Diffsr[1] - self.sign(z1) * (delta - math.sqrt(delta *delta + 8* r * abs(z1))/2)
        else:
            gk = self.Diffsr[1] + z1/h

        if abs(gk) >= delta:
            fst = -r * self.sign(gk)
        else:
            fst = -r *gk /delta

        self.Diffsr[0] = self.Diffsr[0] + self.Diffsr[1] * 0.001  #时间延迟
        self.Diffsr[1] = self.Diffsr[1] + fst * 0.001
        return self.Diffsr[1]

    
    """
    def Diff_w(self,h,r,w):   #用于记录轮速微分
        delta = h*r
        deltal = h*delta
        ek = self.Diffw[0] - w
        z1 = ek + h * self.Diffw[1]
        if abs(z1) > deltal:
            gk = self.Diffw[1] - self.sign(z1) * (delta - math.sqrt(delta *delta + 8* r * abs(z1))/2)
        else:
            gk = self.Diffw[1] + z1/h

        if abs(gk) >= delta:
            fst = -r * self.sign(gk)
        else:
            fst = -r *gk /delta

        self.Diffw[0] = self.Diffw[0] + self.Diffw[1] * 0.0001  #时间延迟
        self.Diffw[1] = self.Diffw[1] + fst * 0.0001
        return self.Diffw[1]
    """


    def sign(self,x):
        """
        符号函数
        """
        if x > 0:
            return 1
        else:
            if x == 0:
                return 0
            return -1


    def update(self):
        """
        环境
        """
        self.T += self.parsettings.Tsc   #环境时间控制参数  

        self.road_respond()  
   
        self.valve_respond()   #伺服阀响应输出刹车压力 self.valve_y[0]

        self.aircraft_Wheelrespond()  #飞机机轮响应

        self.aircraft_respond()   #飞机响应


    def road_respond(self):
        """
        道路响应
        """

        self.slip_ration = (self.Vp - self.w * self.parsettings.r) / self.Vp

        

        #self.ob7 = self.Diff_sr(0.001,2000000,self.slip_ration)   #路面实际滑移率

        self.u_out =  self.parsettings.d * \
        math.sin( self.parsettings.c * math.atan( self.parsettings.b * self.slip_ration))    #结合系数
        """
        if (self.w * self.parsettings.r) < self.Vp:
            self.Ff = self.u_out * self.parsettings.Fn         
        else:
            self.Ff = 0
        """
        #print(self.u_out + random.gauss(self.parsettings.miu,self.parsettings.sigma))

        self.roadcount = self.roadcount +1

        if self.MaxPointdected == 1 and self.roadcount == 5:

            self.Ff = (self.u_out + random.gauss(self.parsettings.roadmiu,self.parsettings.roadsigma)  )\
                * self.parsettings.Fn

            self.roadcount = 0
        # 加入噪声后使控制压力难以上升
        else:

            self.Ff = self.u_out  * self.parsettings.Fn
        

    def aircraft_Wheelrespond(self):
        """
        机轮响应
        """

        self.wheelcount = self.wheelcount + 1

        self.wDot = (self.parsettings.r * self.Ff - self.parsettings.Kb * self.truepb ) / self.parsettings.J  #机轮减速率    Tb为刹车力矩=Kb*pb

        if self.MaxPointdected == 1 and self.wheelcount == 5:

            self.w = self.w + self.wDot * self.parsettings.Tsc + \
                random.gauss(self.parsettings.wheelmiu,self.parsettings.wheelsigma)

            self.wheelcount = 0

        else:

             self.w = self.w + self.wDot * self.parsettings.Tsc


        if self.w <= 0:
            self.w = 0

        if self.w * self.parsettings.r > self.Vp:
            self.w = self.Vp / self.parsettings.r

        if self.w < 0:
            self.w = 0.01

        #self.rwdot = self.Diff_w(0.005,2000000,self.w) #用于轮速采集
        self.wr_error = (self.RefV-self.w) * self.parsettings.r 

    def aircraft_respond(self):
        """
        飞机响应
        """
        self.VpDot = -2 * self.Ff / self.parsettings.Mp         #仅考虑双主轮阻力
    
        self.Vp = self.Vp + self.VpDot * self.parsettings.Tsc    #飞机速度变化
      
        if self.Vp <=0:
            self.Vp = 0.00001
            

    def valve_respond(self):
        "伺服阀响应"

        self.valve_u[0] = self.pb

        self.valve_u[2] = self.valve_u[1]

        self.valve_u[1] = self.valve_u[0]
        

        self.valve_y[0] = self.valve_y[1]*1.934 - self.valve_y[2]*0.9418\
             + self.valve_u[1]*0.003845+self.valve_u[2]*0.003769

        self.valve_y[2] = self.valve_y[1]   
         
        self.valve_y[1] = self.valve_y[0]

        self.valvecount = self.valvecount + 1

        if self.MaxPointdected == 1 and self.valvecount == 5:

            self.truepb = self.valve_y[0] + random.gauss(self.parsettings.valvemiu,self.parsettings.valvesigma) 

            self.valvecount = 0

        else:

            self.truepb = self.valve_y[0]

    def opp_pd(self,command):

        error_vw = self.RefV - self.w

        if error_vw <= 0:
            error_vw = 0

        temperror_Vp = error_vw - self.D_Vpt

        if temperror_Vp > 0:
            Vp = temperror_Vp * self.Kp 
        else:
            Vp = 0
        
        temperror_Vd = error_vw - self.D_Vdt

        if temperror_Vd > 0:
            Vd = (error_vw - self.temperror_vw) * self.Kd / self.Ts_pbm
        else:
            Vd = 0

        if Vd < 0:
            Vd = 0

        Vs = Vp + Vd 
           

        if Vs < 0:
            Vs = 0
        if Vs > command:
            Vs = command
    
        self.Fout = command - Vs

        

        if self.Fout > command:
            self.Fout = command

        return self.Fout

    

if __name__ == "__main__":    #无干扰环境

    #Setting global variables

   

    #Configuration Environment

    parsetting = Parametersetting()
    env =  AircraftBrake(parsetting)


    env.step(1)


   


