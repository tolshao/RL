import matplotlib.pyplot as plt
import time 
import Breakenv as Be
import xlwt
class Datamonitor:
    """监视系统信息"""
    def __init__(self,aircraftstate):
        """初始化统计信息"""
        #plt.ion()
        plt.figure(figsize = (7,8))

        self.aircraftstate = aircraftstate

        self.Vplist = []
        self.wlist = []
        self.slip_rationlist = []
        self.wDotlist = []
        self.time = 0
        self.Pblist = []
        self.Pbdotlist = []
        self.u_outlist = []
        self.VpDotlist = []
        self.tag = [] 
        self.vrlist = []

        self.tcount =[]
        
        self.ob1list = []
        self.ob2list = []
        self.ob3list = []
        self.ob4list = []
        self.ob5list = []
        self.ob6list = []
        self.ob7list = []

        self.normalvpflag = 0
        self.sliptest = 0
        self.slippoint = []
        self.vppoint = []
        self.interpoint = 0

        self.data1 = []
        self.data2 = []
        self.data3 = []
        self.data4 = []
        self.data5 = []
        self.data6 = []
        self.data7 = [] 

    def reset_state(self,):
        
        """初始化一些状态"""
        #plt.figure(figsize = (7,8))
        
        self.Vplist = []
        self.wlist = []
        self.slip_rationlist = []
        self.wDotlist = []
        self.time = 0
        self.Pblist = []
        self.Pbdotlist = []
        self.plot_clear()
        self.u_outlist = []
        self.VpDotlist = []
        self.tag = []
        self.vrlist = []

        self.tcount =[]

        self.ob1list = []
        self.ob2list = []
        self.ob3list = []
        self.ob4list = []
        self.ob5list = []
        self.ob6list = []
        self.ob7list = []

        self.normalvpflag = 0
        self.sliptest = 0
        self.slippoint = []
        self.vppoint = []
        self.interpoint = 0

        self.data1 = []
        self.data2 = []
        self.data3 = []
        self.data4 = []
        self.data5 = []
        self.data6 = []
        self.data7 = [] 


        
    def monitor_state(self):
        """监控在运行中可能变化的统计信息"""

        self.tcount.append(self.aircraftstate.T)
        self.Vplist.append(round(self.aircraftstate.Vp,3))
        self.wlist.append(round(self.aircraftstate.w * 0.35,3))
        self.slip_rationlist.append(round(self.aircraftstate.slip_ration,3))
        self.wDotlist.append(round(self.aircraftstate.wDot,3))
        self.Pblist.append(round(self.aircraftstate.pb,3))
        self.u_outlist.append(round(self.aircraftstate.u_out,3))
        self.VpDotlist.append(round(self.aircraftstate.VpDot,3))
        self.tag.append(round(self.aircraftstate.tag,3))
        self.Pbdotlist.append(round(self.aircraftstate.pbdot,3))
        self.vrlist.append(round(self.aircraftstate.vr,3)) 

        self.ob1list.append(round(self.aircraftstate.ob1,3)) 
        self.ob2list.append(round(self.aircraftstate.ob2,3))
        self.ob3list.append(round(self.aircraftstate.ob3,3))
        self.ob4list.append(round(self.aircraftstate.ob4,3))
        self.ob5list.append(round(self.aircraftstate.ob5,3))
        self.ob6list.append(round(self.aircraftstate.ob6,3))
        self.ob7list.append(round(self.aircraftstate.ob7,3))

        

    def plot_ob(self):
        "OB位调试"
        plt.plot(self.tcount,self.ob1list, label="Vp")
        plt.plot(self.tcount,self.ob3list, label="Vd")
        plt.plot(self.tcount,self.ob2list, label="Vs")
        plt.plot(self.tcount,self.ob6list, label="Vi")

        plt.legend()

        plt.figure()

        plt.plot(self.tcount,self.ob5list,label="dw")
        plt.plot(self.tcount,self.ob3list, label="wDot")

        plt.legend()

        plt.figure()

        plt.plot(self.tcount,self.ob4list,label="sr")

        plt.legend()

        #plt.pause(0.02)
        plt.show()
    
    def plot_combination(self):

        plt.plot(self.tcount,self.ob1list, label="dFdsr")
        plt.plot(self.tcount,self.ob2list, label="Fout")
        plt.plot(self.tcount,self.ob3list, label="dsr")

        plt.legend()
        
        plt.figure()
        plt.plot(self.tcount,self.wlist,label="w")
        plt.legend()

        plt.figure()
        plt.plot(self.tcount,self.ob4list, label="dw")
        plt.legend()

        plt.figure()
        plt.plot(self.tcount,self.ob5list,label="Fout")

        plt.legend()

        plt.figure()
        plt.plot(self.tcount,self.ob3list,label="dsr")

        plt.legend()

        plt.figure()
        plt.plot(self.tcount,self.ob6list,label="true_sr")

        plt.legend()

        plt.figure()
        plt.plot(self.tcount,self.ob7list,label="true_dsr")

        plt.legend()

        plt.show()

      
    def plot_data(self):
        """绘制数据曲线"""
        #self.Tlit = range(0,len(self.Vplist),self.aircraftstate.parsettings.Tsc) 
        #fig = plt.figure(1)

        ax1 = plt.subplot(5,1,1)


        ax2 = plt.subplot(5,1,2)

        ax3 = plt.subplot(5,1,3)

        ax4 = plt.subplot(5,1,4)

        ax5 = plt.subplot(5,1,5)
        
        #plt.figure()

        plt.sca(ax1)
        plt.ylim(0,100)
        plt.ylabel('Vw,Vp')
        
        plt.plot(self.wlist,label="w")
        plt.plot(self.Vplist,label="Vp")
        #plt.plot(self.Pblist)
        
        """
        plt.sca(ax2)
        plt.ylabel('slip_ration')
        plt.plot(self.slip_rationlist)
        """

        plt.sca(ax2)
        plt.ylabel('Pb')
        plt.plot(self.Pblist)

        plt.sca(ax3)
        plt.ylabel('u')
        plt.plot(self.u_outlist)

        plt.sca(ax4)
        plt.ylabel('Vpdot')
        plt.plot(self.VpDotlist)

        #plt.plot(self.wDotlist)

        plt.sca(ax5)
        plt.ylabel('slip_rate')
        plt.plot(self.slip_rationlist)

        #plt.cla()
        #plt.clf()
      
        plt.pause(0.01)
       
        #plt.close('all')

    def plot_showdata(self):

        plt.plot(self.tcount,self.wlist,label="w")
        plt.plot(self.tcount,self.Vplist,label="Vp")
        plt.plot(self.tcount,self.Pblist,label="Pb")
        plt.plot(self.tcount,self.vrlist,label="RefV")

        plt.legend()
       
        #plt.pause(0.02)
        plt.show()

      
    def break_timer(self):
        self.time += self.aircraftstate.parsettings.Tsc

    def plot_clear(self):
        plt.cla()
        plt.clf()

    def set_style(self,name,height,bold=False):
        style = xlwt.XFStyle()
        font = xlwt.Font()
        font.name = name
        font.bold = bold
        font.color_index = 4
        font.height = height
        style.font = font
        return style 

    def slipstorage(self,num):
        self.slippoint.append(num)

    def Vppointstorge(self,num):
        self.vppoint.append(num)

    def interpointstorge(self,num):
        self.interpoint = num 

    def Namecomposition(self,pname,dname):

        name = 'dataVp'+ str(pname) +'_d' +str(dname)+'.xls'

        return name 

    def Internamecomposition(self,pname,dname,intername):
        """
        引入干扰名称
        """

        name = 'dataVp'+ str(pname) +'_d' +str(dname)+'inter_pave'+str(intername)+'.xls'

        return name


    def dataextract(self,dataflag,datanum = 2500):   #注意更改250/0.1ms个数字

        

        num = len(self.slip_rationlist)  #环境步长数据长度
        
        #print("Total amount of data is %d"%num)
        #print("flagnunm is %d"%len(dataflag))
        

        for i in range(len(dataflag)):

            tempda1 =[]
            tempda2 =[]
            tempda3 =[]
            tempda4 =[]
            tempda5 =[]
            tempda6 =[]
            tempda7 =[]
            
            
            if dataflag[i] < datanum:

                tempdata1  = self.slip_rationlist[0:dataflag[i]+datanum+1] #只有前半段数据
                tempdata2  = self.Vplist[0:dataflag[i]+datanum+1]
                tempdata3  = self.wlist[0:dataflag[i]+datanum+1]
                tempdata4  = self.VpDotlist[0:dataflag[i]+datanum+1]
                tempdata5  = self.wDotlist[0:dataflag[i]+datanum+1]
                tempdata6  = self.Pblist[0:dataflag[i]+datanum+1]
                tempdata7  = self.Pbdotlist[0:dataflag[i]+datanum+1]

                tempnum = list(range(0,len(tempdata1),50))

                for i in range(len(tempnum)):
                    tempda1.append(tempdata1[tempnum[i]])
                    tempda2.append(tempdata2[tempnum[i]])
                    tempda3.append(tempdata3[tempnum[i]])
                    tempda4.append(tempdata4[tempnum[i]])
                    tempda5.append(tempdata5[tempnum[i]])
                    tempda6.append(tempdata6[tempnum[i]])
                    tempda7.append(tempdata7[tempnum[i]])
                
                #print(tempda7)
                #print("++++++++++")
                
                
            elif num-dataflag[i] < datanum:
                #if dataflag[i]+1 = len(y)-1:
                #    temdata = y[dataflag[i]+1,len(y)-1]

                tempdata1  = self.slip_rationlist[dataflag[i]-datanum:num] #只有前半段数据
                tempdata2  = self.Vplist[dataflag[i]-datanum:num]
                tempdata3  = self.wlist[dataflag[i]-datanum:num]
                tempdata4  = self.VpDotlist[dataflag[i]-datanum:num]
                tempdata5  = self.wDotlist[dataflag[i]-datanum:num]
                tempdata6  = self.Pblist[dataflag[i]-datanum:num]
                tempdata7  = self.Pbdotlist[dataflag[i]-datanum:num]

                tempnum = list(range(0,len(tempdata1),50))

                for i in range(len(tempnum)):
                    tempda1.append(tempdata1[tempnum[i]])
                    tempda2.append(tempdata2[tempnum[i]])
                    tempda3.append(tempdata3[tempnum[i]])
                    tempda4.append(tempdata4[tempnum[i]])
                    tempda5.append(tempdata5[tempnum[i]])
                    tempda6.append(tempdata6[tempnum[i]])
                    tempda7.append(tempdata7[tempnum[i]])
                #print(tempda7)
                #print("#########")

            
                #temdata = y[dataflag[i]-datanum:len(y)]

            else:
     
                #temdata = y[dataflag[i]-datanum:dataflag[i]+datanum+1]

                tempdata1  = self.slip_rationlist[dataflag[i]-datanum:dataflag[i]+datanum+1] #只有前半段数据
                tempdata2  = self.Vplist[dataflag[i]-datanum:dataflag[i]+datanum+1]
                tempdata3  = self.wlist[dataflag[i]-datanum:dataflag[i]+datanum+1]
                tempdata4  = self.VpDotlist[dataflag[i]-datanum:dataflag[i]+datanum+1]
                tempdata5  = self.wDotlist[dataflag[i]-datanum:dataflag[i]+datanum+1]
                tempdata6  = self.Pblist[dataflag[i]-datanum:dataflag[i]+datanum+1]
                tempdata7  = self.Pbdotlist[dataflag[i]-datanum:dataflag[i]+datanum+1]

                tempnum = list(range(0,len(tempdata1),50))

                for i in range(len(tempnum)):
                    tempda1.append(tempdata1[tempnum[i]])
                    tempda2.append(tempdata2[tempnum[i]])
                    tempda3.append(tempdata3[tempnum[i]])
                    tempda4.append(tempdata4[tempnum[i]])
                    tempda5.append(tempdata5[tempnum[i]])
                    tempda6.append(tempdata6[tempnum[i]])
                    tempda7.append(tempdata7[tempnum[i]])


                #print(tempda7)
                #print("********")

            
            self.data1 = self.data1 + tempda1
            self.data2 = self.data2 + tempda2
            self.data3 = self.data3 + tempda3
            self.data4 = self.data4 + tempda4
            self.data5 = self.data5 + tempda5
            self.data6 = self.data6 + tempda6
            self.data7 = self.data7 + tempda7

            #print("datanum is %d"%len(self.data7))



    def data_intermanage(self,datanum = 5000):

        flagdata = self.interpoint 

        num = len(self.slip_rationlist)

        tempda1 =[]
        tempda2 =[]
        tempda3 =[]
        tempda4 =[]
        tempda5 =[]
        tempda6 =[]
        tempda7 =[]

        if num - flagdata < datanum:
         
            tempda1 =  self.slip_rationlist[flagdata:num]
            tempda2 =  self.Vplist[flagdata:num]
            tempda3 =  self.wlist[flagdata:num]
            tempda4 =  self.VpDotlist[flagdata:num]
            tempda5 =  self.wDotlist[flagdata:num]
            tempda6 =  self.Pblist[flagdata:num]
            tempda7 =  self.Pbdotlist[flagdata:num]

            tempnum = list(range(0,len(tempda1),50))

            for i in range(len(tempnum)):
                self.data1.append(tempda1[tempnum[i]])
                self.data2.append(tempda2[tempnum[i]])
                self.data3.append(tempda3[tempnum[i]])
                self.data4.append(tempda4[tempnum[i]])
                self.data5.append(tempda5[tempnum[i]])
                self.data6.append(tempda6[tempnum[i]])
                self.data7.append(tempda7[tempnum[i]])


        

        else:

            tempda1 =  self.slip_rationlist[flagdata:flagdata+datanum]
            tempda2 =  self.Vplist[flagdata:flagdata+datanum]
            tempda3 =  self.wlist[flagdata:flagdata+datanum]
            tempda4 =  self.VpDotlist[flagdata:flagdata+datanum]
            tempda5 =  self.wDotlist[flagdata:flagdata+datanum]
            tempda6 =  self.Pblist[flagdata:flagdata+datanum]
            tempda7 =  self.Pbdotlist[flagdata:flagdata+datanum]

            tempnum = list(range(0,len(tempda1),50))

            for i in range(len(tempnum)):
                self.data1.append(tempda1[tempnum[i]])
                self.data2.append(tempda2[tempnum[i]])
                self.data3.append(tempda3[tempnum[i]])
                self.data4.append(tempda4[tempnum[i]])
                self.data5.append(tempda5[tempnum[i]])
                self.data6.append(tempda6[tempnum[i]])
                self.data7.append(tempda7[tempnum[i]])

    
          


            
        
            

        


    def data_manage(self):
        "Extract data without interference"

    
        if self.slippoint == []:
            """没有打滑点按一定的飞机速度取样"""
            self.dataextract(self.vppoint)

        else:
            """"对打滑点进行记录"""
            self.dataextract(self.slippoint)

    




    
    def write_excel(self, name = "test.xls"):

     
        f = xlwt.Workbook()
        #创建sheet
        sheet1 = f.add_sheet('breakdata_normal',cell_overwrite_ok=True)
        #第一行

        row0 = ["序号","lamada","Vp","w","Vpdot","wdot","Pb","Pbdot","tag"]
        
        #第一列
        colum0 = []
        
        for i in range(len(self.data7)):
            colum0.append(i+1)
        
        
        colum1 = self.data1   #self.slip_rationlist
        #print(colum1)
        colum2 = self.data2   #self.Vplist
        colum3 = self.data3   #self.wlist
        colum4 = self.data4   #self.VpDotlist
        colum5 = self.data5   #self.wDotlist
        colum6 = self.data6   #self.Pblist
        colum7 = self.data7   #self.Pbdotlist
        #colum8 = self.tag 

        colum = [colum0,colum1,colum2,colum3,colum4,colum5,colum6,colum7]
        
        #写第一行
        for i in range(0,len(row0)):
            sheet1.write(0,i,row0[i])
        #写j列
        for j in range(len(colum)):
            for i in range(0,len(colum[j])):
                sheet1.write(i+1,j,colum[j][i])
        #name = name + '.xls'    
        #print(name)
        f.save(name)



if __name__ == "__main__":

    parsetting = Be.Parametersetting()
    aircraft = Be.Aircraft(parsetting)
    datamonitor = Datamonitor(aircraft)
    #datamonitor.plot_data()
    #datamonitor.plot_showdata()






   
    
    