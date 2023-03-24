################################################################################
# x：-10~10
# y：不設限
# phi：angle of the car (360度制)，0度~180度，車尾朝正右為0度，車尾朝正左為180度
# theta: steering angle (360度制)，-16度~16度，正使車順時針轉、負使車逆時針轉
# 控制目標：x=0、phi=90度 
################################################################################

import math
def Car_Kinematics_Model(x,y,phi,theta):
    phi=phi/180*math.pi
    theta=theta/180*math.pi
    x=x+math.cos(phi+theta)+math.sin(phi)*math.sin(theta)
    y=y+math.sin(phi+theta)-math.sin(phi)*math.sin(theta)
    phi=phi-math.asin(2*math.sin(theta)/4) # 4為車長
    phi=phi/math.pi*180
    return x,y,phi

import numpy
import skfuzzy.control
import matplotlib.pyplot as plt
def Fuzzy_Controller(V1,V2): # V1：delta_x，V2：delta_phi
    # 定義linquist variable(delta x、delta phi、theta)的universe of discourse
    universe_delta_x=numpy.arange(-10,10,0.1) # delta x #[start, ]stop, [step, ]
    universe_delta_phi=numpy.arange(0,180.1,0.1) # delta phi
    universe_theta=numpy.arange(-16,16.1,0.1) # theta

    # 定義linquist variable (delta x、delta phi、theta)
    delta_x=skfuzzy.control.Antecedent(universe_delta_x,'delta_x')
    delta_phi=skfuzzy.control.Antecedent(universe_delta_phi,'delta_phi')
    theta=skfuzzy.control.Consequent(universe_theta,'theta')

    # 定義delta x的linquist value及其membership function
    delta_x['L']=skfuzzy.trapmf(universe_delta_x,[-10,-10,-5,0])
    delta_x['M']=skfuzzy.trimf(universe_delta_x,[-5,0,5])
    delta_x['R']=skfuzzy.trapmf(universe_delta_x,[0,5,10,10])
    # 定義delta phi的linquist value及其membership function
    delta_phi['L']=skfuzzy.trimf(universe_delta_phi,[0,45,90]) #0~90
    delta_phi['M']=skfuzzy.trimf(universe_delta_phi,[45,90,135])
    delta_phi['R']=skfuzzy.trimf(universe_delta_phi,[90,135,180])
    # 定義theta的linquist value及其membership function
    theta['L']=skfuzzy.trimf(universe_theta,[-16,-8,0])
    theta['M']=skfuzzy.trimf(universe_theta,[-8,0,8])
    theta['R']=skfuzzy.trimf(universe_theta,[0,8,16])
    # 解模糊化方法
    theta.defuzzify_method='centroid' # 重心法

    # 繪membership funcion圖
    #delta_x.view()
    #delta_phi.view()
    #theta.view()
    #plt.show()

    # rule base
    
    rule1=skfuzzy.control.Rule(antecedent=((delta_x['L']&delta_phi['L'])|(delta_x['L']&delta_phi['M'])|(delta_x['M']&delta_phi['L'])),consequent=theta['L'],label='Low')
    rule2=skfuzzy.control.Rule(antecedent=((delta_x['M']&delta_phi['M'])|(delta_x['L']&delta_phi['R'])|(delta_x['R']&delta_phi['L'])),consequent=theta['M'],label='Medium')
    rule3=skfuzzy.control.Rule(antecedent=((delta_x['M']&delta_phi['R'])|(delta_x['R']&delta_phi['M'])|(delta_x['R']&delta_phi['R'])),consequent=theta['R'],label='Hight')

    system=skfuzzy.control.ControlSystem(rules=[rule1,rule2,rule3])
    sim=skfuzzy.control.ControlSystemSimulation(system)
    sim.input['delta_x']=V1
    sim.input['delta_phi']=V2
    sim.compute()
    # theta.view(sim=sim)
    # plt.show()
    return sim.output['theta']

x=-10 # x初始位置
y=10 # y初始位置
phi=90 # phi初始位置

x_pos=[0]*200
y_pos=[0]*200
for t in range(0,200,1):
    theta=Fuzzy_Controller(x-0,phi-90)
    x,y,phi=Car_Kinematics_Model(x,y,phi,theta)
    x_pos[t]=x
    y_pos[t]=y

plt.plot(x_pos,y_pos)
plt.show()


