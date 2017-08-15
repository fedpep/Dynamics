from math import *
import random
import numpy as np
from numpy.linalg import inv,det,solve
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.lines as lines
import matplotlib.patches as mpatches
from dynamic_system import *
import threading

RENDERING_INTERVAL=60

theta1_0=pi/180*90
theta2_0=pi/180*180
rho1=8
rho2=10
b_t1=2300.0*0
b_t2=2300.0*0
m1=1900.0
m2=1900.0
g=9.81

def spend_law(y,y_dot,t):

    
    theta1,theta1_dot=y[0],y_dot[0]
    theta2,theta2_dot=y[1],y_dot[1]

    ext_torque=0
    """if t>5 and t<5.014:
        print 'trq'
        ext_torque=0.000001
    """
    B=np.array([[(m1+m2)*rho1**2,m2*rho1*rho2*cos(theta1-theta2)],
                [m2*rho1*rho2*cos(theta1-theta2),m2*rho2**2]])
    C=np.array([-(m2*rho1*rho2*theta2_dot**2*sin(theta1-theta2)+(m1+m2)*g*rho1*sin(theta1)+b_t1*theta1_dot-ext_torque),
                    -(-m2*rho1*rho2*theta1_dot**2*sin(theta1-theta2)+m2*g*rho2*sin(theta2)+b_t2*theta2_dot)])
    #print t,y,y_dot

    acc_m=solve(B,C)

    return acc_m





#t=[0]
double_pendulum=[DynamicSystem([theta1_0,theta2_0],[0,0],0.001,spend_law),DynamicSystem([theta1_0+0.01,theta2_0],[0,0],0.001,spend_law)]

fig, ax = plt.subplots()
plt_range = np.arange(0, 0.5, 0.25)

def computer():
    for k in range(RENDERING_INTERVAL):
        for i in range(2):
            double_pendulum[i].update_state()


def animate(n):
    
    x1=[0,0]
    y1=[0,0]
    x2=[0,0]
    y2=[0,0]
    theta1=[0,0]
    theta2=[0,0]
    for i in range(2):
        theta1[i]=double_pendulum[i].get_position()[0]
        theta2[i]=double_pendulum[i].get_position()[1]
        x1[i]=rho1*sin(theta1[i])
        y1[i]=-rho1*cos(theta1[i])
        x2[i]=x1[i]+rho2*sin(theta2[i])
        y2[i]=y1[i]-rho2*cos(theta2[i])

        line1[i].set_xdata([0,x1[i]])
        line1[i].set_ydata([0,y1[i]])
        body1[i].center=(x1[i],y1[i])
        line2[i].set_xdata([x1[i],x2[i]])
        line2[i].set_ydata([y1[i],y2[i]])
        body2[i].center=(x2[i],y2[i])

    t = threading.Thread(target=computer)
    t.start()
    
    print float(n)*RENDERING_INTERVAL
    return line1[0],body1[0],line2[0],body2[0],line1[1],body1[1],line2[1],body2[1]


# Init only required for blitting to give a clean slate.
def init():
    global body1,body2,line1,line2
    body1=[]
    body2=[]
    line1=[]
    line2=[]

    cols_1=['r','b']

    for i in range(2):
        
        body1 += [plt.Circle((0,0),0.7,color=cols_1[i])]
        body2 += [plt.Circle((0,0),0.7,color=cols_1[i])]
        line1+=[lines.Line2D([0,0],[0,0],linestyle='-.')]
        line2+=[lines.Line2D([0,0],[0,0],linestyle='-.')]
        
        ax.add_line(line1[-1])
        ax.add_line(line2[-1])
        ax.add_patch(body1[-1])
        ax.add_patch(body2[-1])

    return animate(0)

init()
ani = animation.FuncAnimation(fig, animate, np.arange(1, 3000),
                              interval=RENDERING_INTERVAL, blit=False)


    
plt.xlim((-20,20))
plt.ylim((-20,20))
plt.show()
