from math import *
import random
import numpy as np
from numpy.linalg import solve,det
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.lines as lines
import matplotlib.patches as mpatches
from dynamic_system import *
import threading

b_r1=600.0
b_r2=600.0
b_t1=600.0
b_t2=600.0

RHO1_MAX=30
RHO2_MAX=30

l1=10
l2=10
k1=700.0
k2=k1/2
m1=500.0
m2=m1
g=9.81

RENDERING_INTERVAL=80
    
def spend_law(y,y_dot,t):
    
    rho1,rho1_dot=y[0],y_dot[0]
    theta1,theta1_dot=y[1],y_dot[1]
    rho2,rho2_dot=y[2],y_dot[2]
    theta2,theta2_dot=y[3],y_dot[3]
    
    cost12=cos(theta1-theta2)
    sint12=sin(theta1-theta2)
    
    B=np.array([[m1+m2, 0, m2*cost12,m2*(l2+rho2)*sint12],
                [0,(m1+m2)*(l1+rho1)**2,-m2*(l1+rho1)*sint12,m2*(l1+rho1)*(l2+rho2)*cost12],
                [m2*cost12,-m2*(l1+rho1)*sint12,m2,0],
                [m2*(l2+rho2)*sint12,m2*(l1+rho1)*(l2+rho2)*cost12,0,m2*(l2+rho2)**2]])
    C=np.array([(2*m2*rho2_dot*theta2_dot*sint12-m2*(l2+rho2)*theta2_dot**2*cost12-(m1+m2)*g*cos(theta1)+k1*(l1+rho1)+b_r1*rho1_dot),
                    (m2*cost12*(2*(l1+rho1)*rho2_dot*theta2_dot-(l1+rho1)*rho2_dot*theta1_dot+(l2+rho2)*rho2_dot*theta1_dot)+m2*(l1+rho1)*(l2+rho2)*theta2_dot**2*sint12+2*(m1+m2)*(l1+rho1)*rho1_dot*theta1_dot+(m1+m2)*g*(l1+rho1)*sin(theta1)+b_t1*theta1_dot),
                    (-2*m2*rho1_dot*theta1_dot*sint12-m2*(l1+rho1)*theta1_dot**2*cost12-m2*(l2+rho2)*theta2_dot**2-m2*g*cos(theta2)+k2*(l2+rho2)+b_r2*rho2_dot),
                    (2*m2*(l2+rho2)*rho2_dot*theta2_dot+2*m2*(l2+rho2)*rho1_dot*theta1_dot*cost12-m2*(l1+rho1)*(l2+rho2)*theta1_dot**2*sint12+m2*g*(l2+rho2)*sin(theta2)+b_t2*theta2_dot)])
    #print y,y_dot

    acc_m=solve(B,-C)
    """acc_m[0]=0
    acc_m[2]=0
    """
    #print det(B),acc_m
    return acc_m

    
rho1_0=10
theta1_0=-pi/180*30*0
rho2_0=10
theta2_0=pi/180*10*0


spend=DynamicSystem([rho1_0,theta1_0,rho2_0,theta2_0],[0,0,0,0],0.001,spend_law)

fig, ax = plt.subplots()
plt_range = np.arange(0, 0.5, 0.25)
body1 = plt.Circle((0,0),0.7,color='r')
body2 = plt.Circle((0,0),0.7,color='b')
line1=lines.Line2D([0,0],[0,0],linestyle=':')
line2=lines.Line2D([0,0],[0,0],linestyle=':')
bar1=lines.Line2D([0,0],[0,0],linestyle='-',color='k')
bar2=lines.Line2D([0,0],[0,0],linestyle='-',color='k')
ax.add_line(line1)
ax.add_line(bar1)
ax.add_line(line2)
ax.add_line(bar2)
ax.add_patch(body1)
ax.add_patch(body2)

def computer():
    for k in range(RENDERING_INTERVAL):
        spend.update_state()
            
        if spend.y[0]<0:
            spend.y[0]=0
            spend.y_dot[0]=-spend.y_dot[0]
        elif spend.y[0]>RHO1_MAX:
            spend.y[0]=RHO1_MAX
            spend.y_dot[0]=-spend.y_dot[0]

        if spend.y[2]<0:
            spend.y[2]=0
            spend.y_dot[2]=-spend.y_dot[2]
        elif spend.y[2]>RHO2_MAX:
            spend.y[2]=RHO2_MAX
            spend.y_dot[2]=-spend.y_dot[2]

    

def animate(i):
    #spend.update_state()


    rho1=spend.get_position()[0]
    theta1=spend.get_position()[1]
    rho2=spend.get_position()[2]
    theta2=spend.get_position()[3]
    
    x1=(l1+rho1)*sin(theta1)
    y1=-(l1+rho1)*cos(theta1)
    xb1=l1*sin(theta1)
    yb1=-l1*cos(theta1)
    x2=x1+(l2+rho2)*sin(theta2)
    y2=y1-(l2+rho2)*cos(theta2)
    xb2=x1+l2*sin(theta2)
    yb2=y1-l2*cos(theta2)

    bar1.set_xdata([0,xb1])
    bar1.set_ydata([0,yb1])
    line1.set_xdata([xb1,x1])
    line1.set_ydata([yb1,y1])
    body1.center=(x1,y1)
    bar2.set_xdata([x1,xb2])
    bar2.set_ydata([y1,yb2])
    line2.set_xdata([xb2,x2])
    line2.set_ydata([yb2,y2])
    body2.center=(x2,y2)

    t = threading.Thread(target=computer)
    t.start()
    #print x2,y2
    #print float(i)*RENDERING_INTERVAL#,rho1,rho2
    return line1,bar1,bar2,body1,line2,body2


# Init only required for blitting to give a clean slate.
def init():
    return animate(0)

ani = animation.FuncAnimation(fig, animate, np.arange(1, 30000), init_func=init,
                              interval=RENDERING_INTERVAL)



def onclick(event):
    global cid
    rho1=spend.get_position()[0]
    theta1=spend.get_position()[1]
    rho2=spend.get_position()[2]
    theta2=spend.get_position()[3]
    x1=(l1+rho1)*sin(theta1)
    y1=-(l1+rho1)*cos(theta1)
    x2=x1+(l2+rho2)*sin(theta2)
    y2=y1-(rho2+l2)*cos(theta2)
    
    mx,my=event.xdata,event.ydata
    if abs(mx-x1)<5 and abs(my-y1)<5:
        print 'pressed body1',mx,my
        cid=fig.canvas.mpl_connect('motion_notify_event',ondrag1)

    if abs(mx-x2)<5 and abs(my-y2)<5:
        print 'pressed body2',mx,my
        cid=fig.canvas.mpl_connect('motion_notify_event',ondrag2)
    

cid=None
def onrelease(event):
    global cid
    if cid is not None:
        fig.canvas.mpl_disconnect(cid)
    print 'released'
    
def ondrag1(event):
    theta1=spend.get_position()[1]
    mx,my=event.xdata,event.ydata
    print 'drap',mx,my
    spend.y[0]=sqrt((mx-l1*sin(theta1))**2+(my+l2*cos(theta1))**2)
    spend.y[1]=atan2(mx,-my)
    spend.y_dot[0]=0
    spend.y_dot[1]=0

    
def ondrag2(event):
    rho1=spend.get_position()[0]
    theta1=spend.get_position()[1]
    theta2=spend.get_position()[3]
    x1=(l1+rho1)*sin(theta1)
    y1=-(l1+rho1)*cos(theta1)
    mx,my=event.xdata,event.ydata
    print 'drap',mx,my
    spend.y[2]=sqrt((mx-x1-l2*sin(theta2))**2+(my-y1+l2*cos(theta2))**2)
    spend.y[3]=atan2((mx-x1-l2*sin(theta2)),-(my-y1+l2*cos(theta2)))
    spend.y_dot[2]=0
    spend.y_dot[3]=0
    
fig.canvas.mpl_connect('button_press_event',onclick)
fig.canvas.mpl_connect('button_release_event',onrelease)
plt.xlim((-25,25))
plt.ylim((-40,10))
plt.show()
