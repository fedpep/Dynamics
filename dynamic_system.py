class DynamicSystem(object):
    def __init__(self,y_0,y_dot_0,dt,law=None):
        self.y=y_0
        self.y_dot=y_dot_0
        self.t=0
        self.dt=dt
        self.law=law
    
    def get_acceleration(self):

        if self.law is None:
            raise Exception('motion law undefined')

        return self.law(self.y,self.y_dot,self.t)

    def update_state(self):
        y_dot_dot=self.get_acceleration()
        for i in range(len(y_dot_dot)): 
            self.y_dot[i]=self.y_dot[i]+self.dt*y_dot_dot[i]
            self.y[i]=self.y[i]+self.dt*self.y_dot[i]
        self.t=self.t+self.dt

    #def get_state(self):
    #    return (self.y,self.y_dot,)

    def get_position(self):
        return self.y

    def get_speed(self):
        return self.y_dot
    
    def get_time(self):
        return self.t
