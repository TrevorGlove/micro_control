# Elaborado por Robert Vásquez @TrevorGlove
  
import ulab.numpy as np

class PIDz:
    def __init__(self, Ts=None):
        self.reset()
        self._Kp = 0
        self._Ki = 0
        self._Kd = 0
        self.Ts = Ts
        self.update()

    @property
    def Kp(self):
        return self._Kp

    @Kp.setter
    def Kp(self, value):
        self._Kp = value
        self.update()

    @property
    def Ki(self):
        return self._Ki

    @Ki.setter
    def Ki(self, value):
        self._Ki = value
        self.update()

    @property
    def Kd(self):
        return self._Kd

    @Kd.setter
    def Kd(self, value):
        self._Kd = value
        self.update()

    def update(self): 
        self.K = np.array([self.Kp, self.Ki, self.Kd])
        self.q = np.array([self.K[0] + self.K[1]*self.Ts/2 + self.K[2]/self.Ts,  
                           -self.K[0] + self.K[1]*self.Ts/2 - 2*self.K[2]/self.Ts,
                           self.K[2]/self.Ts
                           ])

    def step(self, e0):
        self.e[0] = e0
        self.c[0] = self.c[1] + np.dot(self.e, self.q)   # PID Discrete
        self.c[1:], self.e[1:] = self.c[:-1], self.e[:-1]
        
        return self.c[0]

    def reset(self):
        self.e = np.zeros(3)
        self.c = np.zeros(3)
