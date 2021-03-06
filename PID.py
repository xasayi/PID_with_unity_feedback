## Math behind
# PID CONTROLLER
# u(t) = K_p*e(t) + K_i*int{e(t) dt} + K_d der{de/dt}
# laplace transform: Kp + Ki/s + Kd*s
# r is desired output
# y is actual output
# e is error between r and y
# u is control signal
# K_p is proportional gain
# K_i is the integral gain
# K_d is the derivative gain

# purpose of PID controller
# 1. reduce rise time
# 2. reduce set time
# 3. eliminate steady-state error

# THE PLANT UNITY FEEDBACK
# 1D spring mass damper system
# m = 1 kg
# b = 10 Ns/m
# k = 20 N/m
# ma + bv + kx = F
# laplace transform: ms^2*X + bs*X + k*X = F(s)
# X(s) / F(s) = 1/(ms^2+bs+k) = 1/(s^2+10s+20)

## Defind PID and Plant classes + functions
import control.matlab as matlab
import time

# PID object
class PID:
    # initialize PID
    def __init__(self, Kp = 1, Ki = 1, Kd = 1):
        # gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # transfer function
        self.func = self.transferFunc()

    def changeKp(self, kp):
        # change proportional gain
        self.Kp = kp

    def changeKi(self, ki):
        # change integral gain
        self.Ki = ki

    def changeKd(self, kd):
        # change derivative gain
        self.Kd = kd

    def transferFunc(self):
        # transfer funciton of PID
        s = matlab.tf('s')
        func = self.Kp + self.Ki / s + self.Kd * s
        return func

# plant object
class Damping1D:
    def __init__(self, m = 1, b = 10, k = 20):
        # define variables
        self.m = m
        self.b = b
        self.k = k

        # transfer function
        s = matlab.tf('s')
        func = 1/(m*s**2 + b*s + k)
        self.func = func

# compute feedforward loop transfer function
def total_transfer(func1, func2):
    transfer = func1 * func2 / (1+ func1 * func2)
    return transfer

## TESTING
import matplotlib.pyplot as plt

# placing poles?

loop = PID(300,0,0)
unity = Damping1D()

t = total_transfer(loop.func, unity.func)
print(t)

result = matlab.step(t)

plt.plot(result[1], result[0])
#plt.show()
