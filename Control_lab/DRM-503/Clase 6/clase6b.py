import control as ct
import numpy as np
import matplotlib.pyplot as plt

m,k,b = 2,0.1,0.2

A = np.array([[0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, -1.7019, 0, 1, 0],
              [0, -13.3301, 0, 1, 0],
              [1, 0, 0, 0, 0]])
B = np.array([[0],
             [0],
             [18.2478],
             [21.1299],
             [0]])
C = np.array([[1, 0, 0, 0, 0],
              [0, 1, 0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, 0, 0, 0, 1]])
D = np.array([[0],[0],[0],[0],[0]])

sys = ct.ss(A, B, C, D)

Q = np.diag([1, 1, 1, 1, 1])
R = np.array([[1]])

K,s,e = ct.lqr(A,B,Q,R)
print(K)

#Controlled System
A_cl = A - B@K
sys_cl = ct.ss(A_cl,B,C,D)
print(A_cl)


x0 = np.array([[0.0],
               [1.0],
               [2.0],
               [3.0],
               [4.0]])
t = np.linspace(0,10,500)

t, y = ct.initial_response(sys_cl, T=t, X0=x0)

plt.plot(t,y[0], label = 'State1')
plt.plot(t,y[1], label = 'State2')
plt.plot(t,y[2], label = 'State3')
plt.plot(t,y[3], label = 'State4')
plt.plot(t,y[4], label = 'State5')
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('LQR Simulation')
plt.legend()
plt.grid()
plt.show()
