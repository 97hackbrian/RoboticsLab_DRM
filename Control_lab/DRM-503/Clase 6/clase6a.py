import control as ct
import numpy as np
import matplotlib.pyplot as plt

m,k,b = 2,0.1,0.2

A = np.array([[0 , 1],
             [-k/m, -b/m]])
B = np.array([[0],
             [1]])
C = np.array([[1, 0],[0, 1]])
D = np.array([[0],[0]])

sys = ct.ss(A, B, C, D)

Q = np.array([[1, 0],
              [0, 1]])
R = np.array([[1]])

K,s,e = ct.lqr(A,B,Q,R)
print(K)

#Controlled System
A_cl = A - B@K
sys_cl = ct.ss(A_cl,B,C,D)
print(A_cl)


x0 = np.array([[2.0],
               [0.0]])
t = np.linspace(0,10,500)

t, y = ct.initial_response(sys_cl, T=t, X0=x0)

plt.plot(t,y[0], label = 'Position')
plt.plot(t,y[1], label = 'Velocity')
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('LQR Simulation')
plt.legend()
plt.grid()
plt.show()
