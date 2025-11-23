import numpy as np, matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

R,L,Ke,Kt,J,b = 1,0.5,0.01,0.01,0.01,0.1
A = np.array([[-R/L, -Ke/L],[Kt/J, -b/J]])
B = np.array([[1/L],
              [0.0]]);
C = np.array([[0,1]])

def motor_dc(t, x, u):
    x = np.asarray(x).reshape(-1, 1)             # (2,1)
    dx = A @ x + B * u                           # (2,1)
    return dx.flatten() 

u=5; #Volts
x0=np.zeros(2) #Initial Conditions
t_eval=np.linspace(0,3,400) #Simulation Time 3s
sol=solve_ivp(motor_dc,(0,3),x0,args=(u,),t_eval=t_eval) #Solve differential equation
y=(C@sol.y).flatten() #Multiply C*x in which X are system states
plt.plot(sol.t,y)
plt.title("Motor DC – Lazo abierto")
plt.xlabel("Tiempo [s]") 
plt.ylabel("Velocidad [rad/s]") 
plt.grid(); 
plt.show()

# CLOSED LOOP
Kp=5; r=50
def closed_loop(t,x):
    y=(C@x).item();
    e=r-y;
    u=Kp*e
    x = np.asarray(x).reshape(-1, 1)             # (2,1)
    dx = A @ x + B * u                           # (2,1)
    return dx.flatten() 

sol2=solve_ivp(closed_loop,(0,3),x0,t_eval=t_eval)
y2=(C@sol2.y).flatten()
plt.plot(sol2.t,y2,label="Lazo cerrado")
plt.axhline(r,color='r',ls='--',label="Referencia")
plt.legend(); plt.title("Control proporcional de velocidad")
plt.xlabel("Tiempo [s]"); plt.ylabel("Velocidad [rad/s]"); plt.grid(); plt.show()