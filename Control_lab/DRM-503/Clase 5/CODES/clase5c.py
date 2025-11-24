import numpy as np, matplotlib.pyplot as plt
def wrap(a): return (a + np.pi)%(2*np.pi) - np.pi

def simulate_hinf_like(goal=(5,5), k=6.0, beta=2.0, delta_max=0.4, T=15, dt=0.01):
    t = np.arange(0,T,dt)
    x,y,th = 0.0,0.0,0.0
    v = 1.0
    XY, E = [], []

    for _ in t:
        e = wrap(np.arctan2(goal[1]-y, goal[0]-x) - th)
        w_cmd = -k*np.tanh(beta*e)
        delta = np.random.uniform(-delta_max, delta_max)
        w = (1+delta)*w_cmd

        x += v*np.cos(th)*dt
        y += v*np.sin(th)*dt
        th = wrap(th + w*dt)

        XY.append((x,y)); E.append(e)

    return t, np.array(XY), np.array(E)

t, XY, E = simulate_hinf_like()
plt.figure(); plt.plot(XY[:,0], XY[:,1]); plt.scatter([5],[5],c='r'); plt.axis('equal'); plt.grid(); plt.title("Uniciclo con robusto H∞-like")
plt.figure(); plt.plot(t, E); plt.grid(); plt.title("Error angular e_theta (H∞-like)"); plt.show()
