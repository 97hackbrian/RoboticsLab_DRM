import numpy as np, matplotlib.pyplot as plt
def wrap(a): return (a + np.pi)%(2*np.pi) - np.pi

def simulate_lqr(goal=(5,5), Q=10.0, R=1.0, T=15, dt=0.01):
    t = np.arange(0,T,dt)
    x,y,th = 0.0,0.0,0.0
    v = 1.0
    K = np.sqrt(Q/R)
    XY, E = [], []

    for _ in t:
        e = wrap(np.arctan2(goal[1]-y, goal[0]-x) - th)
        w = -K*e

        x += v*np.cos(th)*dt
        y += v*np.sin(th)*dt
        th = wrap(th + w*dt)

        XY.append((x,y)); E.append(e)

    return t, np.array(XY), np.array(E), K

t, XY, E, K = simulate_lqr()
plt.figure(); plt.plot(XY[:,0], XY[:,1]); plt.scatter([5],[5],c='r'); plt.axis('equal'); plt.grid(); plt.title(f"Uniciclo con LQR (K={K:.2f})")
plt.figure(); plt.plot(t, E); plt.grid(); plt.title("Error angular e_theta (LQR)"); plt.show()
