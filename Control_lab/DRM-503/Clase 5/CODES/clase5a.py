import numpy as np, matplotlib.pyplot as plt
def wrap(a): return (a + np.pi)%(2*np.pi) - np.pi

def simulate_mrac(goal=(5,5), a_m=3.0, gamma=3.0, T=15, dt=0.01):
    t = np.arange(0,T,dt)
    x,y,th = 0.0,0.0,0.0
    v = 1.0
    theta_a = 1.0
    em = 0.0
    XY, E, THA = [], [], []

    for _ in t:
        e = wrap(np.arctan2(goal[1]-y, goal[0]-x) - th)
        # referencia (virtual): e_m
        em += (-a_m*em)*dt
        # control
        w = -theta_a * (e - em)
        # adaptación
        theta_a += -gamma*(e - em)*e*dt

        x += v*np.cos(th)*dt
        y += v*np.sin(th)*dt
        th = wrap(th + w*dt)

        XY.append((x,y)); E.append(e); THA.append(theta_a)

    return t, np.array(XY), np.array(E), np.array(THA)

t, XY, E, THA = simulate_mrac()
plt.figure(); plt.plot(XY[:,0], XY[:,1]); plt.scatter([5],[5],c='r'); plt.axis('equal'); plt.grid(); plt.title("Uniciclo con MRAC")
plt.figure(); plt.plot(t, E, label="e_theta"); plt.plot(t, THA, label="theta_a"); plt.legend(); plt.grid(); plt.title("Evolución: error y parámetro")
plt.show()
