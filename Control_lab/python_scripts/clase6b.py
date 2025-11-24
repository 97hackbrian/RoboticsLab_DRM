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

#Q = np.diag([2.666, 10, 2.9, 2, 2])
Q = np.diag([1, 1, 1, 1, 1])

def pretty_print(name, M, precision=3):
    """Print matrix/array M in a compact, aligned console-friendly form.
    Tries to use `tabulate` for a clean ASCII table if available; otherwise
    falls back to NumPy's formatted array string. Silent fallback (no install
    messages) so prints stay clean.
    """
    M = np.asarray(M)
    print(f"\n{name} (shape {M.shape}):")
    # try tabulate for nicer tables
    try:
        from tabulate import tabulate
        fmt = f"{{: .{precision}f}}"
        rows = []
        # ensure 2D
        if M.ndim == 1:
            rows = [[fmt.format(x)] for x in M]
        else:
            for r in M:
                rows.append([fmt.format(x) for x in r])
        print(tabulate(rows, tablefmt='fancy_grid'))
        return
    except Exception:
        pass

    # numpy fallback: aligned, reduced precision, suppress small values
    old_opts = np.get_printoptions()
    try:
        np.set_printoptions(precision=precision, suppress=True, linewidth=120)
        print(np.array2string(M, precision=precision, suppress_small=True))
    finally:
        np.set_printoptions(**old_opts)

pretty_print('Q', Q)
R = np.array([[1]])

K,s,e = ct.lqr(A,B,Q,R)
pretty_print('K (LQR gain)', K)

#Controlled System
A_cl = A - B@K
sys_cl = ct.ss(A_cl,B,C,D)
pretty_print('A_cl', A_cl)


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
