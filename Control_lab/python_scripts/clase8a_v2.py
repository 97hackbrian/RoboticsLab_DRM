import numpy as np
from numpy import sin, cos
import tensorflow as tf
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

np.random.seed(0)
tf.random.set_seed(0)

#1DOF Robot
J_real = 1.0 #M
b_real = 0.4 #C
m_real = 1.0 #G
l_real = 1.1 #G
g = 9.81     #G

#Parametros ideales
J_hat = 0.8
b_hat = 0.2
m_hat = 0.9
l_hat = 1.0

#Ganancias del Reference Tracking
Kp = 25.0
Kd = 10.0

#Función para el RT con múltiples tipos de trayectoria y ruido
def ref_traj(t, traj_type="multi_sinusoidal", noise_level=0.02):
    """
    Genera trayectorias de referencia con opción de ruido
    
    Parámetros:
    t: tiempo
    traj_type: tipo de trayectoria ("sinusoidal", "step", "ramp", "polynomial")
    noise_level: nivel de ruido gaussiano (desviación estándar)
    """
    
    # Ruido gaussiano
    noise_q = np.random.normal(0, noise_level)
    noise_dq = np.random.normal(0, noise_level * 0.5)
    noise_ddq = np.random.normal(0, noise_level * 0.2)
    
    if traj_type == "sinusoidal":
        # Trayectoria sinusoidal original
        qd = 0.5 * np.sin(0.5 * t)
        dqd = 0.5 * 0.5 * np.cos(0.5 * t)
        ddqd = -0.5 * 0.5 * 0.5 * np.sin(0.5 * t)
        
    elif traj_type == "step":
        # Trayectoria escalón suavizado
        if t < 5.0:
            qd = 0.0
            dqd = 0.0
            ddqd = 0.0
        else:
            qd = 0.3
            dqd = 0.0
            ddqd = 0.0
            
    elif traj_type == "ramp":
        # Trayectoria rampa
        if t < 8.0:
            qd = 0.1 * t
            dqd = 0.1
            ddqd = 0.0
        else:
            qd = 0.8
            dqd = 0.0
            ddqd = 0.0
            
    elif traj_type == "polynomial":
        # Trayectoria polinómica suave
        if t < 10.0:
            qd = 0.02 * t**2 - 0.001 * t**3
            dqd = 0.04 * t - 0.003 * t**2
            ddqd = 0.04 - 0.006 * t
        else:
            qd = 0.2
            dqd = 0.0
            ddqd = 0.0
            
    elif traj_type == "multi_sinusoidal":
        # Trayectoria sinusoidal múltiple
        qd = 0.3 * np.sin(0.3 * t) + 0.2 * np.sin(0.8 * t)
        dqd = 0.3 * 0.3 * np.cos(0.3 * t) + 0.2 * 0.8 * np.cos(0.8 * t)
        ddqd = -0.3 * 0.3 * 0.3 * np.sin(0.3 * t) - 0.2 * 0.8 * 0.8 * np.sin(0.8 * t)
    
    else:
        # Default: sinusoidal
        qd = 0.5 * np.sin(0.5 * t)
        dqd = 0.5 * 0.5 * np.cos(0.5 * t)
        ddqd = -0.5 * 0.5 * 0.5 * np.sin(0.5 * t)
    
    # Añadir ruido a las trayectorias
    qd_noisy = qd + noise_q
    dqd_noisy = dqd + noise_dq
    ddqd_noisy = ddqd + noise_ddq
    
    return qd_noisy, dqd_noisy, ddqd_noisy

#Cálculo del torque con modelo aproximado
def tau_ctc(q, dq, t):
    
    #Torque computado basado en modelo aproximado (J_hat, b_hat, etc.)
    #No incluye la NN aún
    
    qd, dqd, ddqd = ref_traj(t)
    e = qd - q
    de = dqd - dq
    v = ddqd + Kd * de + Kp * e

    # gravedad modelo - g=mgl*sin(q)
    g_hat = m_hat * g * l_hat * sin(q)

    # Dinámica inversa = tau_modelo
    tau_model = J_hat * v + b_hat * dq + g_hat
    return tau_model

#Dinamica real utilizada para simular datos
def dynamics_real(t, x, use_nn=False, nn_model=None):
    
    #x = [q, dq]
    #Si use_nn=True, se añade la compensación NN al torque.
    
    q, dq = x
    qd, dqd, ddqd = ref_traj(t)
    # Torque computado base
    tau_base = tau_ctc(q, dq, t)

    # Torque extra de la NN (compensador)
    tau_comp = 0.0
    if use_nn and nn_model is not None:
        # Entrada NN = [q, dq, e, de]
        e = qd - q
        de = dqd - dq
        inp = np.array([[q, dq, e, de]], dtype=np.float32)
        tau_comp = nn_model(inp).numpy().flatten()[0]

    tau = tau_base + tau_comp

    # Dinámica real = J_real * ddq + b_real * dq + m_real*g*l_real*sin(q) = tau
    ddq = (tau - b_real * dq - m_real * g * l_real * sin(q)) / J_real
    return [dq, ddq]

#Generación de datos para entrenar el compensador
def generate_dataset(num_trajectories=20, T=20.0):
    X = []
    Y = [] # residuo = tau_real - tau_ctc

    for k in range(num_trajectories):

        # Condiciones iniciales aleatorias alrededor de 0
        q0 = np.random.uniform(-0.5, 0.5)
        dq0 = np.random.uniform(-0.5, 0.5)
        x0 = [q0, dq0]

        t_eval = np.linspace(0, T, 200)

        # Integramos con tau_ctc sin NN, pero midiendo el "tau_real" necesario
        sol = solve_ivp(lambda t, x: dynamics_real_no_data(t, x),
                        [0, T], x0, t_eval=t_eval)

        q = sol.y[0]
        dq = sol.y[1]

        for i, t in enumerate(t_eval):
            qi = q[i]
            dqi = dq[i]
            qd, dqd, ddqd = ref_traj(t)

            # AUMENTAR RUIDO A LOS SETPOINTS
            noise_qd = qd + np.random.normal(0, 0.05)
            noise_dqd = dqd + np.random.normal(0, 0.02)
            noise_ddqd = ddqd + np.random.normal(0, 0.01)

            # Torque de CTC (modelo aproximado)
            tau_base = tau_ctc(qi, dqi, t)

            # Torque "real" requerido según la dinámica real
            # tau_real = J_real*ddq + b_real*dq + m_real*g*l_real*sin(q)
            # ddq lo obtenemos de la derivada numérica de dq
            # Para simplificar, aproximamos ddq con diferencias finitas:
            if i == 0 or i == len(t_eval) - 1:
                continue
            dt = t_eval[1] - t_eval[0]
            ddqi = (dq[i+1] - dq[i-1]) / (2*dt)
            tau_real = J_real * ddqi + b_real * dqi + m_real * g * l_real * sin(qi)

            # Residuo que la NN debe aprender
            r = tau_real - tau_base

            e = noise_qd - qi
            de = noise_dqd - dqi
            X.append([qi, dqi, e, de])
            Y.append([r])

    X = np.array(X, dtype=np.float32)
    Y = np.array(Y, dtype=np.float32)
    return X, Y

def dynamics_real_no_data(t, x):
    #Dinámica utilizada solo para generar datos
    #Usa tau_ctc sin NN

    q, dq = x
    # Torque base
    tau = tau_ctc(q, dq, t)
    ddq = (tau - b_real * dq - m_real * g * l_real * sin(q)) / J_real
    return [dq, ddq]

# Generamos dataset
X_train, Y_train = generate_dataset()

print("Dataset shape:", X_train.shape, Y_train.shape)

#Definimos el modelo de la NN (Compensador)
comp_model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(4,)), # [q, dq, e, de]
    tf.keras.layers.Dense(32, activation="tanh"),
    tf.keras.layers.Dense(32, activation="tanh"),
    tf.keras.layers.Dense(1) # salida = tau_comp
])

comp_model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                   loss="mse")

# Entrenamos el modelo a 20 epocas
history = comp_model.fit(X_train, Y_train,
                         epochs=80, batch_size=64, verbose=1)

#No NN vs NN
def simulate_system(use_nn=False, nn_model=None, T=20.0):
    x0 = [0.0, 0.0] # Condiciones iniciales
    t_eval = np.linspace(0, T, 500)
    sol = solve_ivp(lambda t, x: dynamics_real(t, x, use_nn=use_nn, nn_model=nn_model),
                    [0, T], x0, t_eval=t_eval)
    return t_eval, sol.y

# Simulación sin conmpensador NN
t, x_no_nn = simulate_system(use_nn=False, nn_model=None)
q_no_nn, dq_no_nn = x_no_nn

# Simulación con compensador NN
t2, x_with_nn = simulate_system(use_nn=True, nn_model=comp_model)
q_nn, dq_nn = x_with_nn

# Trayectoria deseada
qd = np.array([ref_traj(tt)[0] for tt in t])

#Figuras
plt.figure()
plt.plot(t, qd, 'k--', label="q_d (deseada)")
plt.plot(t, q_no_nn, 'r', label="q sin NN")
plt.plot(t2, q_nn, 'b', label="q con NN")
plt.xlabel("t [s]")
plt.ylabel("posición articular q [rad]")
plt.legend()
plt.title("Tracking con y sin compensación NN")
plt.grid(True)
plt.show()


#Controlador Directo por NN
def tau_reference(q, dq, t):

    #Controlador de referencia "ideal", por ejemplo torque computado
    #pero ahora usando los parámetros reales J_real, b_real, m_real.
    #Así generamos las mejores acciones posibles para entrenar a la NN

    qd, dqd, ddqd = ref_traj(t)
    e = qd - q
    de = dqd - dq
    v = ddqd + Kd * de + Kp * e

    g_real = m_real * g * l_real * sin(q)
    tau_star = J_real * v + b_real * dq + g_real
    return tau_star

def generate_dataset_direct(num_trajectories=20, T=10.0):
    X = []
    Y = []

    for k in range(num_trajectories):
        q0 = np.random.uniform(-0.5, 0.5)
        dq0 = np.random.uniform(-0.5, 0.5)
        x0 = [q0, dq0]
        t_eval = np.linspace(0, T, 200)

        # Integramos la dinámica REAL usando tau_reference como controlador
        def dyn_ref(t, x):
            q, dq = x
            tau = tau_reference(q, dq, t)
            ddq = (tau - b_real * dq - m_real * g * l_real * sin(q)) / J_real
            return [dq, ddq]

        sol = solve_ivp(dyn_ref, [0, T], x0, t_eval=t_eval)
        q = sol.y[0]
        dq = sol.y[1]

        for i, t in enumerate(t_eval):
            qi = q[i]
            dqi = dq[i]
            qd, dqd, _ = ref_traj(t)
            
            # AUMENTAR RUIDO A LOS SETPOINTS
            noise_qd = qd + np.random.normal(0, 0.05)
            noise_dqd = dqd + np.random.normal(0, 0.02)
            
            tau_star = tau_reference(qi, dqi, t)

            # Entrada para la NN directa: [q, dq, qd, dqd]
            X.append([qi, dqi, noise_qd, noise_dqd])
            Y.append([tau_star])

    X = np.array(X, dtype=np.float32)
    Y = np.array(Y, dtype=np.float32)
    return X, Y

X_dir, Y_dir = generate_dataset_direct()
print("Dataset (direct) shape:", X_dir.shape, Y_dir.shape)

#Definimos el modelo del controlador NN
direct_model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(4,)), # [q, dq, qd, dqd]
    tf.keras.layers.Dense(32, activation="tanh"),
    tf.keras.layers.Dense(32, activation="tanh"),
    tf.keras.layers.Dense(1) # tau
])

direct_model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                     loss="mse")

history_dir = direct_model.fit(X_dir, Y_dir,
                               epochs=80, batch_size=64, verbose=1)

#Simluación haciendo uso de la NN directa
def dynamics_real_direct_nn(t, x, model):
    q, dq = x
    qd, dqd, _ = ref_traj(t)
    
    # AUMENTAR RUIDO A LOS SETPOINTS
    noise_qd = qd + np.random.normal(0, 0.05)
    noise_dqd = dqd + np.random.normal(0, 0.02)
    
    inp = np.array([[q, dq, noise_qd, noise_dqd]], dtype=np.float32)
    tau_nn = model(inp).numpy().flatten()[0]
    ddq = (tau_nn - b_real * dq - m_real * g * l_real * sin(q)) / J_real
    return [dq, ddq]

def simulate_direct_nn(model, T=10.0):
    x0 = [0.3, 0.0]
    t_eval = np.linspace(0, T, 500)
    sol = solve_ivp(lambda t, x: dynamics_real_direct_nn(t, x, model),
                    [0, T], x0, t_eval=t_eval)
    return t_eval, sol.y

t3, x_direct = simulate_direct_nn(direct_model)
q_direct, dq_direct = x_direct
qd_direct = np.array([ref_traj(tt)[0] for tt in t3])

plt.figure()
plt.plot(t3, qd_direct, 'k--', label="q_d (deseada)")
plt.plot(t3, q_direct, 'g', label="q (NN directa)")
plt.xlabel("t [s]")
plt.ylabel("q [rad]")
plt.legend()
plt.title("Tracking usando controlador directo NN")
plt.grid(True)
plt.show()