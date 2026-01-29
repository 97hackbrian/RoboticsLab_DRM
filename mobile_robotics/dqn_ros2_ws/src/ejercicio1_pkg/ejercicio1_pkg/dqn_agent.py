"""
DQN Agent using sklearn MLPRegressor

Este módulo implementa un agente Deep Q-Network (DQN) usando MLPRegressor
de scikit-learn en lugar de PyTorch o TensorFlow.

Características implementadas:
- Experience Replay Buffer
- Target Network para estabilidad
- Política Epsilon-Greedy
- Entrenamiento con Bellman Equation

Autor: Proyecto Académico DQN Navigation
"""

import numpy as np
import random
from collections import deque
from sklearn.neural_network import MLPRegressor
import warnings
import pickle

# Suppress sklearn warnings about partial_fit
warnings.filterwarnings("ignore")


# ============================================================================
# HIPERPARÁMETROS DQN
# ============================================================================
GAMMA = 0.99           # Factor de descuento para recompensas futuras
EPSILON_START = 1.0    # Exploración inicial (100%)
EPSILON_MIN = 0.01     # Exploración mínima (1%)
EPSILON_DECAY = 0.995  # Decaimiento por episodio
LEARNING_RATE = 0.001  # Tasa de aprendizaje
BATCH_SIZE = 64        # Tamaño del minibatch para replay
MEMORY_SIZE = 10000    # Tamaño del buffer de experiencia


class DQNAgent:
    """
    Agente DQN que aprende a navegar usando Q-Learning profundo.
    
    Usa dos redes neuronales (MLPRegressor):
    - Main Network: Se entrena activamente
    - Target Network: Proporciona targets estables
    
    El concepto matemático clave es minimizar el error de Diferencia Temporal (TD):
    L = (y_j - Q(s_j, a_j))^2
    
    donde y_j = r + γ * max(Q_target(s', a'))
    """
    
    def __init__(self, state_size: int, action_size: int):
        """
        Inicializa el agente DQN.
        
        Args:
            state_size: Dimensión del vector de estado (22 para nuestro caso)
            action_size: Número de acciones discretas (5 para nuestro caso)
        """
        self.state_size = state_size
        self.action_size = action_size
        
        # Buffer de experiencia (Experience Replay)
        self.memory = deque(maxlen=MEMORY_SIZE)
        
        # Parámetro de exploración
        self.epsilon = EPSILON_START
        
        # ====================================================================
        # MAIN NETWORK (Q-Network)
        # Esta red se entrena en cada paso de replay
        # ====================================================================
        self.model = MLPRegressor(
            hidden_layer_sizes=(128, 128),  # Dos capas ocultas de 128 neuronas
            activation='relu',               # ReLU como función de activación
            solver='adam',                   # Optimizador Adam
            learning_rate_init=LEARNING_RATE,
            warm_start=True,                 # IMPORTANTE: Mantiene pesos entre llamadas
            max_iter=1                       # Solo 1 iteración por partial_fit
        )
        
        # ====================================================================
        # TARGET NETWORK (Red Objetivo)
        # Esta red se actualiza periódicamente para estabilizar el entrenamiento
        # ====================================================================
        self.target_model = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=LEARNING_RATE,
            warm_start=True,
            max_iter=1
        )
        
        # Inicialización: MLPRegressor necesita un fit inicial para funcionar
        # Creamos datos dummy para inicializar ambas redes
        self._initialize_networks()
    
    def _initialize_networks(self):
        """
        Inicializa las redes con datos dummy.
        MLPRegressor requiere al menos una llamada a fit/partial_fit antes de predict.
        """
        dummy_X = np.zeros((1, self.state_size))
        dummy_y = np.zeros((1, self.action_size))
        
        self.model.partial_fit(dummy_X, dummy_y)
        self.target_model.partial_fit(dummy_X, dummy_y)
    
    def remember(self, state, action: int, reward: float, next_state, done: bool):
        """
        Almacena una experiencia en el buffer de replay.
        
        Esta es la implementación del Experience Replay:
        Guardamos tuplas (s, a, r, s', done) para muestrear después.
        
        Args:
            state: Estado actual
            action: Acción tomada
            reward: Recompensa recibida
            next_state: Estado siguiente
            done: True si el episodio terminó
        """
        self.memory.append((state, action, reward, next_state, done))
    
    def act(self, state) -> int:
        """
        Selecciona una acción usando política Epsilon-Greedy.
        
        - Con probabilidad epsilon: acción aleatoria (exploración)
        - Con probabilidad (1-epsilon): acción óptima según Q-values (explotación)
        
        Args:
            state: Estado actual (array de dimensión state_size)
            
        Returns:
            Índice de la acción seleccionada (0 a action_size-1)
        """
        # EXPLORACIÓN: Acción aleatoria
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        
        # EXPLOTACIÓN: Mejor acción según Q-values
        state_reshaped = np.reshape(state, [1, self.state_size])
        q_values = self.model.predict(state_reshaped)
        return np.argmax(q_values[0])
    
    def replay(self, batch_size: int = BATCH_SIZE):
        """
        Entrena la red con un minibatch del buffer de experiencia.
        
        Implementa la ecuación de Bellman para DQN:
        Q(s, a) = r + γ * max(Q_target(s', a'))
        
        Usamos la Target Network para calcular el valor futuro,
        lo que estabiliza el entrenamiento.
        
        Args:
            batch_size: Número de experiencias a muestrear
        """
        # No entrenar si no hay suficientes experiencias
        if len(self.memory) < batch_size:
            return
        
        # Muestreo aleatorio del buffer (rompe correlación temporal)
        minibatch = random.sample(self.memory, batch_size)
        
        # Preparar arrays para predicción en batch (más eficiente)
        states = np.array([experience[0] for experience in minibatch])
        next_states = np.array([experience[3] for experience in minibatch])
        
        # Asegurar dimensiones correctas
        if len(states.shape) == 3:
            states = states.reshape(batch_size, -1)
        if len(next_states.shape) == 3:
            next_states = next_states.reshape(batch_size, -1)
        
        # Predicción en batch
        current_qs = self.model.predict(states)
        # Usamos TARGET NETWORK para estabilidad
        next_qs = self.target_model.predict(next_states)
        
        X_train = []
        y_train = []
        
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            if done:
                # Si el episodio terminó, el Q-value es solo la recompensa
                target = reward
            else:
                # Ecuación de Bellman: R + γ * max(Q_target(s'))
                # RECOMPENSA +200 por meta, -100 por colisión (ya incluidas en reward)
                target = reward + GAMMA * np.max(next_qs[i])
            
            # Actualizar solo el Q-value de la acción tomada
            # Los demás Q-values permanecen igual
            target_f = current_qs[i].copy()
            target_f[action] = target
            
            X_train.append(states[i])
            y_train.append(target_f)
        
        # Entrenar la red con el minibatch
        self.model.partial_fit(np.array(X_train), np.array(y_train))
        
        # Decaimiento de epsilon (menos exploración con el tiempo)
        if self.epsilon > EPSILON_MIN:
            self.epsilon *= EPSILON_DECAY
    
    def update_target_network(self):
        """
        Actualiza la Target Network copiando los pesos de la Main Network.
        
        En sklearn no podemos copiar pesos directamente como en PyTorch,
        así que reentrenamos la target network con los mismos datos
        que usamos para la main network.
        
        NOTA: Esta es una aproximación. En frameworks como PyTorch/TensorFlow
        simplemente copiamos los pesos.
        """
        # Obtener una muestra del buffer para sincronizar
        if len(self.memory) < BATCH_SIZE:
            return
        
        sample_size = min(len(self.memory), BATCH_SIZE * 2)
        sample = random.sample(self.memory, sample_size)
        
        states = np.array([exp[0] for exp in sample])
        if len(states.shape) == 3:
            states = states.reshape(sample_size, -1)
        
        # Obtener predicciones de la main network
        predictions = self.model.predict(states)
        
        # Entrenar target network para que imite a main network
        self.target_model.partial_fit(states, predictions)
    
    def save(self, filepath: str):
        """
        Guarda el modelo entrenado en un archivo pickle.
        
        Args:
            filepath: Ruta del archivo (ej: 'trained_model.pkl')
        """
        save_data = {
            'model': self.model,
            'target_model': self.target_model,
            'epsilon': self.epsilon,
            'state_size': self.state_size,
            'action_size': self.action_size
        }
        with open(filepath, 'wb') as f:
            pickle.dump(save_data, f)
    
    def load(self, filepath: str):
        """
        Carga un modelo previamente entrenado.
        
        Args:
            filepath: Ruta del archivo pickle
        """
        with open(filepath, 'rb') as f:
            save_data = pickle.load(f)
        
        self.model = save_data['model']
        self.target_model = save_data['target_model']
        self.epsilon = save_data['epsilon']
        self.state_size = save_data['state_size']
        self.action_size = save_data['action_size']
