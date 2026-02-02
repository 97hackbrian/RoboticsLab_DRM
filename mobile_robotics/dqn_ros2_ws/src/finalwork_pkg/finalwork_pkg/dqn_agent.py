"""
DQN Agent con sklearn MLPRegressor

Implementa un agente Deep Q-Network (DQN) mejorado usando MLPRegressor
de scikit-learn como aproximador de función.

Características:
- Double DQN: Reduce sobreestimación de Q-values
- Prioritized Experience Replay (PER): Enfoca en experiencias importantes
- Target Network: Proporciona targets estables para entrenamiento
- Epsilon-Greedy: Exploración con decaimiento adaptativo
- Reward Clipping: Estabiliza gradientes

Autor: Proyecto DQN Navigation - finalwork_pkg
"""

import numpy as np
import random
import pickle
import warnings
from collections import deque
from typing import Optional, Tuple, List, Dict, Any

from sklearn.neural_network import MLPRegressor

# Suprimir warnings de sklearn sobre partial_fit
warnings.filterwarnings("ignore", category=UserWarning)


# =============================================================================
# HIPERPARÁMETROS
# =============================================================================
GAMMA = 0.99                # Factor de descuento
EPSILON_START = 1.0         # Exploración inicial (100%)
EPSILON_MIN = 0.02          # Exploración mínima (2%) - mantener exploración
EPSILON_DECAY = 0.998       # Decaimiento más rápido (llega a 0.02 en ~1500 episodios)
LEARNING_RATE = 0.0005      # Tasa de aprendizaje aumentada
BATCH_SIZE = 256            # Batch size estándar
MEMORY_SIZE = 70000        # Buffer más grande

# Reward clipping - DEBE cubrir las recompensas terminales
# REWARD_GOAL=200 y REWARD_COLLISION=-200, los límites deben cubrirlos con margen
REWARD_CLIP_MIN = -250.0    # Cubre colisión completa (-200) con margen
REWARD_CLIP_MAX = 250.0     # Cubre goal completo (200) con margen

# Prioritized Experience Replay
PER_ALPHA = 0.6             # Prioridad (0 = uniforme, 1 = full priority)
PER_BETA_START = 0.4        # Importance sampling inicial
PER_BETA_INCREMENT = 0.0006  # Incremento de beta por episodio (llega a 1.0 en ~1000 eps)
PER_EPSILON = 0.000001          # Valor mínimo para evitar prioridad 0

# Double DQN
USE_DOUBLE_DQN = True


# =============================================================================
# SUM TREE PARA PER
# =============================================================================
class SumTree:
    """
    Estructura de datos para Prioritized Experience Replay.
    Permite muestreo eficiente proporcional a prioridades O(log n).
    """
    
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.tree = np.zeros(2 * capacity - 1)
        self.data = np.zeros(capacity, dtype=object)
        self.data_pointer = 0
        self.n_entries = 0
    
    def add(self, priority: float, data: Any):
        """Añade experiencia con prioridad."""
        tree_idx = self.data_pointer + self.capacity - 1
        self.data[self.data_pointer] = data
        self.update(tree_idx, priority)
        
        self.data_pointer = (self.data_pointer + 1) % self.capacity
        self.n_entries = min(self.n_entries + 1, self.capacity)
    
    def update(self, tree_idx: int, priority: float):
        """Actualiza la prioridad de una experiencia."""
        change = priority - self.tree[tree_idx]
        self.tree[tree_idx] = priority
        
        # Propagar cambio hacia arriba
        while tree_idx != 0:
            tree_idx = (tree_idx - 1) // 2
            self.tree[tree_idx] += change
    
    def get(self, value: float) -> Tuple[int, float, Any]:
        """Obtiene experiencia basada en valor de muestreo."""
        parent_idx = 0
        
        while True:
            left_idx = 2 * parent_idx + 1
            right_idx = left_idx + 1
            
            if left_idx >= len(self.tree):
                leaf_idx = parent_idx
                break
            
            if value <= self.tree[left_idx]:
                parent_idx = left_idx
            else:
                value -= self.tree[left_idx]
                parent_idx = right_idx
        
        data_idx = leaf_idx - self.capacity + 1
        return leaf_idx, self.tree[leaf_idx], self.data[data_idx]
    
    @property
    def total_priority(self) -> float:
        return self.tree[0]


# =============================================================================
# PRIORITIZED REPLAY BUFFER
# =============================================================================
class PrioritizedReplayBuffer:
    """
    Buffer de experiencia con priorización.
    Las experiencias con mayor error TD se muestrean más frecuentemente.
    """
    
    def __init__(self, capacity: int, alpha: float = PER_ALPHA):
        self.tree = SumTree(capacity)
        self.capacity = capacity
        self.alpha = alpha
        self.max_priority = 1.0
    
    def add(self, state: np.ndarray, action: int, reward: float, 
            next_state: np.ndarray, done: bool):
        """Añade experiencia con prioridad máxima inicial."""
        experience = (state, action, reward, next_state, done)
        priority = self.max_priority ** self.alpha
        self.tree.add(priority, experience)
    
    def sample(self, batch_size: int, beta: float = PER_BETA_START) -> Tuple[List, List, np.ndarray]:
        """
        Muestrea batch con prioridades.
        
        Returns:
            Tuple de (batch, indices, weights)
        """
        batch = []
        indices = []
        priorities = []
        
        segment = self.tree.total_priority / batch_size
        
        for i in range(batch_size):
            low = segment * i
            high = segment * (i + 1)
            value = random.uniform(low, high)
            
            idx, priority, data = self.tree.get(value)
            
            if data != 0:
                batch.append(data)
                indices.append(idx)
                priorities.append(priority)
        
        # Importance sampling weights
        sampling_probs = np.array(priorities) / self.tree.total_priority
        weights = (self.tree.n_entries * sampling_probs) ** (-beta)
        weights /= weights.max()  # Normalizar
        
        return batch, indices, weights
    
    def update_priorities(self, indices: List[int], td_errors: np.ndarray):
        """Actualiza prioridades basadas en errores TD."""
        for idx, td_error in zip(indices, td_errors):
            priority = (abs(td_error) + PER_EPSILON) ** self.alpha
            self.tree.update(idx, priority)
            self.max_priority = max(self.max_priority, priority)
    
    def __len__(self) -> int:
        return self.tree.n_entries


# =============================================================================
# DQN AGENT
# =============================================================================
class DQNAgent:
    """
    Agente DQN con Double DQN y Prioritized Experience Replay.
    
    Usa MLPRegressor de sklearn como red neuronal.
    
    Arquitectura:
    - Input: Vector de estado (22 dims)
    - Hidden: (256, 256, 128) con ReLU
    - Output: Q-values para cada acción
    
    Double DQN:
    - Main Network selecciona la mejor acción
    - Target Network evalúa el Q-value de esa acción
    - y_j = r + γ * Q_target(s', argmax_a Q_main(s', a))
    """
    
    def __init__(self, state_size: int, action_size: int, use_per: bool = True):
        """
        Inicializa el agente DQN.
        
        Args:
            state_size: Dimensión del vector de estado
            action_size: Número de acciones discretas
            use_per: Usar Prioritized Experience Replay
        """
        self.state_size = state_size
        self.action_size = action_size
        self.use_per = use_per
        
        # Buffer de experiencia
        if use_per:
            self.memory = PrioritizedReplayBuffer(MEMORY_SIZE)
        else:
            self.memory = deque(maxlen=MEMORY_SIZE)
        
        # Parámetros de exploración
        self.epsilon = EPSILON_START
        self.beta = PER_BETA_START
        
        # Contador de pasos de entrenamiento
        self.train_step = 0
        
        # Main Network (Q-Network)
        self.model = MLPRegressor(
            hidden_layer_sizes=(256, 256, 128),
            activation='relu',
            solver='adam',
            learning_rate='adaptive',  # Adapta learning rate cuando se estanca
            learning_rate_init=LEARNING_RATE,
            warm_start=True,
            max_iter=10  # 10 iteraciones por partial_fit (CRÍTICO para aprendizaje)
        )
        
        # Target Network
        self.target_model = MLPRegressor(
            hidden_layer_sizes=(256, 256, 128),
            activation='relu',
            solver='adam',
            learning_rate='adaptive',  # Adapta learning rate cuando se estanca
            learning_rate_init=LEARNING_RATE,
            warm_start=True,
            max_iter=10  # 10 iteraciones por partial_fit (CRÍTICO para aprendizaje)
        )
        
        # Inicializar redes
        self._initialize_networks()
        
        # Estadísticas
        self.td_errors_history: List[float] = []
        self.q_values_history: List[float] = []
    
    def _initialize_networks(self):
        """Inicializa las redes con datos random pequeños."""
        # Usar valores aleatorios pequeños en vez de zeros para evitar sesgo inicial
        dummy_X = np.random.rand(10, self.state_size) * 0.1
        dummy_y = np.random.rand(10, self.action_size) * 0.1
        
        self.model.partial_fit(dummy_X, dummy_y)
        self.target_model.partial_fit(dummy_X, dummy_y)
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                 next_state: np.ndarray, done: bool):
        """
        Almacena una experiencia en el buffer.
        
        Aplica reward clipping para estabilidad.
        """
        clipped_reward = np.clip(reward, REWARD_CLIP_MIN, REWARD_CLIP_MAX)
        
        if self.use_per:
            self.memory.add(state, action, clipped_reward, next_state, done)
        else:
            self.memory.append((state, action, clipped_reward, next_state, done))
    
    def act(self, state: np.ndarray, training: bool = True) -> int:
        """
        Selecciona una acción usando política Epsilon-Greedy.
        
        Args:
            state: Estado actual
            training: Si False, usa epsilon mínimo (explotación)
            
        Returns:
            Índice de la acción seleccionada
        """
        eps = self.epsilon if training else EPSILON_MIN
        
        if np.random.rand() <= eps:
            return random.randrange(self.action_size)
        
        state_reshaped = np.reshape(state, [1, self.state_size])
        q_values = self.model.predict(state_reshaped)
        
        # Guardar estadísticas
        self.q_values_history.append(float(np.max(q_values[0])))
        
        return int(np.argmax(q_values[0]))
    
    def replay(self, batch_size: int = BATCH_SIZE):
        """
        Entrena la red con Double DQN y PER.
        
        Double DQN reduce la sobreestimación sistemática de Q-values.
        """
        # Verificar memoria suficiente
        if len(self.memory) < batch_size:
            return
        
        # Muestrear batch
        if self.use_per:
            minibatch, indices, weights = self.memory.sample(batch_size, self.beta)
        else:
            minibatch = random.sample(self.memory, batch_size)
            weights = np.ones(batch_size)
            indices = None
        
        # Preparar arrays
        states = np.array([exp[0] for exp in minibatch])
        next_states = np.array([exp[3] for exp in minibatch])
        
        if len(states.shape) == 3:
            states = states.reshape(batch_size, -1)
        if len(next_states.shape) == 3:
            next_states = next_states.reshape(batch_size, -1)
        
        # Predicciones en batch
        current_qs = self.model.predict(states)
        next_qs_main = self.model.predict(next_states)
        next_qs_target = self.target_model.predict(next_states)
        
        X_train = []
        y_train = []
        td_errors = []
        
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            if done:
                target = reward
            else:
                if USE_DOUBLE_DQN:
                    # Double DQN: seleccionar con main, evaluar con target
                    best_action = np.argmax(next_qs_main[i])
                    target = reward + GAMMA * next_qs_target[i][best_action]
                else:
                    # DQN estándar
                    target = reward + GAMMA * np.max(next_qs_target[i])
            
            # Calcular TD error para PER
            td_error = abs(target - current_qs[i][action])
            td_errors.append(td_error)
            
            # Actualizar Q-value (SIN aplicar weight al target)
            # Los weights solo afectan el muestreo, no el target Q-value
            target_f = current_qs[i].copy()
            target_f[action] = target
            
            X_train.append(states[i])
            y_train.append(target_f)
        
        # Entrenar
        self.model.partial_fit(np.array(X_train), np.array(y_train))
        
        # Actualizar prioridades en PER
        if self.use_per and indices is not None:
            self.memory.update_priorities(indices, np.array(td_errors))
        
        # Guardar estadísticas
        self.td_errors_history.append(float(np.mean(td_errors)))
        
        # Decaimiento de epsilon
        if self.epsilon > EPSILON_MIN:
            self.epsilon *= EPSILON_DECAY
        
        # Incrementar beta para importance sampling
        self.beta = min(1.0, self.beta + PER_BETA_INCREMENT)
        
        self.train_step += 1
    
    def update_target_network(self):
        """
        Actualiza Target Network usando HARD COPY (deep copy real).
        
        Esto es crítico para estabilidad: la target network debe proporcionar
        targets ESTABLES, no targets que cambian continuamente.
        """
        # Realizar deep copy usando pickle (única forma con MLPRegressor)
        self.target_model = pickle.loads(pickle.dumps(self.model))
    
    def get_training_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas de entrenamiento."""
        stats = {
            'epsilon': self.epsilon,
            'beta': self.beta,
            'memory_size': len(self.memory),
            'train_steps': self.train_step
        }
        
        if self.td_errors_history:
            stats['avg_td_error'] = np.mean(self.td_errors_history[-100:])
        if self.q_values_history:
            stats['avg_q_value'] = np.mean(self.q_values_history[-100:])
        
        return stats
    
    def save(self, filepath: str):
        """Guarda el modelo entrenado."""
        save_data = {
            'model': self.model,
            'target_model': self.target_model,
            'epsilon': self.epsilon,
            'beta': self.beta,
            'state_size': self.state_size,
            'action_size': self.action_size,
            'use_per': self.use_per,
            'train_step': self.train_step,
            'td_errors_history': self.td_errors_history[-1000:],
            'q_values_history': self.q_values_history[-1000:]
        }
        with open(filepath, 'wb') as f:
            pickle.dump(save_data, f)
    
    def load(self, filepath: str):
        """Carga un modelo previamente entrenado."""
        with open(filepath, 'rb') as f:
            save_data = pickle.load(f)
        
        self.model = save_data['model']
        self.target_model = save_data['target_model']
        self.epsilon = save_data['epsilon']
        self.beta = save_data.get('beta', PER_BETA_START)
        self.state_size = save_data['state_size']
        self.action_size = save_data['action_size']
        self.use_per = save_data.get('use_per', False)
        self.train_step = save_data.get('train_step', 0)
        self.td_errors_history = save_data.get('td_errors_history', [])
        self.q_values_history = save_data.get('q_values_history', [])
