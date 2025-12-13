# RRT* Path Planning System for Skid-Steer Robot

Sistema de planificación de trayectorias usando **RRT\*** (Rapidly-exploring Random Tree Star) integrado con el controlador Fuzzy+PID+Lag Compensator existente.

## 📁 Estructura del Proyecto

```
Perception_and_planning_lab/final_work/
├── config/
│   ├── environment_map.m       # Configuración de mapas y obstáculos
│   └── planner_parameters.m    # Parámetros del algoritmo RRT*
├── planners/
│   ├── rrt_star.m              # Implementación del algoritmo RRT*
│   └── generate_waypoints.m    # Generador de waypoints suavizados
├── utils/
│   ├── collision_check.m       # Detección de colisiones
│   ├── path_smoothing.m        # Suavizado de trayectorias
│   └── visualization_utils.m   # Funciones de visualización
├── simulation/
│   ├── rrt_star_navigation_simulation.m  # Simulación completa con robot
│   └── demo_rrt_star_planner.m           # Demo del algoritmo RRT*
├── tests/
│   ├── test_rrt_star_algorithm.m   # Tests unitarios del RRT*
│   └── test_full_navigation.m      # Test de integración
└── README.md
```

## 🚀 Uso Rápido

### Demo del Algoritmo RRT*
```matlab
cd Perception_and_planning_lab/final_work/simulation
demo_rrt_star_planner
```

### Simulación Completa con Robot
```matlab
cd Perception_and_planning_lab/final_work/simulation
rrt_star_navigation_simulation('simple')  % o 'maze', 'narrow', 'challenge'
```

### Ejecutar Tests
```matlab
cd Perception_and_planning_lab/final_work/tests
test_rrt_star_algorithm    % Tests unitarios
test_full_navigation       % Test de integración
```

## 🎯 Escenarios Disponibles

| Escenario   | Descripción                           |
| ----------- | ------------------------------------- |
| `empty`     | Entorno vacío para pruebas básicas    |
| `simple`    | 4 obstáculos (círculos y rectángulos) |
| `maze`      | Laberinto con paredes                 |
| `narrow`    | Pasajes estrechos                     |
| `random`    | 8 obstáculos aleatorios               |
| `challenge` | Escenario complejo con zona densa     |

## ⚙️ Parámetros Principales

Modificables en `config/planner_parameters.m`:

| Parámetro        | Valor Default | Descripción                       |
| ---------------- | ------------- | --------------------------------- |
| `max_iterations` | 3000          | Iteraciones máximas del árbol     |
| `step_size`      | 0.8 m         | Distancia máxima de extensión     |
| `goal_bias`      | 10%           | Probabilidad de muestrear la meta |
| `goal_tolerance` | 0.5 m         | Tolerancia para alcanzar meta     |

## 🔗 Integración con Controlador

Este sistema usa el controlador existente en `Control_lab/final_work/` mediante:
```matlab
addpath('../../../Control_lab/final_work/config');
addpath('../../../Control_lab/final_work/controllers');
addpath('../../../Control_lab/final_work/system');
```

**Componentes utilizados:**
- `robot_parameters.m` - Parámetros físicos del robot
- `fuzzy_yaw_rate_controller_setup.m` - Configuración FIS
- `fuzzy_yaw_rate_controller.m` - Controlador Fuzzy+PID+Lag
- `skid_steer_robot_model.m` - Modelo dinámico del robot

## 📊 Algoritmo RRT*

### Características
- **Asintóticamente óptimo**: Converge al camino más corto
- **Rewiring**: Optimiza continuamente el árbol
- **Goal bias**: Acelera convergencia hacia la meta
- **Suavizado**: Post-procesamiento para waypoints navegables

### Flujo de Ejecución
```
1. Cargar mapa y parámetros
2. Ejecutar RRT* → Path crudo
3. Aplicar suavizado → Waypoints
4. Navegación con controlador Fuzzy+PID+Lag
5. Visualización en tiempo real
```

## 📈 Métricas de Rendimiento

La simulación muestra en tiempo real:
- Distancia al waypoint actual
- Velocidad del robot
- Velocidad angular (omega)
- Tiempo transcurrido
- Waypoints alcanzados

## 📝 Licencia

Parte del proyecto RoboticsLab_DRM - Laboratorio de Robótica
