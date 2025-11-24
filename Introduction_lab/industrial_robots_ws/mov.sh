#!/bin/bash
# ============================================================
# Script para mover 6 joints en grupos de acciones (posturas)
# Cada acción define una posición para TODOS los joints
# Compatible con Ignition Gazebo / Gazebo Garden / Harmonic
# Autor: Brayan Gerson Durán Toconas
# ============================================================

# ---- Configuración ----

# Lista de joints
JOINTS=(joint_b1 joint_b2 joint_b3 joint_b4 joint_b5 joint_b6)

# Acciones: cada fila = una postura completa del robot
# Formato: (j1 j2 j3 j4 j5 j6)
ACCION_1=(0.0  0.2  -1.5  0.0  0.3  -0.8)
ACCION_2=(-0.16 -0.1  -0.9  0.0  0.0  -0.12)
ACCION_3=(-1.5  -0.1  -1.2  -0.7  0.0  -0.12)
ACCION_4=(1.5  -0.1  -1.2  -0.7  0.0  -0.12)

# Tiempo entre acciones (segundos)
DELAY=4

# ---- Ejecución ----

echo "🤖 Iniciando secuencia de acciones de joints..."

while true; do
  for a in 1 2 3 4; do
    echo "➡️ Ejecutando acción $a ..."
    
    # Obtener el array correspondiente
    eval POSICIONES=(\${ACCION_${a}[@]})
    
    # Publicar en paralelo las posiciones para cada joint
    for i in "${!JOINTS[@]}"; do
      J=${JOINTS[$i]}
      POS=${POSICIONES[$i]}
      echo "   - ${J} → ${POS} rad"
      ign topic -t "/${J}/cmd_pos" -m ignition.msgs.Double -p "data:${POS}" &
    done

    wait  # Esperar a que todos terminen antes de pasar a la siguiente acción
    sleep $DELAY
  done
done
