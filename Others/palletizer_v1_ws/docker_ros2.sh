#!/bin/bash

# Configuración de imagen
export ROS2_IMG=ros2-humble-gazebo-fortress

#permisos aceleracion grafica
#sudo chmod 666 /dev/dri/renderD128
#sudo chmod 666 /dev/dri/renderD129

# --- LÓGICA DE DIRECTORIO UNIVERSAL ---

# 1. Definir la acción
ACCION=$1

# 2. Determinar la RUTA REAL DEL PROYECTO (ROS2_WS_PATH)
# Si pasas un segundo argumento (ej: ./script init carpeta_x), usa esa carpeta.
# Si no pasas nada, usa la carpeta actual donde estás parado ($PWD).
TARGET_DIR="${2:-$PWD}"

# 'realpath' convierte "../proyecto" o "." en "/home/usuario/documentos/proyecto"
if ! command -v realpath &> /dev/null; then
    # Fallback para sistemas sin realpath
    export ROS2_WS_PATH="$(cd "$TARGET_DIR" && pwd)"
else
    export ROS2_WS_PATH=$(realpath "$TARGET_DIR")
fi

# 3. Definir el Nombre del Proyecto (ROS2_DIR)
# Usamos el nombre de la última carpeta de la ruta para identificar el proyecto
export ROS2_DIR=$(basename "$ROS2_WS_PATH")

# 4. Definir el Nombre del Contenedor (ROS2_CONT)
# Por defecto usa el nombre del proyecto. Si pasas un 3er argumento, usa ese.
export ROS2_CONT="${3:-$ROS2_DIR}"

# --- CARGAR FUNCIONES ---
source ~/ros/ros2-cont

# --- FEEDBACK VISUAL ---
echo "---------------------------------------------"
echo "Acción:     $ACCION"
echo "Ruta Host:  $ROS2_WS_PATH"   # <-- Esto es lo que se montará en /ros
echo "Proyecto:   $ROS2_DIR"
echo "Contenedor: $ROS2_CONT"
echo "---------------------------------------------"

# --- EJECUCIÓN ---
case "$ACCION" in
    exec)
        ros2-exec
        ;;
    init)
        ros2-init
        ;;
    rm)
        ros2-rm
        ;;
    *)
        echo "Uso: $0 {exec|init|rm} [ruta_carpeta_proyecto] [nombre_contenedor]"
        exit 1
        ;;
esac