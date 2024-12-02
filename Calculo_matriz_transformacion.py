import numpy as np

def calculate_transformation_matrix(tx, ty, tz, rx, ry, rz):
    # Crear la matriz de rotación (usando la convención de ángulos pequeños)
    rx_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)],
    ])
    ry_matrix = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)],
    ])
    rz_matrix = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1],
    ])
    
    # Multiplicar las matrices para obtener la rotación total
    rotation_matrix = rz_matrix @ ry_matrix @ rx_matrix
    
    # Construir la matriz de transformación 4x4
    transformation_matrix = np.eye(4)  # Matriz identidad
    transformation_matrix[:3, :3] = rotation_matrix  # Insertar la rotación
    transformation_matrix[:3, 3] = [tx, ty, tz]  # Insertar la traslación
    
    return transformation_matrix

def format_matrix_for_yaml(matrix):
    # Formatear la matriz en estilo YAML con notación científica
    formatted_data = []
    for row in matrix:
        formatted_data.extend(row)
    yaml_data = ", ".join([f"{value:.8e}" for value in formatted_data])
    return yaml_data

# Valores de entrada (en radianes para los ángulos)
tx, ty, tz = 119.8580, -0.1914, 0.5166
rx, ry, rz = 0.0011, 0.0000, 0.0016  # Rotaciones pequeñas en radianes

# Cálculo de la matriz
transformation_matrix = calculate_transformation_matrix(tx, ty, tz, rx, ry, rz)

# Formatear para YAML
yaml_formatted_matrix = format_matrix_for_yaml(transformation_matrix)

# Mostrar el resultado
print("Matriz:")
print(yaml_formatted_matrix)

