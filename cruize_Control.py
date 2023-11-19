import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# Definición de parámetros del sistema
m = 1000  # Masa del vehículo en kg
b = 50  # Coeficiente de resistencia al movimiento

# Función de transferencia del sistema
num = [1]
den = [m, b]
P_cruise = ctl.TransferFunction(num, den)

# Espacio de estados
A = -b/m
B = 1/m
C = 1
D = 0
sys_ss = ctl.ss(A, B, C, D)

# Simulación de la respuesta en espacio de estados
t_ss = np.linspace(0, 50, 500)  # Definir un rango de tiempo
y_ss = ctl.step_response(sys_ss, t_ss)[1]  # Respuesta al escalón unitario

# Gráfica de la respuesta en espacio de estados
plt.figure()
plt.plot(t_ss, y_ss)
plt.title('Respuesta en Espacio de Estados a Entrada Escalón Unitario')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (m/s)')
plt.grid(True)

# Controlador PID
Kp = 800  # Ganancia proporcional
Ki = 40  # Ganancia integral
Kd = 10  # Ganancia derivativa
controlador_PID = ctl.TransferFunction([Kd, Kp, Ki], [1, 0])

# Sistema en lazo cerrado con controlador PID
sys_cl = ctl.feedback(controlador_PID*P_cruise, 1)

# Simulación de la respuesta del sistema con controlador PID
setpoint = 80  # Velocidad deseada en m/s
t, y = ctl.step_response(setpoint*sys_cl, t_ss)

# Gráfica de la respuesta del sistema con controlador PID
plt.figure()
plt.plot(t, y)
plt.title('Respuesta del Sistema de Control de Crucero con Controlador PID')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (m/s)')
plt.grid(True)

# Graficar polos y ceros del sistema en lazo cerrado
plt.figure()
ctl.pzmap(sys_cl, Plot=True)
plt.title('Mapa de Polos y Ceros del Sistema en Lazo Cerrado')

# Definir el tiempo de asentamiento
settling_time = 10  # Tiempo de asentamiento en segundos

# Obtener e imprimir los polos del sistema en lazo cerrado
polos = ctl.pole(sys_cl)
print('Los polos del sistema en lazo cerrado son:')
print(polos)


# Función de monitoreo CBM
def cbm_monitoring(velocity, time, setpoint, threshold, settling_time):
    # Asegurarse de que el tiempo de asentamiento está dentro del rango de tiempo simulado
    if time[-1] < settling_time:
        print('El sistema aún no ha alcanzado el tiempo de asentamiento.')
        return
    # Encontrar el índice donde el tiempo es mayor que el tiempo de asentamiento
    settled_idx = np.where(time >= settling_time)[0][0]
    deviation = np.abs(velocity[settled_idx:] - setpoint)
    if any(deviation > threshold):
        print('Mantenimiento recomendado. Desviación excede el umbral.')
    else:
        print('El sistema está operando dentro del rango aceptable.')

# Llamar a la función de monitoreo CBM con los resultados de la simulación
cbm_monitoring(y, t, setpoint, 0.5, settling_time)



# Mostrar todas las gráficas
plt.show()
