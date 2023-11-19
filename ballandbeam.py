import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import control as ctl

# Parámetros del sistema
m = 0.5  # Masa de la pelota (kg)
g = 9.81  # Aceleración de la gravedad (m/s^2)
L = 2.0   # Longitud de la viga (m)
d = 0.1   # Coeficiente de amortiguamiento

# Parámetros del controlador PID
Kp = 3.0  # Ganancia proporcional
Ki = 0.02  # Ganancia integral
Kd = 2  # Ganancia derivativa

# Setpoint
setpoint = 1.0  # Posición deseada de la pelota (m)

# Función del sistema
def ball_beam_system(t, X, Kp, Ki, Kd, setpoint, m, g, L, d):
    x, x_dot, integral_error = X
    error = setpoint - x
    integral_error += error * t
    derivative_error = -x_dot

    # Controlador PID
    u = Kp * error + Ki * integral_error + Kd * derivative_error

    # Dinámica del sistema
    x_ddot = (m * g * np.sin(u) - d * x_dot) / (m * L)

    return [x_dot, x_ddot, error]

# Condiciones iniciales
X0 = [0, 0, 0]  # Posición inicial, velocidad inicial, error integral inicial

# Tiempo de simulación
t_span = (0, 40)  # 10 segundos de simulación
t_eval = np.linspace(*t_span, 300)  # 300 puntos de tiempo

# Resolver la ecuación diferencial
solution = solve_ivp(
    ball_beam_system,
    t_span,
    X0,
    args=(Kp, Ki, Kd, setpoint, m, g, L, d),
    t_eval=t_eval,
    rtol=1e-5,
    atol=1e-8
)

# Extraer resultados
t = solution.t
x = solution.y[0]

# Graficar resultados
plt.plot(t, x, label='Posición')
plt.plot(t, np.full_like(t, setpoint), 'r--', label='Setpoint')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición de la pelota (m)')
plt.title('Respuesta del Sistema Ball & Beam con Controlador PID')
plt.legend()
plt.grid(True)
plt.show()
