import numpy as np
import matplotlib.pyplot as plt

# Definición de parámetros
zeta = 0.59219
omega_n = 1.2865
omega_d = omega_n * np.sqrt(1 - zeta**2)

# Definición del tiempo
t = np.linspace(0, 10, 1000)

# Cálculo de la respuesta temporal
y = 1 - np.exp(-zeta * omega_n * t) * (np.cos(omega_d * t) + (zeta/np.sqrt(1-zeta**2)) * np.sin(omega_d * t))

# Gráfica
plt.plot(t, y)
plt.title('Respuesta Temporal del circuito LRC a una entrada de escalón unitario')
plt.xlabel('Tiempo (s)')
plt.ylabel('Respuesta')
plt.grid(True)
plt.show()
