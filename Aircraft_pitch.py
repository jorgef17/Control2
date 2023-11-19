import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# Definición de parámetros del sistema
A = np.array([[-0.313, 56.7, 0], [-0.0139, -0.426, 0], [0, 56.7, 0]])
B = np.array([[0.232], [0.0203], [0]])
C = np.array([[0, 0, 1]])
D = np.array([[0]])

# Controlador LQR
p = 2
Q = p * np.dot(C.T, C)
R = 1
K, _, _ = ctl.lqr(A, B, Q, R)

# Sistema en lazo cerrado
sys_cl = ctl.ss(A - np.dot(B, K), B, C, D)

# Simulación de la respuesta del sistema
t = np.linspace(0, 30, 1000)  # Vector de tiempo
plt.figure()
_, y = ctl.step_response(0.2 * sys_cl, t)
plt.plot(t, y)
plt.ylabel('pitch angle (rad)')
plt.title('Closed-Loop Step Response: LQR')
plt.grid(True)

# Ajustar Q y recalcular K
p = 50
Q = p * np.dot(C.T, C)
K, _, _ = ctl.lqr(A, B, Q, R)

# Sistema en lazo cerrado con nuevo K
sys_cl = ctl.ss(A - np.dot(B, K), B, C, D)

# Simulación de la respuesta del sistema con nuevo K
t = np.linspace(0, 3, 1000)  # Vector de tiempo
plt.figure()
_, y = ctl.step_response(0.2 * sys_cl, t)
plt.plot(t, y)
plt.ylabel('pitch angle (rad)')
plt.title('Closed-Loop Step Response: LQR with adjusted Q')
plt.grid(True)

# Calcular Nbar
Nbar = -np.linalg.inv(C @ np.linalg.inv(A - B @ K) @ B)
print(Nbar)
# Sistema en lazo cerrado con compensación previa
sys_cl = ctl.ss(A - B @ K, B * Nbar, C, D)

# Simulación de la respuesta del sistema con compensación previa
t = np.linspace(0, 3, 1000)  # Vector de tiempo
plt.figure()
_, y = ctl.step_response(0.2 * sys_cl, t)
plt.plot(t, y)
plt.ylabel('pitch angle (rad)')
plt.title('Closed-Loop Step Response: LQR with Precompensation')
plt.grid(True)

plt.show()
