import numpy as np
import matplotlib.pyplot as plt

class AdaptivePIDController:
    def __init__(self, kp_initial, ki_initial, kd_initial, setpoint):
        self.kp = kp_initial
        self.ki = ki_initial
        self.kd = kd_initial
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0

        # Parámetros de adaptación (ajustados)
        self.alpha =  0.0005 # Tasa de adaptación kp (reducida)
        self.beta =  0.0005  # Tasa de adaptación ki (reducida)
        self.gamma =  0.01 # Tasa de adaptación kd (reducida)

        # Listas para almacenar los valores de ganancias a lo largo del tiempo
        self.kp_values = []
        self.ki_values = []
        self.kd_values = []

    def update(self, feedback):
        error = self.setpoint - feedback

        # Actualizar la integral y el término derivativo
        self.integral += error
        derivative = error - self.prev_error

        # Actualizar las ganancias del controlador
        self.kp += self.alpha * error
        self.ki += self.beta * self.integral
        self.kd += self.gamma * derivative

        # Almacenar los valores de ganancias a lo largo del tiempo
        self.kp_values.append(self.kp)
        self.ki_values.append(self.ki)
        self.kd_values.append(self.kd)

        # Calcular la señal de control
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Actualizar el error previo
        self.prev_error = error

        return control_signal

# Simulación de un proceso controlado
def simulate_process(controller, target_time):
    time = 0
    feedback = 0

    # Listas para almacenar los valores a lo largo del tiempo
    time_values = []
    setpoint_values = []
    feedback_values = []
    control_signal_values = []

    while time < target_time:
        control_signal = controller.update(feedback)

        # Simular el proceso (aquí puedes sustituirlo por tu propio modelo de proceso)
        process_output = control_signal * 0.5

        # Simular el feedback del proceso (ruido + respuesta del proceso)
        feedback = process_output + np.random.normal(0, 0.1)

        # Almacenar valores para graficar
        time_values.append(time)
        setpoint_values.append(controller.setpoint)
        feedback_values.append(feedback)
        control_signal_values.append(control_signal)

        time += 1

    # Graficar los resultados
    plt.figure(figsize=(12, 10))

    plt.subplot(3, 1, 1)
    plt.plot(time_values, setpoint_values, label="Setpoint")
    plt.plot(time_values, feedback_values, label="Feedback")
    plt.xlabel("Tiempo")
    plt.ylabel("Valor")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_values, control_signal_values, label="Control Signal")
    plt.xlabel("Tiempo")
    plt.ylabel("Señal de Control")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_values, controller.kp_values, label="kp")
    plt.plot(time_values, controller.ki_values, label="ki")
    plt.plot(time_values, controller.kd_values, label="kd")
    plt.xlabel("Tiempo")
    plt.ylabel("Ganancias")
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    initial_kp = 0.01
    initial_ki = 0.09
    initial_kd = 0.009
    setpoint_value = 25.0

    adaptive_controller = AdaptivePIDController(initial_kp, initial_ki, initial_kd, setpoint_value)
    simulate_process(adaptive_controller, target_time=100)
