import pandas as pd
import numpy as np

# Carregar os dados
pid_errors = pd.read_csv('/home/gab/lrs_ws/src/Matrice100_dev/pid_position_error.txt', header=None, names=['x', 'y', 'z', 'yaw'])
dmrac_errors = pd.read_csv('/home/gab/lrs_ws/src/Matrice100_dev/dmrac_position_error.txt', header=None, names=['x', 'y', 'z', 'yaw'])

# Calcular métricas para o erro de posição X
pid_x_error = pid_errors['x']
dmrac_x_error = dmrac_errors['x']

# Erro Médio Quadrático (MSE)
mse_pid = np.mean(pid_x_error**2)
mse_dmrac = np.mean(dmrac_x_error**2)

# Erro Absoluto Médio (MAE)
mae_pid = np.mean(np.abs(pid_x_error))
mae_dmrac = np.mean(np.abs(dmrac_x_error))

# Desvio Padrão
std_pid = np.std(pid_x_error)
std_dmrac = np.std(dmrac_x_error)

# Resultados
print("PID MSE:", mse_pid)
print("DMRAC MSE:", mse_dmrac)
print("PID MAE:", mae_pid)
print("DMRAC MAE:", mae_dmrac)
print("PID Std Dev:", std_pid)
print("DMRAC Std Dev:", std_dmrac)
