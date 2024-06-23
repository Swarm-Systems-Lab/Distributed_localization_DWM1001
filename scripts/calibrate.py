import numpy as np
from scipy.optimize import minimize
from scipy.optimize import basinhopping
from scipy.optimize import dual_annealing
import numpy as np

def read_and_sum_matrices(filename):
    sum_matrix = np.zeros((3, 3), dtype=np.float64)
    count_matrix = np.zeros((3, 3), dtype=np.int64)

    with open(filename, 'r') as file:
        current_matrix = []
        for line in file:
            if line.strip():
                current_matrix.append([int(x) for x in line.strip().split(',') if x])
                if len(current_matrix) == 3:
                    for i in range(3):
                        for j in range(3):
                            value = current_matrix[i][j]
                            if value >= 0:
                                sum_matrix[i, j] += value
                                count_matrix[i, j] += 1
                    current_matrix = []
    
    return sum_matrix, count_matrix

def calculate_average_matrix(sum_matrix, count_matrix):
    average_matrix = np.divide(sum_matrix, count_matrix, where=(count_matrix != 0))
    return average_matrix


filename = 'matrices.txt'
sum_matrix, count_matrix = read_and_sum_matrices(filename)
average_matrix = calculate_average_matrix(sum_matrix, count_matrix)
average_matrix = average_matrix / 100000

# Example EDM as given in the document (measured with delays set to zero) order 1-5-7
edm_measured = np.array([
    [0, 165.171185, 162.843257],
    [161.132828, 0, 160.384045],
    [163.93291, 166.703973, 0]
])

range_bias = -0.05

id_edm = np.array([
    [0, 1, 1],
    [1, 0, 1],
    [1, 1, 0]
])

#edm_measured = average_matrix

edm_measured = edm_measured + range_bias * id_edm

print(f"Measured EDM:\n {edm_measured}")

# Actual distances (all devices equidistant at 7.914 meters)
edm_actual = 8.0 * id_edm

# Convert distances to time using the speed of light in air (~299,702,547 meters/second)
def distances_to_times(distances):
    c = 299702547  # m/s
    return distances / c * 1e9  # converting meters to nanoseconds

# Convert actual EDM to times
tof_actual = distances_to_times(edm_actual)
tof_measured = distances_to_times(edm_measured)

print(f"actual tof:\n {tof_actual}")
print(f"measured tof:\n {tof_measured}")

# Initial guesses for the antenna delays (in nanoseconds)
initial_delays = np.array([513.0, 513.0, 513.0])

# Objective function to minimize
def objective_function(delays, measured, actual):
	tof_canditate = np.array([
		[0, (4.0*measured[0][1]-2.0*delays[0]-2.0*delays[1])/4.0, (4.0*measured[0][2]-2.0*delays[0]-2.0*delays[2])/4.0],
		[(4.0*measured[1][0]-2.0*delays[0]-2.0*delays[1])/4.0, 0, (4.0*measured[1][2]-2.0*delays[1]-2.0*delays[2])/4.0],
		[(4.0*measured[2][0]-2.0*delays[0]-2.0*delays[2])/4.0, (4.0*measured[2][1]-2.0*delays[2]-2.0*delays[1])/4.0, 0]
	])    

	difference_matrix = actual - tof_canditate
	# Calculate the Frobenius norm of the difference matrix
	norm = np.linalg.norm(difference_matrix)

	return norm

# Minimize the objective function to find the antenna delays
#result = minimize(objective_function, initial_delays, args=(tof_measured, tof_actual), bounds=((400.0,600.0),(400.0,600.0),(400.0,600.0)))
#result = basinhopping(objective_function, initial_delays, minimizer_kwargs={'args':(tof_measured, tof_actual)})

result = dual_annealing(objective_function, ((0.0,10000.0),(0.0,10000.0),(0.0,10000.0)), args=(tof_measured, tof_actual))

#Print the results
if result.success:
    antenna_delays = result.x
    error_value = result.fun
    tof_chosen = np.array([
		[0, (4.0*tof_measured[0][1]-2.0*antenna_delays[0]-2.0*antenna_delays[1])/4.0, (4.0*tof_measured[0][2]-2.0*antenna_delays[0]-2.0*antenna_delays[2])/4.0],
		[(4.0*tof_measured[1][0]-2.0*antenna_delays[0]-2.0*antenna_delays[1])/4.0, 0, (4.0*tof_measured[1][2]-2.0*antenna_delays[1]-2.0*antenna_delays[2])/4.0],
		[(4.0*tof_measured[2][0]-2.0*antenna_delays[0]-2.0*antenna_delays[2])/4.0, (4.0*tof_measured[2][1]-2.0*antenna_delays[2]-2.0*antenna_delays[1])/4.0, 0]
	])
    err_m = tof_actual - tof_chosen
    err_m = err_m/1e9 * 299702547
    dw_values = antenna_delays*63.897763
    dw_values_tx = dw_values*0.44
    dw_values_rx = dw_values*0.56
    print(f"Antenna delays: {antenna_delays}")
    print(f"dw_values_tx: {dw_values_tx}")
    print(f"dw_values_rx: {dw_values_rx}")
    print(f"Error norm: {error_value}")
    print(f"Error: {err_m}")
else:
    print("Optimization failed")

# Validation with the example results from the document
#objective_function(initial_delays, tof_measured, tof_actual)
expected_delays = np.array([514.4747, 514.5911, 515.0413])

print(f"Expected antenna delays: {expected_delays}")
print(f"Difference: {antenna_delays - expected_delays}")
