import numpy as np
from scipy.optimize import minimize
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

# Example EDM as given in the document (measured with delays set to zero)
edm_measured = np.array([
    [0, 162.1613, 162.2531],
    [162.1720, 0, 162.2449],
    [162.2155, 162.2582, 0]
])

edm_measured = average_matrix

print(f"Measured EDM:\n {edm_measured}")

# Actual distances (all devices equidistant at 7.914 meters)
edm_actual = np.array([
    [0, 7.914, 7.914],
    [7.914, 0, 7.914],
    [7.914, 7.914, 0]
])

# Convert distances to time using the speed of light in air (~299,702,547 meters/second)
def distances_to_times(distances):
    speed_of_light = 299702547  # m/s
    return distances / speed_of_light * 1e9  # converting meters to nanoseconds

# Convert actual EDM to times
tof_actual = distances_to_times(edm_actual)
tof_measured = distances_to_times(edm_measured)

print(f"Expected antenna delays: {tof_measured}")

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

result = dual_annealing(objective_function, ((450.0,550.0),(450.0,550.0),(450.0,550.0)), args=(tof_measured, tof_actual))

#Print the results
if result.success:
    antenna_delays = result.x
    print(f"Antenna delays: {antenna_delays}")
else:
    print("Optimization failed")

# Validation with the example results from the document
#objective_function(initial_delays, tof_measured, tof_actual)
expected_delays = np.array([514.4747, 514.5911, 515.0413])
print(f"Expected antenna delays: {expected_delays}")
print(f"Difference: {antenna_delays - expected_delays}")
