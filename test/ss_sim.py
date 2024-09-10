import numpy as np


def potential_at_point(x_point, y_point, k=1):
    # Calculate the distance r from the center to the point
    r = np.sqrt((x_point - 10) ** 2 + (y_point - 10) ** 2)
    
    # Handle the case where the point is exactly at the center
    if r == 0:
        return np.inf  # or some large value, e.g., 1e9
    
    # Calculate the potential
    phi = k / r
    
    return phi

class DecaSim:
	def __init__(self, id, n):
		self.id = id
		self.centroids = np.zeros((n,2))
		self.asc_dir = np.zeros((n,2))
		self.field_value = np.random.uniform(10, 100)
		# self.positions = np.array([[i, i] for i in range(1, n + 1)])
		self.positions = np.array([[1, 1], [1, 2], [2, 1]])

	def update_centroid(self, n):
		sum_v = np.zeros(2)
		for i in range(n):
			sum_v += self.centroids[self.id] - self.centroids[i] - (self.positions[self.id] - self.positions[i])
		self.centroids[self.id] += -0.75/n * sum_v

		return self.centroids[self.id]
	
	def update_asc_dir(self, n):
		sum_v = np.zeros(2)
		for i in range(n):
			sum_v += self.asc_dir[i]
		self.asc_dir[self.id] = 0.75/n * sum_v

		return self.asc_dir[self.id]
	
	def clean(self, n):
		Devices[i].centroids = np.zeros((n,2))
		Devices[i].asc_dir = np.zeros((n,2))

n = 3
Devices = [DecaSim(i, n) for i in range(n)]
c_centr = np.zeros(2)

for l in range(3):
	print(f'ITER {l}\n')
	for i in range(n):
		# print(f'Pos id {i} value {Devices[i].positions}')
		for j in range(n):
			if i != j:
				Devices[i].positions[j] = Devices[j].positions[j]

	for i in range(n):
		c_centr = Devices[i].centroids[i]
		Devices[i].clean(n)
		Devices[i].field_value = potential_at_point(Devices[i].positions[i][0], Devices[i].positions[i][1])
		Devices[i].asc_dir[i] = Devices[i].field_value * c_centr	
		print(f'field value id {i} value {Devices[i].field_value}')
	for k in range(10):
		for i in range(n):
			for j in range(n):
				if i != j:
					Devices[i].centroids[j] = Devices[j].centroids[j]
					Devices[i].asc_dir[j] = Devices[j].asc_dir[j]
		for i in range(n):
			c_centr = Devices[i].update_centroid(n)
			c_asc_dir = Devices[i].update_asc_dir(n)
			c_asc_dir = c_asc_dir/np.linalg.norm(c_asc_dir)
			# if i == 0:
			print(f'Centroid iteration {k} id {i} value {Devices[i].positions[Devices[i].id] - c_centr}')
			print(f'Asc dir iteration {k} id {i} value {c_asc_dir}')
		