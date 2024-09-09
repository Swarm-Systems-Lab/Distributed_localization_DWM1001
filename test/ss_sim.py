import numpy as np

class DecaSim:
	def __init__(self, id, n):
		self.id = id
		self.centroids = np.zeros((n,2))
		self.positions = np.array([[i, i] for i in range(1, n + 1)])

	def update_centroid(self, n):
		sum_v = 0
		for i in range(n):
			sum_v += self.centroids[self.id] - self.centroids[i] - (self.positions[self.id] - self.positions[i])
		self.centroids[self.id] += -0.9/n * sum_v

		return self.centroids[self.id]
	
	def clean(self, n):
		self.centroids = np.zeros((n,2))

n = 3
Devices = [DecaSim(i, n) for i in range(n)]

for i in range(n):
	print(f'Pos id {i} value {Devices[i].positions}')
	for j in range(n):
		if i != j:
			Devices[i].positions[j] = Devices[j].positions[j]
for k in range(10):
	for i in range(n):
		for j in range(n):
			if i != j:
				Devices[i].centroids[j] = Devices[j].centroids[j]
	for i in range(n):
		c_centr = Devices[i].positions[Devices[i].id] - Devices[i].update_centroid(n)
		print(f'Centroid iteration {k} id {i} value {c_centr}')
	