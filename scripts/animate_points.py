import matplotlib.pyplot as plt
import matplotlib.animation as animation

max_x = 0.0
min_x = 500.0
max_y = 0.0

# Read and parse the input file
def read_points(file_path):
	global max_x
	global min_x
	global max_y

	with open(file_path, 'r') as file:
		lines = file.readlines()

	frames = []
	frame = []
	for line in lines:
		line = line.strip()
		if line:
			x, y = map(float, line.split(','))
			x = x/100.0
			y = y/100.0
			if (x > max_x):
				max_x = x
			if (x < min_x):
				min_x = x
			if (y > max_y):
				max_y = y
			frame.append((x, y))
		else:
			if frame:
				frames.append(frame)
				frame = []

	# Append the last frame if the file does not end with a newline
	if frame:
		frames.append(frame)

	return frames

# Update function for animation
def update(frame):
	global max_x
	global min_x
	global max_y

	ax.clear()
	x, y = zip(*frame)
	ax.scatter(x, y)
	ax.scatter(4.2, 3.6)
	ax.scatter(0, 3.6)
	ax.scatter(4.2, 0)

  	# Draw accuracy err for each point
	circles = []
	radius = 3.16
	for i in range(len(x)-1):
		i += 1
		circle = plt.Circle((x[i], y[i]), radius, linestyle='--', fill=False)
		circles.append(circle)
		ax.add_patch(circle)

	ax.grid()
	ax.set_xlabel('X (m)')  # Set x-axis label with '(m)' unit
	ax.set_ylabel('Y (m)')  # Set y-axis label with '(m)' unit
	ax.set_xlim(min_x-1, max_x+1)
	ax.set_ylim(-1, max_y+1)
	ax.set_title('Relative localization')
	update.frame_number += 1

update.frame_number = 0

# File path to the input file
file_path = 'points.txt'

# Read points from the file
frames = read_points(file_path)

# Create a figure and axis for plotting
fig, ax = plt.subplots()

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=frames, repeat=False, interval=500)

writervideo = animation.FFMpegWriter(fps=5) 
ani.save('dis_loc.mp4', writer=writervideo) 

# Show the animation
#plt.show()
