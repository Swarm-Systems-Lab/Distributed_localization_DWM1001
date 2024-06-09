import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Read and parse the input file
def read_points(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    frames = []
    frame = []
    for line in lines:
        line = line.strip()
        if line:
            x, y = map(float, line.split(','))
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
    ax.clear()
    x, y = zip(*frame)
    ax.scatter(x, y)
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_title('Time: {}'.format(update.frame_number))
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

# Show the animation
plt.show()
