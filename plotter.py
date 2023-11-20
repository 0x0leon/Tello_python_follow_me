import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Create a figure and axis
fig, ax = plt.subplots()
x_data = np.linspace(0, 2 * np.pi, 100)
y_data = np.sin(x_data)

# Create a line plot that will be updated during animation
line, = ax.plot(x_data, y_data, label='Sin(x)')

# Function to update the plot in each animation frame
def update(frame):
    # Update data for the plot
    new_x = x_data + 0.01 * frame
    new_y = np.sin(new_x)

    # Update the plot with new data
    line.set_data(new_x, new_y)

    # Optionally, update the axis limits for a dynamic view
    ax.set_xlim(min(new_x), max(new_x))
    ax.set_ylim(min(new_y), max(new_y))

    return line,

# Create the animation
animation = FuncAnimation(fig, update, frames=100, interval=50, blit=True)

# Show the plot
plt.show()
