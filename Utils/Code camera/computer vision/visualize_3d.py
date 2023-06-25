import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_3d_points(x, y, z):
    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the 3D points
    ax.scatter(x, y, z)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Points Visualization')

    # Show the plot
    plt.show()

if __name__ == '__main__':
    # Define the 3D points
    x = np.array([0, 1, 2, 3, 4, 5])
    y = np.array([0, 1, 2, 3, 4, 5])
    z = np.array([0, 1, 2, 3, 4, 5])

    # Plot the 3D points
    plot_3d_points(x, y, z)