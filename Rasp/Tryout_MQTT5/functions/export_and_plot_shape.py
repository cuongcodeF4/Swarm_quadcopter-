import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np

def export_and_plot_shape(output_file):
    """
    Load the trajectory data from a CSV file, plot the trajectory, and save the plot.

    Parameters
    ----------
    output_file : str
        The path to the CSV file containing the trajectory data.
    """
    # Load the data from the active.csv file
    data = pd.read_csv(output_file)

    # Extract the position coordinates and flight modes
    x = data['px'].to_numpy()
    y = data['py'].to_numpy()
    # z = -1*data['pz'].to_numpy()
    modes = data['mode'].to_numpy()

    # Create a 3D plot
    fig, ax = plt.subplots()


    # Set colormap (you can change these colors to anything you like)
    colors = {0: 'grey', 10: 'orange', 20: 'yellow', 30: 'green', 40: 'blue', 50: 'purple', 60: 'brown', 70: 'red', 80: 'pink', 90: 'cyan', 100: 'black'}

    # Define flight mode names
    mode_names = {0: 'On the ground', 10: 'Initial climbing', 20: 'Initial holding after climb', 30: 'Moving to start point', 40: 'Holding at start point', 50: 'Moving to maneuver start point', 60: 'Holding at maneuver start point', 70: 'Maneuvering (trajectory)', 80: 'Holding at end of trajectory', 90: 'Returning to home', 100: 'Landing'}

    # Plot each segment with the corresponding color
    for mode in np.unique(modes):
        ix = np.where(modes == mode)[0]
        if mode == 30:
            ax.plot(x[ix], y[ix], color=colors[mode], label=mode_names[mode], linestyle='dashed')
        elif mode == 50:
            ax.plot(x[ix], y[ix], color=colors[mode], label=mode_names[mode], linestyle='dotted')
        elif mode == 70:
            ax.plot(x[ix], y[ix], color=colors[mode], label=mode_names[mode], linestyle='dashdot')
        else:
            ax.plot(x[ix], y[ix], color=colors[mode], label=mode_names[mode])

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    ax.set_title('Drone Trajectory')

    # Create legend
    ax.legend(loc='best')
    ax.grid(True)

    # Save the figure before showing it
    plt.savefig('shapes/trajectory_plot.png')

    # Then show the plot
    plt.show()