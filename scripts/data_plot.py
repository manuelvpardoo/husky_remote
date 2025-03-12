import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_2d_path(df):
    # Extract x and y coordinates
    x = df.iloc[:,1].values
    y = df.iloc[:, 2].values

    # Plot 2D path
    plt.scatter(x, y, marker='o')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Path')
    plt.show()


def main():
    # Load CSV file
    df = pd.read_csv('/home/huanyu/rosbag_out/path_data.csv')
    plot_2d_path(df)


if __name__ == "__main__":
    main()