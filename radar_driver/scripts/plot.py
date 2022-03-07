from collections import defaultdict
from itertools import cycle

import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')
cycol = cycle('bgrcmykw')

def read_output(filename):
    x = defaultdict(list)
    y = defaultdict(list)
    z = defaultdict(list)
    with open(filename) as f:
        for line in f.readlines():
            point = line.split(',')
            x[point[3][:1]].append(float(point[0]))
            y[point[3][:1]].append(float(point[1]))
            z[point[3][:1]].append(float(point[2]))
    f.close()

    return x, y, z

def plot_clusters(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim3d([-5, 5])
    ax.set_ylim3d([-5, 5])
    ax.set_zlim3d([-5, 5])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    for cluster in x:
        ax.scatter(x[cluster], y[cluster], z[cluster], c=next(cycol))
    
    fig.savefig("data/test.png")

if __name__ == "__main__":
    x, y, z = read_output('data/output.txt')
    plot_clusters(x, y, z)
