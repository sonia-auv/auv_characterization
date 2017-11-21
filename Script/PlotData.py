from matplotlib import pyplot as plt
from scipy import ndimage, io
import numpy as np


class PlotData:

    def __init__(self):
        self.data = io.loadmat('speed_x_15.mat')
        self.speed_x_15 = self.data['SPEED'][:, 0]
        self.data = io.loadmat('speed_x_20.mat')
        print self.data['SPEED']
        self.speed_x_20 = self.data['SPEED'][:, 0]
        self.data = io.loadmat('speed_x_25.mat')
        self.speed_x_25 = self.data['SPEED'][:, 0]

        self.X_15 = np.arange(0, len(self.speed_x_15))
        self.X_20 = np.arange(0, len(self.speed_x_20))
        self.X_25 = np.arange(0, len(self.speed_x_25))

        plt.plot(self.X_15, self.speed_x_15)
        plt.plot(self.X_20, self.speed_x_20)
        plt.plot(self.X_25, self.speed_x_25)
        plt.show()

if __name__ == '__main__':
    PlotData()
