from matplotlib import pyplot as plt
from scipy import io
import numpy as np


class ManageData:
    def __init__(self, file_name, var_name):
        data = io.loadmat(file_name)
        self.Y = data[var_name][0]


class LowPassFilter:
    LPF_Beta = 0.15

    def __init__(self):
        data = ManageData('data.mat', 'CMD_Z')
        self.Y = data.Y
        self.low_pass()

    def low_pass(self):
        out = []
        SmoothData = 0
        for y in self.Y:
            SmoothData = SmoothData - (self.LPF_Beta * (SmoothData - y))
            out.append(SmoothData)
        X = np.arange(0, len(out), 1)
        plt.plot(X, out)
        plt.plot(X, self.Y)
        plt.show()


if __name__ == '__main__':
    LowPassFilter()
