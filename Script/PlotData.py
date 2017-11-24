from matplotlib import pyplot as plt
from scipy import io
from scipy.signal import savgol_filter
import numpy as np


class ManageData:
    def __init__(self, file_name, key, axe):
        data = io.loadmat(file_name)
        self.data = data[str(key)][:, axe]

    def filter_data(self):
        self.data = savgol_filter(self.data, 5, 2)


class PlotData:

    def __init__(self):

        self.speed_15 = ManageData('speed_y_15.mat', 'SPEED', 0)
        self.speed_20 = ManageData('speed_y_20.mat', 'SPEED', 0)
        self.speed_25 = ManageData('speed_y_25.mat', 'SPEED', 0)

        self.thrust_15 = ManageData('thrust_y_15.mat', 'SPEED', 0)
        self.thrust_20 = ManageData('thrust_y_20.mat', 'SPEED', 0)
        self.thrust_25 = ManageData('thrust_y_25.mat', 'SPEED', 0)

        self.data_filter()

        self.thrust_15_rescale = self.rescale_data(self.thrust_15.data, len(self.thrust_15.data) / len(self.speed_15.data))
        self.thrust_20_rescale = self.rescale_data(self.thrust_20.data, len(self.thrust_20.data) / len(self.speed_20.data))
        self.thrust_25_rescale = self.rescale_data(self.thrust_25.data, len(self.thrust_25.data) / len(self.speed_25.data))

        thrust_15, speed_15 = self.set_data(self.thrust_15_rescale, self.speed_15.data)
        thrust_20, speed_20 = self.set_data(self.thrust_20_rescale, self.speed_20.data)
        thrust_25, speed_25 = self.set_data(self.thrust_25_rescale, self.speed_25.data)

        self.x_speed_15 = self.set_x_axis(len(speed_15))
        self.x_speed_20 = self.set_x_axis(len(speed_20))
        self.x_speed_25 = self.set_x_axis(len(speed_25))

        self.x_thrust_15 = self.set_x_axis(len(thrust_15))
        self.x_thrust_20 = self.set_x_axis(len(thrust_20))
        self.x_thrust_25 = self.set_x_axis(len(thrust_25))

        self.plot_data(self.x_speed_15, speed_15, 'Y Speed 15% with thrust')
        self.plot_data(self.x_thrust_15, thrust_15, 'Thrust 15%')
        plt.show()
        self.plot_data(self.x_speed_20, speed_20, 'Y Speed 20% with thrust')
        self.plot_data(self.x_thrust_20, thrust_20, 'Thrust 20%')
        plt.show()
        self.plot_data(self.x_speed_25, speed_25, 'Y Speed 25% with thrust')
        self.plot_data(self.x_thrust_25, thrust_25, 'Thrust 25%')
        plt.show()
        
        self.save_data('SPEED', speed_15, 'speed_y_rescale_15.mat')
        self.save_data('SPEED', speed_20, 'speed_y_rescale_20.mat')
        self.save_data('SPEED', speed_25, 'speed_y_rescale_25.mat')

        self.save_data('THRUST', thrust_15, 'thrust_y_rescale_15.mat')
        self.save_data('THRUST', thrust_20, 'thrust_y_rescale_20.mat')
        self.save_data('THRUST', thrust_25, 'thrust_y_rescale_25.mat')

    def save_data(self, Key, data, file_name):
        dic = {Key: data}
        io.savemat(file_name, dic)

    def plot_data(self, x, y, label):
        plt.plot(x, y, label=label)
        plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)

    def data_filter(self):
        self.speed_15.filter_data()
        self.speed_20.filter_data()
        self.speed_25.filter_data()

        self.thrust_15.filter_data()
        self.thrust_20.filter_data()
        self.thrust_25.filter_data()

    def rescale_data(self, data, skip):
        j, new_data = skip, []
        for i in data:
            if j == skip:
                j = 0
                new_data.append(i)
            j += 1
        return new_data

    def set_data(self, thrusts, speeds):
        new_thrust, new_speed, add_data = [], [], False
        good_data = sorted(thrusts, reverse=True)[0]
        mean_thrust = good_data / 2.0
        i = 0
        for thrust, speed in zip(thrusts, speeds):
            if thrust == good_data:
                add_data = True
            if add_data:
                new_speed.append(speed), new_thrust.append(mean_thrust)
            if thrust == 0.0 and add_data:
                i += 1
            if add_data and i == 10:
                add_data = False
        return new_thrust, new_speed

    def set_x_axis(self, length):
        return np.arange(0, length)

if __name__ == '__main__':
    PlotData()
