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

        self.speed_1 = ManageData('x_10_3.mat', 'LINEAR_SPEED', 0)
        self.speed_2 = ManageData('x_20_1.mat', 'LINEAR_SPEED', 0)
        self.speed_3 = ManageData('x_25_1.mat', 'LINEAR_SPEED', 0)
        self.speed_4 = ManageData('x_30_1.mat', 'LINEAR_SPEED', 0)

        self.thrust_1 = ManageData('x_10_2.mat', 'Thrust', 0)
        self.thrust_2 = ManageData('x_20_2.mat', 'Thrust', 0)
        self.thrust_3 = ManageData('x_25_3.mat', 'Thrust', 0)
        self.thrust_4 = ManageData('x_30_1.mat', 'Thrust', 0)

        self.data_filter()

        self.thrust_1_rescale = self.rescale_data(self.thrust_1.data, len(self.thrust_1.data) / len(self.speed_1.data))
        self.thrust_2_rescale = self.rescale_data(self.thrust_2.data, len(self.thrust_2.data) / len(self.speed_2.data))
        self.thrust_3_rescale = self.rescale_data(self.thrust_3.data, len(self.thrust_3.data) / len(self.speed_3.data))
        self.thrust_4_rescale = self.rescale_data(self.thrust_4.data, len(self.thrust_4.data) / len(self.speed_4.data))

        thrust_1, speed_1 = self.thrust_1_rescale, self.speed_1.data
        thrust_2, speed_2 = self.thrust_2_rescale, self.speed_2.data
        thrust_3, speed_3 = self.thrust_3_rescale, self.speed_3.data
        thrust_4, speed_4 = self.thrust_4_rescale, self.speed_4.data

        self.x_speed_1 = self.set_x_axis(len(speed_1))
        self.x_speed_2 = self.set_x_axis(len(speed_2))
        self.x_speed_3 = self.set_x_axis(len(speed_3))
        self.x_speed_4 = self.set_x_axis(len(speed_4))

        self.x_thrust_1 = self.set_x_axis(len(thrust_1))
        self.x_thrust_2 = self.set_x_axis(len(thrust_2))
        self.x_thrust_3 = self.set_x_axis(len(thrust_3))
        self.x_thrust_4 = self.set_x_axis(len(thrust_4))

        self.plot_data(self.x_speed_1, speed_1, 'x Speed with 10% thrust')
        #self.plot_data(self.x_thrust_1, thrust_1, 'Thrust 10%')
        #plt.show()
        self.plot_data(self.x_speed_2, speed_2, 'x Speed with 20% thrust')
        #self.plot_data(self.x_thrust_2, thrust_2, 'Thrust 25%')
        #plt.show()
        self.plot_data(self.x_speed_3, speed_3, 'x Speed with 25% thrust')
        #self.plot_data(self.x_thrust_3, thrust_3, 'Thrust 25%')
        #plt.show()
        self.plot_data(self.x_speed_4, speed_4, 'x Speed with 30% thrust')
        #self.plot_data(self.x_thrust_4, thrust_4, 'Thrust 30%')
        plt.show()
        
        #self.save_data(thrust_1, speed_1, 'x_rescale_10_1.mat')
        #self.save_data(thrust_2, speed_2, 'x_rescale_10_2.mat')
        #self.save_data(thrust_3, speed_3, 'x_rescale_10_3.mat')

    def save_data(self, thrust, speed, file_name):
        dic = {'Thrust': thrust, 'LINEAR_SPEED': speed}
        io.savemat(file_name, dic)

    def plot_data(self, x, y, label):
        plt.plot(x, y, label=label)
        plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)

    def data_filter(self):
        self.speed_1.filter_data()
        self.speed_2.filter_data()
        self.speed_3.filter_data()
        self.speed_4.filter_data()

        self.thrust_1.filter_data()
        self.thrust_2.filter_data()
        self.thrust_3.filter_data()
        self.thrust_4.filter_data()

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
