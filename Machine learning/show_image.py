from scipy.io import loadmat, savemat
from matplotlib import pyplot as plt
from sklearn.model_selection import train_test_split


data = loadmat('Data_img.mat')

X, Y = data['X'], data['Y']

X = X.reshape(X.shape[0], 150, 150, 3)
print Y.shape

Y = Y.reshape(2874)

XTrain, XTest, YTrain, YTest = train_test_split(X, Y, test_size=0.2)

#dic1 = {'X': XTrain.reshape(XTrain.shape[0], 67500), 'Y': YTrain}
#dic2 = {'X': XTest.reshape(XTest.shape[0], 67500), 'Y': YTest}
#savemat('DataTrain.mat', dic1)
#savemat('DataTest.mat', dic2)

for img, label in zip(XTrain, YTrain):
    if label == 3:
        plt.imshow(img), plt.title(label)
        plt.show()
