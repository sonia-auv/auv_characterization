from keras.models import Sequential
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.layers.core import Activation, Flatten, Dense
from keras.layers import Dropout


class CNN:

    def __init__(self, Width, Height, Depth, total_classes):
        # Initialize the Model
        self.model = Sequential()

        self.model.add(Conv2D(32, 5, 5, border_mode="same", input_shape=(Width, Height, Depth)))
        self.model.add(Activation("relu"))
        self.model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        self.model.add(Conv2D(64, 5, 5, border_mode="same"))
        self.model.add(Activation("relu"))
        self.model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        self.model.add(Conv2D(64, 5, 5, border_mode="same"))
        self.model.add(Activation("relu"))
        self.model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        self.model.add(Flatten())
        self.model.add(Dropout(0.5))
        self.model.add(Dense(2040))
        self.model.add(Activation("sigmoid"))

        self.model.add(Dropout(0.5))
        self.model.add(Dense(500))
        self.model.add(Activation("sigmoid"))

        self.model.add(Dropout(0.5))
        self.model.add(Dense(total_classes))
        self.model.add(Activation("softmax"))

    def cnn_model(self):
        return self.model