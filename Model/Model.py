import Config


def create_model(input_shape):
        from keras import Input, models
        from keras.layers import Flatten, Dense, Conv2D, BatchNormalization, AveragePooling2D

        y = Input(shape=input_shape)
        # z = Conv2D(256, kernel_size=(8, 8), activation='relu', data_format='channels_last')(y)
        # z = Conv2D(Config.CONV1_FILTER_NUMBER, kernel_size=Config.CONV1_SIZE, activation=Config.CONV1_ACTIVATION, data_format='channels_last')(y)
        # z = Conv2D(Config.CONV2_FILTER_NUMBER, kernel_size=Config.CONV2_SIZE, activation=Config.CONV2_ACTIVATION, data_format='channels_last')(y)
        # z = BatchNormalization()(z)
        # z = AveragePooling2D(pool_size=(2, 2), padding='valid')(z)
        z = Flatten()(y)
        z = Dense(Config.DENSE1_SIZE, activation=Config.DENSE1_ACTIVATION)(z)
        z = Dense(Config.DENSE2_SIZE, activation=Config.DENSE2_ACTIVATION)(z)

        # our model will accept the inputs of the two branches and
        # then output a single value
        model = models.Model(y, z)

        model.compile(optimizer=Config.OPTIMIZER, loss='mse')
        model.summary()
        return model
