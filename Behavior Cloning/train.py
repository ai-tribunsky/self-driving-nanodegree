import csv
import imageio
import math
import numpy as np
import os.path
from zipfile import ZipFile

import sklearn
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

import tensorflow as tf
from keras import Sequential
from keras.callbacks import EarlyStopping
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Conv2D, MaxPooling2D


# data consts
DATA_DIR = './data/'
DATA_DRIVING_LOG = DATA_DIR + 'driving_log.csv'
VALIDATION_SET_RATE = 0.3
BATCH_SIZE = 33

# data augmentation consts
SIDE_CAMERA_ANGLE_ADJUST = 0.2

# data processing consts
IMAGE_SIZE = (160, 320, 3)
CROP_MARGINS = ((70, 25), (0, 0))

# data training consts
EPOCHS = 5


def pre_processing(x):
    return x / 127.5 - 1.

def generator(data_dir, samples, batch_size, angle_adjust):
    num_samples = len(samples)
    while 1:  # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size // 3):
            batch_samples = samples[offset:offset + batch_size // 3]

            images = []
            angles = []
            for batch_sample in batch_samples:
                center_image_path = data_dir + '/' + batch_sample[0].strip()
                left_image_path = data_dir + '/' + batch_sample[1].strip()
                right_image_path = data_dir + '/' + batch_sample[2].strip()

                center_angle = float(batch_sample[3])
                left_angle = center_angle + angle_adjust
                right_angle = center_angle - angle_adjust

                images.extend([
                    imageio.imread(center_image_path),
                    imageio.imread(left_image_path),
                    imageio.imread(right_image_path)
                ])
                angles.extend([center_angle, left_angle, right_angle])

            # trim image to only see section with road
            X_train = np.array(images, dtype=np.float32)
            y_train = np.array(angles, dtype=np.float32)
            yield sklearn.utils.shuffle(X_train, y_train)

# extract required data
if not os.path.exists(DATA_DIR):
    with ZipFile('data.zip', 'r') as arch:
        arch.extractall(DATA_DIR)

with open(DATA_DRIVING_LOG, 'r') as driving_log:
    reader = csv.reader(driving_log)
    header = next(reader)  # skip header
    samples = list(reader)

train_samples, validation_samples = train_test_split(samples, test_size=VALIDATION_SET_RATE)

samples_count = len(samples)
train_samples_count = len(train_samples)
validation_samples_count = len(validation_samples)
print('Total Samples=%d; Train Samples=%d; Validation Samples=%d' % (
    samples_count, train_samples_count, validation_samples_count))

# Train Model
train_generator = generator(DATA_DIR, train_samples, BATCH_SIZE, SIDE_CAMERA_ANGLE_ADJUST)
validation_generator = generator(DATA_DIR, validation_samples, BATCH_SIZE, SIDE_CAMERA_ANGLE_ADJUST)

model = Sequential()
model.add(Cropping2D(cropping=CROP_MARGINS, input_shape=IMAGE_SIZE))
model.add(Lambda(pre_processing))

model.add(Conv2D(
    filters=24,
    kernel_size=(5, 5),
    strides=(2, 2),
    activation='relu'
))
model.add(Conv2D(
    filters=36,
    kernel_size=(5, 5),
    strides=(2, 2),
    activation='relu'
))
model.add(Conv2D(
    filters=48,
    kernel_size=(5, 5),
    strides=(2, 2),
    activation='relu'
))
model.add(Conv2D(
    filters=64,
    kernel_size=(3, 3),
    activation='relu'
))
model.add(Conv2D(
    filters=64,
    kernel_size=(3, 3),
    activation='relu'
))

model.add(Flatten())
model.add(Dense(1164, activation='relu'))
model.add(Dense(100, activation='relu'))
model.add(Dense(50, activation='relu'))
model.add(Dense(10, activation='relu'))
model.add(Dense(1))
print(model.summary())

early_stop = EarlyStopping(monitor='val_loss', patience=10)
model.compile(loss='mse', optimizer='rmsprop', metrics=['mae', 'mse'])
model.fit_generator(
    train_generator,
    validation_data=validation_generator,
    steps_per_epoch=math.ceil(train_samples_count / BATCH_SIZE),
    validation_steps=math.ceil(validation_samples_count / BATCH_SIZE),
    epochs=EPOCHS,
    callbacks=[early_stop],
    verbose=2
)
model.save('model.h5')
