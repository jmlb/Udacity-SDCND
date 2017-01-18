#!/usr/bin/env python
"""
Udacity Self Driving Car Nanodegree
P3 - Behavioral Cloning
https://github.com/jmlb/Udacity-SDCND
jmbeaujour.com  

Copyright 2016 Visible Energy Inc. All Rights Reserved.
"""

import numpy as np
import sys
import os
#from keras.models import Sequential, Graph
from keras.optimizers import SGD, RMSprop, Adagrad, Adam
from keras.layers.core import Dense, Dropout, Activation
from keras.layers import Input, Convolution2D, MaxPooling2D, AveragePooling2D, Flatten, PReLU
from keras.models import Sequential, Model
from keras import backend as K
import csv
import cv2


def model_continuousSteering():
    # size of pooling area for max pooling
    pool_size = (2, 2)

    model = Sequential()

    model.add(Convolution2D(16, 8, 8, subsample=(4, 4), border_mode="same", input_shape=(row, col, ch), activation='relu'))
    model.add(MaxPooling2D(pool_size=pool_size))

    model.add(Convolution2D(32, 5, 5, subsample=(2, 2), border_mode="same", activation='relu'))
    model.add(MaxPooling2D(pool_size=pool_size))

    model.add(Convolution2D(64, 5, 5, subsample=(2, 2), border_mode="same", activation='relu' ))
    model.add(MaxPooling2D(pool_size=pool_size))

    model.add(Flatten())
    model.add(Dense(256, init='he_normal', activation='relu'))
    model.add(Dropout(.25))

    model.add(Dense(num_outputs, init='he_normal', activation='relu'))

    sgd = RMSprop(lr=0.001)
    model.compile(optimizer=sgd, loss='rms')

    print('Model relu2 is created and compiled..')
    return model


def model_ladderSteering(img_size):
    keep_rate = 0.5
    pool_size = (2, 2)
    print('Number of outputs:', num_outputs)
    img_input = Input(shape= img_size)
    x = Convolution2D(16, 5, 5, subsample=(2, 2), border_mode="same", activation='relu')(img_input)
    x = MaxPooling2D(pool_size=pool_size)(x)
    #x = Dropout(0.5)(x)
    x = Convolution2D(32, 2, 2, subsample=(1, 1), border_mode="same", activation='relu')(x)
    x = MaxPooling2D(pool_size=pool_size)(x)
    x = Flatten()(x)
    x = Dense(128, activation='relu')(x)
    x = Dropout(keep_rate)(x)
    #x = Dropout(0.33)(x)
    o_st = Dense(num_outputs, activation='softmax', name='o_st')(x)
    o_thr = Dense(num_outputs, activation='softmax', name='o_thr')(x)
    model = Model(input=img_input, output=[o_st, o_thr])
    model.compile(optimizer='adam', loss={'o_st': 'categorical_crossentropy', 'o_thr': 'categorical_crossentropy'}, metrics=['accuracy'])

    return model

if __name__ == "__main__":

  try:
    data_path = os.path.expanduser(sys.argv[1])
  except Exception as e:
    print(e, "Usage: ./model.py <DATA-DIR>")
    sys.exit(-1)

  if not os.path.exists(data_path):
    print("Directory %s not found." % data_path)
    sys.exit(-1)

  # open driving log file
  with open(data_path + '/driving_log.csv', 'r') as csvfile:
    file_reader = csv.reader(csvfile, delimiter=',')
    driving_log = []
    for row in file_reader:
      driving_log.append(row)
  # log file includes in order: center_camera|left_camera|right_camera|steering|throttle|brake|speed
  # column of interest are center_camera and steering 
  # create summary log with only img name and steering info - header removed
  driving_log = np.array( driving_log )

  # get column image name and reshape from (nbr_imgs,) to (nbr_imgs, 1)
  # create a 2 column log with imgs/steering

  driving_log_short = np.hstack( (driving_log[1:, 0].reshape((-1,1)), driving_log[1:,3].reshape((-1,1))))

  i=0
  img_i = driving_log_short[i][0]
  print(img_i)
  img = cv2.imread('data/'+img_i[4:])
  print(img.shape)
'''
  driving_log = 
  img_height, img_width, img_depth = 

  num_epoch = 
  batch_size = 
  num_outputs = 1

  

  model = model_continuousSteering()
  print(model.summary())

  print("loading images and labels")
  X = np.load("{}/X_yuv_gray.npy".format(data_path))-0.5
  y1_steering = np.load("{}/y1_steering.npy".format(data_path))
  
  # and trained it via:
  history = model.fit(X, {'o_st': y1_steering, 'o_thr': y2_throttle}, batch_size=batch_size, nb_epoch=100, verbose=1, validation_split=0.30 )
  




  start_val = round(len(X)*0.8)
  X_val = X[start_val:start_val + 200]
  print(y1_steering[0:50])
  y_val = y1_steering[start_val:start_val + 200, :]
  pred_val = np.array( model.predict(X_val, batch_size=batch_size) )
  print(pred_val[0,:,:].shape)
  print(y_val.shape)
  np.save('pred_validation.npy', np.hstack([y_val, pred_val[0,:,:]]))
  print("saving model and weights")
  with open("{}/autonomia_cnn.json".format(data_path), 'w') as f:
      f.write(model.to_json())

  model.save_weights("{}/autonomia_cnn.h5".format(data_path))
  '''
