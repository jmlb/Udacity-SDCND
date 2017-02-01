import numpy as np
from keras.optimizers import Adam
from keras.layers.core import Dense, Dropout, Activation
from keras.layers import Convolution2D, MaxPooling2D, Flatten, PReLU
from keras.models import Sequential, Model
from keras import backend as K
from keras.regularizers import l2
import os.path
import csv
import cv2
import glob
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import json
from keras import callbacks
import math
from pre_processing import *
SEED = 42



def continuousSteering(img_sz, activation_fn = 'relu', l2_reg=10**-3):
    '''
    steering angle predictor: takes an image and predict the steerign angle value
    img_sz: size of the image that the model accepts (128, 128, 3)
    activation_fn: non-linear function - relu, prelu or elu
    l2_reg - L2 regularization coefficient for fully connected layers
    '''

    # size of pooling area for max pooling
    pool_size = (2, 2)

    model = Sequential()
    
    model.add(Convolution2D(8, 5, 5, subsample=(1, 1), border_mode="valid", name='conv1', input_shape=img_sz))
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    
    model.add(MaxPooling2D(pool_size=pool_size))
    
    model.add(Convolution2D(8, 5, 5, subsample=(1, 1), border_mode="valid") )
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=pool_size))
    
    model.add(Convolution2D(16, 4, 4, subsample=(1, 1), border_mode="valid") )
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=pool_size))

    model.add(Convolution2D(16, 5, 5, subsample=(1, 1), border_mode="valid"))
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))

    model.add(Flatten())
    
    model.add(Dense(128, W_regularizer=l2(l2_reg)))
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    
    model.add(Dense(50, W_regularizer=l2(l2_reg)))
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    
    model.add(Dense(10, W_regularizer=l2(l2_reg)))
    if activation_fn == 'elu':
        model.add(Activation('elu'))
    elif activation_fn == 'prelu':
        model.add(PReLU())
    else:
        model.add(Activation('relu'))
    
    model.add(Dense(1, activation='linear', W_regularizer=l2(l2_reg), init='he_normal'))

    adam = Adam(lr=0.001) #optimizer
    model.compile(optimizer=adam, loss='mean_squared_error')

    print('Lightweight Model is created and compiled..- activation: {}'.format(activation_fn))
    return model




if __name__ == '__main__':

    ############
    # load drive log csvfile
    ############
    data_path = 'data/'
    # open csv file
    with open(data_path + 'driving_log.csv', 'r') as csvfile:
        file_reader = csv.reader(csvfile, delimiter=',')
        log = []
        for row in file_reader:
            log.append(row)

    log = np.array( log )
    log = log[1:,:] #remove the header

    #total number of images accounting for left/right/center
    print('Dataset: \n {} images | Number of steering data: {}'.format(len(log) * 3, len(log)) ) 

    # Sanity check: count all image files in data/IMG folder and compare to number of imgs listed in csv
    ls_imgs = glob.glob(data_path+ 'IMG/*.jpg')
    assert len(ls_imgs) == len(log)*3, 'Actual number of *jpg images does not match with the csv log file'

    #########
    # Parameters
    #########
    test_size = 0.2
    img_sz = (128, 128, 3)
    batch_size = 200
    data_augmentation = 200
    nb_epoch = 15
    del_rate = 0.95
    activation_fn = 'relu'
    l2_reg = 0.00001

    x_ = log[:, 0] 
    y_ = log[:, 3].astype(float)
    x_, y_ = shuffle(x_, y_)
    # split train/validation set with ratio: 5:1
    X_train, X_val, y_train, y_val = train_test_split(x_, y_, test_size=test_size, random_state=SEED)

    print('Total number of samples per EPOCH: {}'.format(batch_size * data_augmentation))
    print('Train set size: {} | Validation set size: {}'.format(len(X_train), len(X_val)))
        
    samples_per_epoch = batch_size * data_augmentation 
    # make validation set size to be a multiple of batch_size
    nb_val_samples = len(y_val) - len(y_val)%batch_size
    model =  continuousSteering( img_sz, activation_fn = activation_fn, l2_reg=l2_reg)
    print(model.summary())

    '''
    Callbacks: save best and early stop - based on validation loss value
    1. Save the model after each epoch if the validation loss improved, or
    2. stop training and save model if the validation loss doesn't improve for 5 consecutive epochs.
    '''
    model_path = os.path.expanduser('model.h5')
    save_best = callbacks.ModelCheckpoint(model_path, monitor='val_loss', verbose=1, 
                                         save_best_only=True, mode='min')
    early_stop = callbacks.EarlyStopping(monitor='val_loss', min_delta=0, patience=15, 
                                         verbose=0, mode='auto')
    callbacks_list = [early_stop, save_best]

    # batch generator default value: 
    #training=True, del_rate=0.95, data_dir='data/', monitor=True
    history = model.fit_generator(batch_generator(X_train, y_train, batch_size, img_sz, training=True, del_rate=del_rate),
                                  samples_per_epoch=samples_per_epoch,
                                  nb_val_samples=nb_val_samples,
                                  validation_data=batch_generator(X_val, y_val, batch_size, img_sz, 
                                                                  training=False, monitor=False),
                                  nb_epoch=nb_epoch, verbose=1, callbacks=callbacks_list
    
                                 )
    # save model architecture and weights at the end of the training
    with open('model.json', 'w') as f:
            f.write( model.to_json() )
    model.save('model_.h5')
    print('Model saved!')

    #clear session to avoid error at the end of program: "AttributeError: 'NoneType' object has no attribute 'TF_DeleteStatus'"
    # The alternative does not work: import gc; gc.collect()
    # https://github.com/tensorflow/tensorflow/issues/3388
    K.clear_session()