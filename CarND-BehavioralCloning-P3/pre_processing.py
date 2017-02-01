import numpy as np
from keras.optimizers import SGD, RMSprop, Adagrad, Adam
from keras.layers.core import Dense, Dropout, Activation
from keras.layers import Input, Convolution2D, MaxPooling2D, AveragePooling2D, Flatten, PReLU
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
SEED = 42




def horizontal_flip(img, label):
    '''
    Randomly flip image along horizontal axis: 1/2 chance that the image will be flipped
    img: original image in array type
    label: steering angle value of the original image
    '''
    choice = np.random.choice([0,1])
    if choice == 1:
        img, label = cv2.flip(img, 1), -label
    
    return (img, label)


def transf_brightness(img, label):
    '''
    Adjust the brightness of the image, by a randomly generated factor between 0.1 (dark) and 1. (unchanged)
    img: original image in array type
    label: steering angle value of the original image
    '''
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #change Value/Brightness/Luminance: alpha * V
    alpha = np.random.uniform(low=0.1, high=1.0, size=None)
    v = hsv[:,:,2]
    v = v * alpha
    hsv[:,:,2] = v.astype('uint8')
    #min_val = np.min(hsv[:,:,2])
    #max_val = np.max(hsv[:,:,2])
    #print('min:{} | max: {}'.format(min_val, max_val))
    rgb = cv2.cvtColor(hsv.astype('uint8'), cv2.COLOR_HSV2RGB)
    
    return (rgb, label)


def center_RightLeft_swap(img_adress, label, label_corr = 1.0/4):
    '''
    With equal probability, change the camera view point to left, right or center.
    Add a correction to the original steering angle value
    return the new image and corresponding corrected steering angle
    img_adres: physical location of the original image file
    label: steering angle value of the original image
    label_corr: correction of the steering angle to be applied. default value= 1.5/10
    Assume distance center-to-left or center-to-right is 1.5m, 
    and that the car recovers from the track error within 10m
    '''
    swap = np.random.choice(['L', 'R', 'C'])

    if swap == 'L':
        img_adress = img_adress.replace('center', 'left')
        corrected_label = np.arctan( math.tan(label) + label_corr )
        return (img_adress, corrected_label)
    
    elif swap == 'R':
        img_adress = img_adress.replace('center', 'right')
        corrected_label = np.arctan( math.tan(label) - label_corr )
        return (img_adress, corrected_label)
    
    else:
        return (img_adress, label)

    
def filter_zero_steering(label, del_rate):
    '''
    Randomly pick examples with steering angle of 0, and return their index
    label: list of steering angle value in the original dataset
    del_rate: rate of deletion - del_rate=0.9 means delete 90% of the example with steering angle=0
    '''
    steering_zero_idx = np.where(label == 0)
    steering_zero_idx = steering_zero_idx[0]
    size_del = int( len(steering_zero_idx) * del_rate )
    
    return np.random.choice(steering_zero_idx, size=size_del, replace=False)


def image_transformation(img_adress, label, data_dir):
    # Image swap at random: left-right-center
    img_adress, label = center_RightLeft_swap(img_adress, label )
    # Read img file and convert to RGB
    img = cv2.imread(data_dir + img_adress)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # change luminance: 50% chance
    img, label = transf_brightness(img, label)
    #flip image: 50% chance
    img, label = horizontal_flip(img, label)

    return (img, label)


def batch_generator(x, y, batch_size, img_sz, training=True, del_rate=0.95, data_dir='data/', monitor=True, yieldXY=True):
    """
    Generate training batch: Yield X and Y data when the batch is filled.
    Data augmentation schemes: horizontal flip, chnage brightness, change viewpoint
    At each EPOCH, before data augmentation, del_rate=95% of examples with 
    steering angle = 0 are deleted from original dataset
    x: list of the adress of all images to be used for training
    y: steering angles
    training: use True for generating training batch with data augmentation scheme 
              use False to generate validation batch
    batch_size: size of the batch (X, Y)
    img_sz: size of the image (height, width, channel) to generate
    del_rate: percent of examples with steering angle=0 that will be deleted before generating batches
    data_dir: adress to the directory IMG/*.jpg
    monitor: save X of the last batch generated 'X_batch_sample.npy
             save angles of all batches generated 'y_bag.npy
    yieldXY: if True, generator yields (X, Y)
            otherwise, yields X only (useful for predict_generator()
    """
    
    if training:
        y_bag = []
        x, y = shuffle(x, y)
        rand_zero_idx = filter_zero_steering(y, del_rate=del_rate)
        new_x = np.delete(x, rand_zero_idx, axis=0)
        new_y = np.delete(y, rand_zero_idx, axis=0)
        new_x = x
        new_y = y
    else:
        new_x = x
        new_y = y
    offset = 0
    '''
    True as long as total number of examples generated is lower than the number of 'samples_per_epoch' set by user.
    '''
    while True: 
        # Initialize X and Y array
        X = np.empty((batch_size, *img_sz))
        Y = np.empty((batch_size, 1))
        #Generate a batch
        for example in range(batch_size):
            img_adress, img_steering = new_x[example + offset], new_y[example + offset]
            assert os.path.exists(data_dir + img_adress), 'Image file ['+ img_adress +'] not found-'
            
            if training:
                img, img_steering = image_transformation(img_adress, img_steering, data_dir)
            else:
                img = cv2.imread(data_dir + img_adress)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # crop / resize image / scale pixel intensity
            # update batch X and Y array with new example
            X[example,:,:,:] = cv2.resize(img[80:140, 0:320], (img_sz[0], img_sz[1]) ) / 255. - 0.5
            
            Y[example] = img_steering
            if training:
                y_bag.append(img_steering)
            
            '''
             when reaching end of original dataset x, loop from start again
             shuffle original dataset / randomly remove 95% of the example with steering angle = 0
             
            '''
            if (example + 1) + offset > len(new_y) - 1:
                x, y = shuffle(x, y)
                rand_zero_idx = filter_zero_steering(y, del_rate=del_rate)
                new_x = x
                new_y = y
                new_x = np.delete(new_x, rand_zero_idx, axis=0)
                new_y = np.delete(new_y, rand_zero_idx, axis=0)
                offset = 0
        if yieldXY:
            yield (X, Y)
        else:
            yield X

        offset = offset + batch_size
        if training:
            np.save('y_bag.npy', np.array(y_bag) )
            np.save('Xbatch_sample.npy', X ) #save last batch of images