
import numpy as np
import keras
from keras import backend as K
from keras.models import Sequential
from keras.layers import Activation
from keras.layers.core import Dense, Flatten
from keras.optimizers import Adam
from keras.metrics import categorical_crossentropy
from keras.preprocessing.image import ImageDataGenerator
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import *
from sklearn.metrics import confusion_matrix
import itertools
import matplotlib.pyplot as plt
from keras.models import load_model
from keras.models import model_from_json
import cv2
import os,glob
import sys,argparse


vgg16_model = keras.applications.vgg16.VGG16()

model = Sequential()
for layer in vgg16_model.layers:
    model.add(layer)

model.layers.pop()

for layer in model.layers:
    layer.trainable = False

model.add(Dense(2,activation='softmax'))

model.compile(Adam(lr=.0001), loss='categorical_crossentropy', metrics=['accuracy'])
model.load_weights("model_weights_f.h5")
json_file = open('model_architecture1.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)

#print("Loaded model from disk")

K.set_image_data_format('channels_last')
dir_path = os.path.dirname(os.path.realpath(__file__))
image_path=sys.argv[1] 
filename = dir_path +'/' +image_path
im = cv2.resize(cv2.imread(filename), (224, 224)).astype(np.float32)
im[:,:,0] -= 103.939
im[:,:,1] -= 116.779
im[:,:,2] -= 123.68
im = im.transpose((1,0,2))
im = np.expand_dims(im, axis=0)
#tf.reshape(im, [-1, 1])
out = model.predict(im)
#print(out)
m=np.argmax(out) 
if m == 0:
   print "abnormal"
else:
   print "normal"
