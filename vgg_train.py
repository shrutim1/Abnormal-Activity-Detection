from keras.models import Sequential
from keras.layers.core import Flatten, Dense, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D
from keras.optimizers import SGD, Adam
import cv2
import numpy as np
import h5py
from keras import backend as k
from keras.preprocessing import image
import sys
import os
from scipy.misc import imread
import theano
import theano.tensor as T
import glob

k.set_image_data_format('channels_first')
k.set_image_dim_ordering('th')
seed = 128
rng = np.random.RandomState(seed)

def pop_layer(model):
    if not model.outputs:
        raise Exception('Sequential model cannot be popped: model is empty.')

    model.layers.pop()
    if not model.layers:
        model.outputs = []
        model.inbound_nodes = []
        model.outbound_nodes = []
    else:
        model.layers[-1].outbound_nodes = []
        model.outputs = [model.layers[-1].output]
    model.built = False

def VGG_16(weights_path=None):
    model = Sequential()
    model.add(ZeroPadding2D((1,1),input_shape=(3,224,224)))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(128, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(128, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1,1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Flatten())
    model.add(Dense(4096, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(4096, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(1000, activation='softmax'))

    if weights_path:
        model.load_weights(weights_path,by_name=True)
    return model

	

if __name__ == "__main__":
    
    model = VGG_16('vgg16_weights.h5')
    pop_layer(model)
   #pop_layer(model)
    #pop_layer(model)
   
    #model.add(Dense(2,activation='relu'))
    model.add(Dense(2,activation='softmax'))
    for layer in model.layers[:14]:
        layer.trainable = False
    sgd = SGD(lr=0.0001, decay=1e-6, momentum=0.9, nesterov=True)
    adam = Adam(lr=0.0001)
    model.compile(optimizer=adam, loss='sparse_categorical_crossentropy', metrics = ['categorical_accuracy'])
    #model.compile(optimizer=sgd, loss='binary_crossentropy', metrics = ['accuracy'])
    train_img=[]
    train_label=[]
    #Positive stands for normal(1) and negative for abnormal(0)
    train_positive=0;
    train_negative=0;

    train_path = "TrainData/S*/*.jpg"
    files = glob.glob(train_path)

    for file in files:
      temp_img=image.load_img(file,target_size=(224,224))

      temp_img=image.img_to_array(temp_img)

      train_img.append(temp_img)

      if file.find("_A1_") >=0 or file.find("_A2_") >=0 or file.find("_A3_") >=0 or file.find("_A4_") >=0 or file.find("_A5_") >=0 or file.find("_A6_") >=0 or file.find("_A7_") >=0 or file.find("_A8_") >=0 or file.find("_A9_") >=0:
	train_label.append(1);
        train_positive+=1

      elif file.find("_A10_") >=0 or file.find("_A11_") >=0:
	train_label.append(0);
        train_negative+=1

      else:
	print("Label error in " + file)

    X_train=np.array(train_img)
    print("Training: Positive labels: "+ str(train_positive) + " negative labels: "+ str(train_negative) + " Total labels: "+ str(len(train_label)));
    print("Training: Total images: "+ str(len(train_img)));
	


    

    test_img=[]
    test_label=[]
    #Positive stands for normal(1) and negative for abnormal(0)
    test_positive=0;
    test_negative=0;

    test_path = "TestData/S*/*.jpg"
    files = glob.glob(test_path)

    for file in files:
      temp_img=image.load_img(file,target_size=(224,224))

      temp_img=image.img_to_array(temp_img)

      test_img.append(temp_img)

      if file.find("_A1_") >=0 or file.find("_A2_") >=0 or file.find("_A3_") >=0 or file.find("_A4_") >=0 or file.find("_A5_") >=0 or file.find("_A6_") >=0 or file.find("_A7_") >=0 or file.find("_A8_") >=0 or file.find("_A9_") >=0:
	test_label.append(1);
        test_positive+=1

      elif file.find("_A10_") >=0 or file.find("_A11_") >=0:
	test_label.append(0);
        test_negative+=1

      else:
	print("Label error in " + file)

    X_test=np.array(test_img)
    print("Testing: Positive labels: "+str(test_positive)+ " negative labels: "+ str(test_negative) + " Total labels: "+str(len(test_label)));
    print("Testing: Total images: "+ str(len(test_img)));
   

   
    Y_train = np.asarray(train_label)
    Y_test = np.asarray(test_label)
   
    model.fit(X_train, Y_train, epochs=2)
    score = model.evaluate(X_test,Y_test,verbose=1)
    print("%s; %.2f%%" %(model.metrics_names[1],score[1]*100))
    model.save('model_train2.h5')
    
 
