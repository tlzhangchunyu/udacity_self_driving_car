import csv
import cv2
import numpy as np

lines=[]

with open('base_data/driving_log.csv') as csvfile:
    reader=csv.reader(csvfile)
    
    for line in reader:
        lines.append(line)
        
images=[]
measurements=[]
for line in lines:
    for i in range(3):
        source_path=line[i]
        filename=source_path.split('/')[-1]    
        current_path='base_data/IMG/'+filename    
        image=cv2.imread(current_path)
        image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        images.append(image)
        if i==0:
            measurement=float(line[3])
        if i==1:
            measurement=float(line[3])+0.2
        if i==2:
            measurement=float(line[3])-0.2
        
        measurements.append(measurement)

augmented_images, augmented_measurements=[], []
for image,measurement in zip(images,measurements):
    augmented_images.append(image)
    augmented_measurements.append(measurement)
    augmented_images.append(cv2.flip(image,1))
    augmented_measurements.append(measurement*-1.0)
    
X_train=np.array(augmented_images)
y_train=np.array(augmented_measurements)

print(len(X_train))
print(len(y_train))

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout, ELU, Activation
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D

model=Sequential()
model.add(Lambda(lambda x: (x/255.0)-0.5, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((70,25),(0,0))))
model.add(Convolution2D(24,5,5, subsample=(2,2), activation="elu"))
model.add(Convolution2D(36,5,5, subsample=(2,2), activation="elu"))
model.add(Convolution2D(48,5,5, subsample=(2,2), activation="elu"))
model.add(Convolution2D(64,3,3,activation="elu"))
model.add(Convolution2D(64,3,3,activation="elu"))

model.add(Flatten())

model.add(Dense(100))
model.add(Activation('elu'))
model.add(Dropout(.3))
model.add(Dense(50))
model.add(Activation('elu'))
model.add(Dropout(.2))
model.add(Dense(10))
model.add(Dense(1))


model.compile(loss='mse', optimizer='adam')
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=5)

model.save('model.h5')
exit()