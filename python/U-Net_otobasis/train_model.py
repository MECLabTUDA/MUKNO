import h5py
import keras
import SimpleITK as sitk
import numpy as np
import os
import datetime

#Trains the model based on our generator returns a model
def train_model(trainSet, outputFile, model, batch_size, epochs):	
	start = datetime.datetime.now()
	
	checkpoint = keras.callbacks.ModelCheckpoint(outputFile, monitor='dice_coeff', verbose=2, save_best_only=True, mode='max')#For saving
	callbacks_list = [checkpoint]#, early]
		
	model.fit(
		x=trainSet[0],
		y=trainSet[1],
		callbacks=callbacks_list,
		epochs=epochs,
		batch_size=batch_size)

	model.save(outputFile + '_')
	now   = datetime.datetime.now()
	c     = now - start
	print("Trained in {} hours".format(c.total_seconds()/3600.0))
	
	