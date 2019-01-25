import numpy as np
import os
import datetime
from keras import utils

import train_model
import tools
from u_net import get_unet_512 as unet

# ===========================================================================
# \brief
def train_xy_init_model(num_classes, epochs):
	# first get the time
	now     	= datetime.datetime.now()
	now_str 	= now.strftime('%Y-%m-%d_%H-%M-%S')		
	print("Started at {}".format(now_str))	
	# then create a new model folder based on name and date
	model_name 	= "XY-Unet-Init"
	rootDir 	= "C:/users/jfauser/IPCAI2019/ModelData/" + model_name + "/"
	out_dir     = rootDir + now_str + "/"
	if not os.path.isdir(out_dir):
		os.mkdir(out_dir)
		
	# compile the net architecture
	batch_size  = 16
	input_shape = (512, 512)
	model 		= unet(input_shape + (1,), num_classes)
	
	# load the two training sets
	print("loading data set 1")
	if num_classes == 2:
		images_1, labels_1 = tools.get_data_set_for_chorda(rootDir + "dataset1.h5")
	else:
		images_1, labels_1 = tools.get_data_set(rootDir + "dataset1.h5")
		
	print("loading data set 2")
	if num_classes == 2:
		images_2, labels_2 = tools.get_data_set_for_chorda(rootDir + "dataset2.h5")
	else:
		images_2, labels_2 = tools.get_data_set(rootDir + "dataset2.h5")
	
	# and train
	filename_weigths1 = out_dir + "weights1.hdf5"
	filename_weigths2 = out_dir + "weights2.hdf5"
	
	print("train model 1")
	train_model.train_model( [images_1, labels_1], filename_weigths1, model, batch_size, epochs )	# commences training on the set
	del model
	
	print("train model 2")
	model 		= unet(input_shape + (1,), num_classes)
	train_model.train_model( [images_2, labels_2], filename_weigths2, model, batch_size, epochs )	# commences training on the set
	del model
	
	return now_str