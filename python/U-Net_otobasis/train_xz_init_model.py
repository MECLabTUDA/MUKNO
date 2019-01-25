import h5py
import numpy as np
import os
import datetime
import train_model

from u_net import get_unet_128_512 as unet

from keras import utils

# ===========================================================================
# \brief 
def train_xz_init_model(num_classes, epochs):
	#This needs to be set according to your directory
	#Needs to contain /rawdata and /groundtruth with equally named mhd-files
	model_name = "XZ-Unet-Init"
	# create a new model folder based on name and date
	now     = datetime.datetime.now()
	now_str = now.strftime('%Y-%m-%d_%H-%M-%S')
		
	rootDir = "C:/users/jfauser/IPCAI2019/ModelData/" + model_name + "/"
			
	print("Started at {}".format(now_str))
	
	input_shape = (128, 512)
	model 		= unet(input_shape + (1,), num_classes)
	batch_size  = 8
		
	print("loading data set 1")
	images_1, labels_1 = get_data_set(rootDir + "dataset1.h5")
	print("loading data set 2")
	images_2, labels_2 = get_data_set(rootDir + "dataset2.h5")
	
	out_dir           = rootDir + now_str + "/"
	filename_weigths1 = out_dir + "weights1.hdf5"
	filename_weigths2 = out_dir + "weights2.hdf5"
	
	if not os.path.isdir(out_dir):
		os.mkdir(out_dir)
		
	print("train model 1")
	train_model.train_model( [images_1, labels_1], filename_weigths1, model, batch_size, epochs )	# commences training on the set
	del model
	
	print("train model 2")
	model 		= unet(input_shape + (1,), num_classes)
	train_model.train_model( [images_2, labels_2], filename_weigths2, model, batch_size, epochs )	# commences training on the set
	del model
	
# ===========================================================================
# \brief train a 2D Unet slice by slice for the chorda (label 5)
def train_xz_init_model_chorda(num_classes, epochs):
	model_name = "XZ-Unet-Init"
	# create a new model folder based on name and date
	now     = datetime.datetime.now()
	now_str = now.strftime('%Y-%m-%d_%H-%M-%S')
		
	rootDir = "C:/users/jfauser/IPCAI2019/ModelData/" + model_name + "/"
			
	print("Started at {}".format(now_str))
	
	input_shape = (128, 512)
	model 		= unet(input_shape + (1,), num_classes)
	batch_size  = 8
		
	print("loading data set 1")
	file_dataset1 = rootDir + "dataset1.h5"
	f1 = h5py.File(file_dataset1.strip(), "r")
	images_1 = f1["images"][()]
	all_labels = f1["labels"][()] # already in to_categorical
	slices = []
	for set in all_labels:
		slices.append(set[:,:,5]) #chorda label
	slices   = np.stack(slices)
	labels_1 = utils.to_categorical(slices, num_classes)
	
	print("loading data set 2")
	file_dataset2 = rootDir + "dataset2.h5"
	f2 = h5py.File(file_dataset2.strip(), "r")
	images_2 = f2["images"][()]
	all_labels = f2["labels"][()]
	slices = []
	for set in all_labels:
		slices.append(set[:,:,5]) #chorda label
	slices   = np.stack(slices)
	labels_2 = utils.to_categorical(slices, num_classes)
	
	out_dir           = rootDir + now_str + "/"
	filename_weigths1 = out_dir + "weights1.hdf5"
	filename_weigths2 = out_dir + "weights2.hdf5"
	
	if not os.path.isdir(out_dir):
		os.mkdir(out_dir)
		
	print("train model 1")
	train_model.train_model( [images_1, labels_1], filename_weigths1, model, batch_size, epochs )	# commences training on the set
	del model
	
	print("train model 2")
	model 		= unet(input_shape + (1,), num_classes)
	train_model.train_model( [images_2, labels_2], filename_weigths2, model, batch_size, epochs )	# commences training on the set
	del model