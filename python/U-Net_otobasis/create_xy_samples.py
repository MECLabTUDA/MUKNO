import h5py
import numpy as np
import os
from resample import xy_resampler as data_resampler
import tools

#Trains the model based on our generator returns a model
def create_samples(num_classes, forFirstDataSet, data_src_dir, model_dir) :
	if not os.path.isdir( model_dir ):
		os.mkdir(model_dir)
	
	x, y, img_infos = data_resampler(num_classes, data_src_dir, forFirstDataSet)			
	
	sizes, spacings, origins, directions = tools.serialize_img_infos(img_infos)
		
	if forFirstDataSet:
		h5_filename = model_dir + 'dataset1.h5'
	else:
		h5_filename = model_dir + 'dataset2.h5'
	print("write to " + h5_filename)
	with h5py.File(h5_filename, 'w') as h5file:
		print("   write infos")
		h5file.create_dataset('sizes', data=sizes)
		h5file.create_dataset('spacings', data=spacings)
		h5file.create_dataset('origins', data=origins)
		h5file.create_dataset('directions', data=directions)
		print("   write images")
		h5file.create_dataset('images', data=x)
		print("   write labels")
		h5file.create_dataset('labels', data=y)
	
# ===========================================================================
# \brief create an itk image from a numpy array
def create_xy_samples():
	num_classes = 11
	data_dir  = "./Data/"
	model_dir = "C:/users/jfauser/IPCAI2019/ModelData/XY-Unet-Init/"
	
	forFirstDataSet = True #Wether to use dataset 1[TRUE] or 2[FALSE]
	create_samples(num_classes, forFirstDataSet,     data_dir, model_dir)
	create_samples(num_classes, not forFirstDataSet, data_dir, model_dir)