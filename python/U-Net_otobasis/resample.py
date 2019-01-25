from keras import utils
import numpy as np
import os
import SimpleITK as sitk
from tools import get_filenames

#  \brief returns image parameters (size, spacing, origin, direction) in a single list
#
# img  		-> an sitk image
# returns 	-> a python list with entries for size, spacing, origin and direction
def extract_image_info(img):
	info = []
	info.append(img.GetSize())
	info.append(img.GetSpacing())
	info.append(img.GetOrigin())
	info.append(img.GetDirection())
	return info

#  \brief resamples the entire data set into a tuple [images, labels, infos]
#
#	this is doing the same as in yz_resampler, just lots of differences due to indexing of the other dimension 
#
# images -> array of slices
# labels -> array of slices
# infos  -> array of Type [size, spacing, origin, direction], all 1x3
def xz_resampler(num_classes, data_dir, isFirstDataSet):
	# first retrieve the directory and all ct files corresponding to the first/second data set
	subdir = data_dir + "rawdata/"
	files  = get_filenames(isFirstDataSet, subdir)
	print ("num classes: {}".format(num_classes))
	print("resampling in " + subdir)
	# now read and resample
	batch_in  = []
	img_infos = []
	netdim_z = 128
	for f in files:
		print ("resample ct " + f)
		ctimg = sitk.ReadImage(data_dir + "rawdata/"     + f)
		dim_x = ctimg.GetWidth()
		dim_y = ctimg.GetHeight()
		dim_z = ctimg.GetDepth()
		
		extract_labels = netdim_z > dim_z	# network has more slices than original image
		slices = []
		for y in range(0,dim_y):
			slices.append ( sitk.GetArrayFromImage(ctimg[:,y,:]))
		
		extract_labels = netdim_z > dim_z	# network has more slices than original image
		for slice in slices:
			new_slice = np.zeros( (netdim_z,dim_x,1) )
			if extract_labels:
				# original z-direction small than unet input? just take partial unet segmentation
				idx_start = int(0.5*(netdim_z-dim_z))
				new_slice[idx_start:idx_start+dim_z,:,0] = slice
			else:
				# original z-direction is larger than unet input? put in segmentation
				idx_start = int(max(0, 0.5*dim_z-74)) # jugular vein is slightly lower. so do not pick the middle
				new_slice[:,:,0] = slice[idx_start:idx_start+netdim_z,:]
			batch_in.append(new_slice)
		img_infos.append( extract_image_info(ctimg) )
	
	batch_in  = np.stack(batch_in)
	# now retrieve the directory and all gt files corresponding to the first/second data set		
	subdir = data_dir + "groundtruth/"
	files  = get_filenames(isFirstDataSet, subdir);
	print("resampling in " + subdir)
	# now read and resample
	batch_out = []
	for f in files:
		print ("resample gt " + f)
		gtimg = sitk.ReadImage(data_dir + "groundtruth/" + f)
		dim_x = gtimg.GetWidth()
		dim_y = gtimg.GetHeight()
		dim_z = gtimg.GetDepth()
		
		extract_labels = netdim_z > dim_z	# network has more slices than original image
		slices = []
		for y in range(0,dim_y):
			slices.append ( sitk.GetArrayFromImage(gtimg[:,y,:]))
		
		for slice in slices:
			new_slice = np.zeros( (netdim_z,dim_x) )
			if extract_labels:
				# original z-direction small than unet input? just take partial unet segmentation
				idx_start = int(0.5*(netdim_z-dim_z))
				new_slice[idx_start:idx_start+dim_z,:] = slice
			else:
				# original z-direction is larger than unet input? put in segmentation
				idx_start = int(max(0, 0.5*dim_z-74)) # jugular vein is slightly lower. so do not pick the middle
				new_slice[:,:] = slice[idx_start:idx_start+netdim_z,:]
			batch_out.append(new_slice)
	
	batch_out = np.stack(batch_out)
	print ("size {}".format(batch_in.shape))
	print ("size {}".format(batch_out.shape))
	batch_out = utils.to_categorical(batch_out, num_classes)
	return (batch_in, batch_out, img_infos)	
	
#  \brief resamples the entire data set into a tuple [images, labels, infos]
#
#	this is doing the same as in xz_resampler, just lots of differences due to indexing of the other dimension 
#
# images -> array of slices
# labels -> array of slices
# infos  -> array of Type [size, spacing, origin, direction], all 1x3
def yz_resampler(num_classes, data_dir, isFirstDataSet):
	# first retrieve the directory and all ct files corresponding to the first/second data set
	subdir = data_dir + "rawdata/"
	files  = get_filenames(isFirstDataSet, subdir)
	print ("num classes: {}".format(num_classes))
	print("resampling in " + subdir)
	# now read and resample
	batch_in  = []
	img_infos = []
	netdim_z = 128
	for f in files:
		print ("resample ct " + f)
		ctimg = sitk.ReadImage(data_dir + "rawdata/"     + f)
		dim_x = ctimg.GetWidth()
		dim_y = ctimg.GetHeight()
		dim_z = ctimg.GetDepth()
		
		extract_labels = netdim_z > dim_z	# network has more slices than original image
		slices = []
		for x in range(0,dim_x):
			slices.append ( sitk.GetArrayFromImage(ctimg[x,:,:]))
		
		for slice in slices:
			new_slice = np.zeros( (netdim_z,dim_y,1) )
			if extract_labels:
				# original z-direction small than unet input? just take partial unet segmentation
				idx_start = int(0.5*(netdim_z-dim_z))
				new_slice[idx_start:idx_start+dim_z,:,0] = slice
			else:
				# original z-direction is larger than unet input? put in segmentation
				idx_start = int(max(0, 0.5*dim_z-74)) # jugular vein is slightly lower. so do not pick the middle
				new_slice[:,:,0] = slice[idx_start:idx_start+netdim_z,:]
			batch_in.append(new_slice)
			
		img_infos.append( extract_image_info(ctimg) )
	
	batch_in  = np.stack(batch_in)
	# now retrieve the directory and all gt files corresponding to the first/second data set		
	subdir = data_dir + "groundtruth/"
	files  = get_filenames(isFirstDataSet, subdir);
	
	print("resampling in " + subdir)
	# now read and resample
	batch_out = []
	for f in files:
		print ("resample gt " + f)
		gtimg = sitk.ReadImage(data_dir + "groundtruth/" + f)
		dim_x = gtimg.GetWidth()
		dim_y = gtimg.GetHeight()
		dim_z = gtimg.GetDepth()
		
		extract_labels = netdim_z > dim_z	# network has more slices than original image
		slices = []
		for x in range(0,dim_x):
			slices.append ( sitk.GetArrayFromImage(gtimg[x,:,:]))
		
		for slice in slices:
			new_slice = np.zeros( (netdim_z,dim_y) )
			if extract_labels:
				# original z-direction small than unet input? just take partial unet segmentation
				idx_start = int(0.5*(netdim_z-dim_z))
				new_slice[idx_start:idx_start+dim_z,:] = slice
			else:
				# original z-direction is larger than unet input? put in segmentation
				idx_start = int(max(0, 0.5*dim_z-74)) # jugular vein is slightly lower. so do not pick the middle
				new_slice[:,:] = slice[idx_start:idx_start+netdim_z,:]
			batch_out.append(new_slice)
	
	batch_out = np.stack(batch_out)
	print ("size {}".format(batch_in.shape))
	print ("size {}".format(batch_out.shape))
	batch_out = utils.to_categorical(batch_out, num_classes)
	return (batch_in, batch_out, img_infos)	
		

#  \brief resamples the entire data set into a tuple [images, labels, infos]
#
# images -> array of slices
# labels -> array of slices
# infos  -> array of Type [size, spacing, origin, direction], all 1x3
def xy_resampler(num_classes, data_dir, isFirstDataSet):
	# first retrieve the directory and all ct files corresponding to the first/second data set
	subdir = data_dir + "rawdata/"
	files  = get_filenames(isFirstDataSet, subdir)
		
	print("resampling in " + subdir)
	# now read and resample
	batch_in  = []
	img_infos = []
	for f in files:
		print ("resample ct " + f)
		ctimg = sitk.ReadImage(data_dir + "rawdata/"     + f)
		for z in range(0,ctimg.GetDepth()):
			batch_in.append( np.expand_dims(sitk.GetArrayFromImage(ctimg[:,:,z]), axis=2) )
		img_infos.append( extract_image_info(ctimg) )
			
	# now retrieve the directory and all gt files corresponding to the first/second data set		
	subdir = data_dir + "groundtruth/"
	files  = get_filenames(isFirstDataSet, subdir);
	
	print("resampling in " + subdir)
	# now read and resample
	batch_out = []
	for f in files:
		print ("resample gt " + f)
		gtimg = sitk.ReadImage(data_dir + "groundtruth/" + f)
		for z in range(0,gtimg.GetDepth()):
			batch_out.append( sitk.GetArrayFromImage(gtimg[:,:,z]))
	
	batch_in  = np.stack(batch_in)
	batch_out = np.stack(batch_out)
	print ("size {}".format(batch_in.shape))
	print ("size {}".format(batch_out.shape))
	batch_out = utils.to_categorical(batch_out, num_classes)
	return (batch_in, batch_out, img_infos)	
		