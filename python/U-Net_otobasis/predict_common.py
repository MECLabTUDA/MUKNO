import h5py
import numpy as np
import tools

# ===========================================================================
# \brief adds 0 label slices at bottom and top to create image of original size
# 
# slices 	-> a list of subsequent 2D np arrays of the same size
# info   	-> image info in form of a list, encoding size, spacing, origin and direction
# returns	-> slices that match the original image size
def fill_up_labels(slices, info) :
	dim_z      	   = info[0][2]
	new_slices 	   = np.zeros((512,dim_z,512))
	netdim_z 	   = 128
	extract_labels = netdim_z > dim_z	# network has more slices than original image
	for slice, new_slice in zip(slices, new_slices):
		if extract_labels:
			# original z-direction small than unet input? just take partial unet segmentation
			idx_start = int(0.5*(netdim_z-dim_z))
			new_slice[:,:] = slice[idx_start:idx_start+dim_z,:]
		else:
			# original z-direction is larger than unet input? put in segmentation
			idx_start = int(max(0, 0.5*dim_z-74)) # jugular vein is slightly lower. so do not pick the middle
			new_slice[idx_start:idx_start+netdim_z,:] = slice
	return new_slices;
	
# ===========================================================================
# \brief simple maximum vote 
#
def vote_on_predicted_slices(predictions) :
	ret = np.argmax(predictions, axis=2)
	return ret
	
#
#
#
def retrieve_data(model_base_dir, model_dir, raw_data_dir, predict_first_dataset) :
	if predict_first_dataset:
		dataset_ID = "1"
		model_ID   = "2"
	else:
		dataset_ID = "2"
		model_ID   = "1"
		
	print("loading data set {}".format(dataset_ID))
	if predict_first_dataset:
		file_dataset = model_base_dir + "dataset1.h5"
	else:
		file_dataset = model_base_dir + "dataset2.h5"
	f 			= h5py.File(file_dataset.strip(), "r")
	sizes    	= f["sizes"][()]
	spacings    = f["spacings"][()]
	origins    	= f["origins"][()]
	directions  = f["directions"][()]
	img_infos	= tools.deserialize_img_infos(sizes, spacings, origins, directions)
	images	 	= f["images"][()]
	if predict_first_dataset:
		model_file 	= model_dir + "weights2.hdf5"
	else:
		model_file 	= model_dir + "weights1.hdf5"
	image_filename_templates = tools.get_filenames(predict_first_dataset, raw_data_dir)
	image_filenames = [];
	for f in image_filename_templates:
		new_name = f[:3] + "_prediction.mhd" # eg "P01_prediction.mhd"
		image_filenames.append(new_name) 
	
	print("predict with model {}".format(model_ID))
	return images, img_infos, image_filenames, model_file
