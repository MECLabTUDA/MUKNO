import h5py
import keras
import numpy as np

from u_net import get_unet_128_512 as unet
from predict_common import retrieve_data, fill_up_labels, vote_on_predicted_slices
import tools
import sitk_tools

# ===========================================================================
# \brief segments an entire data set
#
# infos  -> array of Type [size, spacing, origin, direction], all 1x3
def predict_xz(images, img_infos, filenames, model_file, out_dir, num_classes) :
	print("predicting")
	for f, info in zip(filenames, img_infos):
		print ("  image " + f)
	
	print("")
	print("with model " + model_file)
	print("writing to " + out_dir)
	input_shape = (128, 512)
	model 		= unet(input_shape + (1,), num_classes)
	print("  loading model.. ")
	model.load_weights(model_file)
	print("  finished. Predicting.. ")
	predictions = model.predict(images, batch_size = 8, verbose=1)
	print("finished")
	
	# devide into images
	idx = 0
	for info, file in zip(img_infos, filenames):
		dim = info[0][1]
		img_predictions = predictions[idx:idx+dim] # predictions for whole image
		print("size of predictions: {}".format(img_predictions.shape))
		labeled_slices  = []
		for slice_predictions in img_predictions: # num_classes predictions for one slice
			labeled_slice = vote_on_predicted_slices(slice_predictions) # make a labeled slice out of num_classes slices
			labeled_slices.append(labeled_slice)
		labeled_slices = fill_up_labels(labeled_slices, info)
		labeled_img = np.stack(labeled_slices, axis=1)
		print("size of voting result: {}".format(labeled_img.shape))
		label_image = sitk_tools.create_itk_image(labeled_img, info)
		sitk_tools.write_img(label_image, out_dir + file);
		idx += dim	
		
		
# ===========================================================================
# \brief segments an entire data set
#
# infos  -> array of Type [size, spacing, origin, direction], all 1x3
def predict_yz(images, img_infos, filenames, model_file, out_dir, num_classes) :
	print("predicting")
	for f, info in zip(filenames, img_infos):
		print ("  image " + f)
	
	print("")
	print("with model " + model_file)
	print("writing to " + out_dir)
	input_shape = (128, 512)
	model 		= unet(input_shape + (1,), num_classes)
	model.load_weights(model_file)
	predictions = model.predict(images, batch_size = 8, verbose=1)
	print("finished")
	
	# devide into images
	idx = 0
	for info, file in zip(img_infos, filenames):
		dim = info[0][0] # dim_x
		img_predictions = predictions[idx:idx+dim] # predictions for whole image
		print("size of predictions: {}".format(img_predictions.shape))
		labeled_slices  = []
		for slice_predictions in img_predictions: # num_classes predictions for one slice
			labeled_slice = vote_on_predicted_slices(slice_predictions) # make a labeled slice out of num_classes slices
			labeled_slices.append(labeled_slice)
		labeled_slices = fill_up_labels(labeled_slices, info)
		labeled_img = []
		for slice in labeled_slices:
			img_slice = slice.reshape(info[0][1],info[0][2])
			labeled_img.append(img_slice)
		labeled_img = np.stack(labeled_slices, axis=2)
		print("size of voting result: {}".format(labeled_img.shape))
		label_image = sitk_tools.create_itk_image(labeled_img, info)
		sitk_tools.write_img(label_image, out_dir + file);
		idx += dim	
		
# ===========================================================================
# \brief predict on dataset
#
# model_base_dir 		-> folder with the weights
# raw_data_dir			-> folder with the h5 file data
# out_dir				-> folder for the output
# predict_first_dataset -> flag indicating which data set to predict
# num_classes			-> number of label classes
def predict_on_dataset_xz(model_base_dir, model_dir, raw_data_dir, out_dir, predict_first_dataset, num_classes) :
	images, img_infos, image_filenames, model_file = retrieve_data(model_base_dir, model_dir, raw_data_dir, predict_first_dataset)
	predict_xz(images, img_infos, image_filenames, model_file, out_dir, num_classes)	# segment the whole set
	keras.backend.clear_session() #stuff

# ===========================================================================
# \brief the same
#
def predict_on_dataset_yz(model_base_dir, model_dir, raw_data_dir, out_dir, predict_first_dataset, num_classes) :
	images, img_infos, image_filenames, model_file = retrieve_data(model_base_dir, model_dir, raw_data_dir, predict_first_dataset)
	predict_yz(images, img_infos, image_filenames, model_file, out_dir, num_classes)	# segment the whole set
	keras.backend.clear_session() #stuff