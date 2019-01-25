import tools 
import sitk_tools as itk 
import os
import numpy as np

# ===========================================================================
# \brief 
#
def test_combined_models(model_date_xy, model_date_xz, model_date_yz):
	#This needs to be set according to your directory
	#Needs to contain /rawdata and /groundtruth with equally named mhd-files
	root_dir   = "C:/users/jfauser/IPCAI2019/ModelData/"
	model_xy = "XY-Unet-Init"
	model_xz = "XZ-Unet-Init"
	model_yz = "YZ-Unet-Init"
	
	model_dir_xy  = root_dir + model_xy + "/" + model_date_xy + "/predictions/"
	model_dir_xz  = root_dir + model_xz + "/" + model_date_xz + "/predictions/"
	model_dir_yz  = root_dir + model_yz + "/" + model_date_yz + "/predictions/"
	
	#  ------------------------ majority vote ------------------------
	useMajorityVoting = True
	if useMajorityVoting:
		model_name 	= "Combined-Unet-Majority"
		# time string as a changing last folder does not makes sense here
		out_dir 	= root_dir + model_name + "/last_result/predictions/"
		if not os.path.isdir(root_dir + model_name):
			os.mkdir(root_dir + model_name)
		if not os.path.isdir(root_dir + model_name + "/last_result/"):
			os.mkdir(root_dir + model_name + "/last_result/")
		if not os.path.isdir(out_dir):
			os.mkdir(out_dir)
		
		# get all results from all models
		files_xy = tools.get_filenames(True, model_dir_xy)
		files_xy.extend(tools.get_filenames(False, model_dir_xy))
		files_xz = tools.get_filenames(True, model_dir_xz)
		files_xz.extend(tools.get_filenames(False, model_dir_xz))	
		files_yz = tools.get_filenames(True, model_dir_yz)
		files_yz.extend(tools.get_filenames(False, model_dir_yz))
		# sort for ease of use
		files_xy.sort()
		files_xz.sort()
		files_yz.sort()
		
		for fx,fy,fz in zip(files_xy, files_xz, files_yz):
			print ("voting on next file: {}".format(fx[:3]))
			img1, info = itk.read_itk_array(model_dir_xy + fx)
			size = info[0]
			img2, info = itk.read_itk_array(model_dir_xz + fy)
			img3, info = itk.read_itk_array(model_dir_yz + fz)
			img  = np.zeros( (size[2], size[1], size[0]) )		
			
			# majority vote
			equals = img1[:,:,:] == img2[:,:,:]
			img[equals] = img1[equals]
			equals = img1[:,:,:] == img3[:,:,:]
			img[equals] = img1[equals]
			equals = img3[:,:,:] == img2[:,:,:]
			img[equals] = img2[equals]
			# chorda vote 
			equals = img1[:,:,:] == 5
			img[equals] = img1[equals]
			equals = img1[:,:,:] == 5
			img[equals] = img1[equals]
			equals = img3[:,:,:] == 5
			img[equals] = img2[equals]
			
			img_out = itk.create_itk_image_2(img, info)
			itk.write_img(img_out, out_dir + fx[:3] + ".mhd")
			
			# ===========================================================================
# \brief 
#
def test_combined_models_plus_chorda(last_all, model_date_xy, model_date_xz, model_date_yz):
	#This needs to be set according to your directory
	#Needs to contain /rawdata and /groundtruth with equally named mhd-files
	root_dir   = "C:/users/jfauser/IPCAI2019/ModelData/"
	model_xy = "XY-Unet-Init"
	model_xz = "XZ-Unet-Init"
	model_yz = "YZ-Unet-Init"
	model_name 	= "Combined-Unet-Majority"
	
	model_dir_all = root_dir + model_name + "/" + last_all + "/predictions/"
	model_dir_xy  = root_dir + model_xy + "/" + model_date_xy + "/predictions/"
	model_dir_xz  = root_dir + model_xz + "/" + model_date_xz + "/predictions/"
	model_dir_yz  = root_dir + model_yz + "/" + model_date_yz + "/predictions/"
	print("chorda combined")
	#  ------------------------ majority vote ------------------------
	useMajorityVoting = True
	if useMajorityVoting:
		# time string as a changing last folder does not makes sense here
		out_dir 	= root_dir + model_name + "/last_result_plus_chorda/"
		if not os.path.isdir(root_dir + model_name):
			os.mkdir(root_dir + model_name)
		if not os.path.isdir(out_dir):
			os.mkdir(out_dir)
			
		# get combined results
		files_all = tools.get_filenames(True, model_dir_all)
		files_all.extend(tools.get_filenames(False, model_dir_all))
		# get all results from all models
		files_xy = tools.get_filenames(True, model_dir_xy)
		files_xy.extend(tools.get_filenames(False, model_dir_xy))
		files_xz = tools.get_filenames(True, model_dir_xz)
		files_xz.extend(tools.get_filenames(False, model_dir_xz))	
		files_yz = tools.get_filenames(True, model_dir_yz)
		files_yz.extend(tools.get_filenames(False, model_dir_yz))
		# sort for ease of use
		files_all.sort()
		files_xy.sort()
		files_xz.sort()
		files_yz.sort()
		
		for f in files_all:
			print ("voting on next file: {}".format(f[:3]))
		
		for f, fx,fy,fz in zip(files_all, files_xy, files_xz, files_yz):
			print ("voting on next file: {}".format(fx[:3]))
			img, info  = itk.read_itk_array(model_dir_all + f)
			img1, info = itk.read_itk_array(model_dir_xy + fx)
			size = info[0]
			img2, info = itk.read_itk_array(model_dir_xz + fy)
			img3, info = itk.read_itk_array(model_dir_yz + fz)
			
			# majority vote
			# chorda vote 
			equals = img1[:,:,:] == 1
			img[equals] = 5
			equals = img2[:,:,:] == 1
			img[equals] = 5
			equals = img3[:,:,:] == 1
			img[equals] = 5
			
			img_out = itk.create_itk_image_2(img, info)
			itk.write_img(img_out, out_dir + fx[:3] + ".mhd")
# ===========================================================================
# \brief 
#		
if __name__ == "__main__":
	test_combined_models()