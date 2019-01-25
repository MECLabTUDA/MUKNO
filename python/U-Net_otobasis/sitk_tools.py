import numpy as np
import SimpleITK as sitk

# ===========================================================================
# \brief writes an itk image to the disc
def write_img(img, filename) :
	print("write segmentation to {}".format(filename))
	sitk.WriteImage(img, filename, False)
		
# ===========================================================================
# \brief create an itk image from a numpy array
#
# \param info : 
def create_itk_image(slices, info) :
	size 		= info[0]
	spacing 	= info[1]
	origin  	= info[2]
	direction 	= info[3]
	img = sitk.GetImageFromArray(slices.astype(np.uint16)) #converts to the needed format
	#print( "size: {}".format(img.GetSize()))
	img.SetOrigin(origin.astype(np.float))
	img.SetSpacing(spacing.astype(np.float))
	img.SetDirection(direction.astype(np.float))
	return img
	
# ===========================================================================
# \brief create an itk image from a numpy array
#
# \param info : 
def create_itk_image_2(img, info) :
	size 		= info[0]
	spacing 	= info[1]
	origin  	= info[2]
	direction 	= info[3]
	img = sitk.GetImageFromArray(img.astype(np.uint16)) #converts to the needed format
	img.SetOrigin(origin)
	img.SetSpacing(spacing)
	img.SetDirection(direction)
	return img
	
# ===========================================================================
# \brief read an itk image and return the numpy array
#
def read_itk_array(filename):
	img  = sitk.ReadImage(filename)
	info = []
	info.append(img.GetSize())
	info.append(img.GetSpacing())
	info.append(img.GetOrigin())
	info.append(img.GetDirection())
	return sitk.GetArrayFromImage(img), info