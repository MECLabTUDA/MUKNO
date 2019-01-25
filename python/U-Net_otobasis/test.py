#import keras
import SimpleITK as sitk
import numpy as np
import os
import itertools
from u_net import get_unet_128 as unet


def segment_128cube(arr):
	ret = np.zeros((128,128,128,11))
	for slice in range(128):
		prediction = model.predict(arr[slice,:,:].reshape(1,128,128,1))
		ret[slice,:,:] = np.maximum(prediction, ret[slice,:,:])
		prediction = model.predict(arr[:,slice,:].reshape(1,128,128,1))
		ret[:,slice,:,:] = np.maximum(prediction, ret[:,slice,:,:])
		prediction = model.predict(arr[:,:,slice].reshape(1,128,128,1))
		ret[:,:,slice,:] = np.maximum(prediction, ret[:,:,slice,:])
	return ret


def incrange(s,e,i,ss):
	ret = s
	while ret+i < e-i:
		yield ret
		ret += ss
	yield e-i


def bbox(inputimg):
	img = inputimg==5
	print img.shape
	r = np.any(img, axis=(1, 2))
	c = np.any(img, axis=(0, 2))
	z = np.any(img, axis=(0, 1))

	rmin, rmax = np.where(r)[0][[0, -1]]
	cmin, cmax = np.where(c)[0][[0, -1]]
	zmin, zmax = np.where(z)[0][[0, -1]]

	out = [rmin, rmax, cmin, cmax, zmin, zmax]
	for i in [0,2,4]:
		toggle = True
		while out[i+1]-out[i] < 128:
			toggle = not toggle
			if toggle and out[i+1] < img.shape[2-int(i/2)]:
				out[i+1] += 1
			if not toggle and out[i] > 0:
				out[i] -=1	
	#print(out[1]-out[0],out[3]-out[2],out[5]-out[4])			
	return tuple(out)

def segment_specific(sitkImg):
	(width, height, depth) = sitkImg.GetSize()
	imgarr = sitk.GetArrayFromImage(sitkImg)
	ret = np.zeros((depth,height,width,11))
	weights = os.listdir(cwd+weight_dir+"Specialized"+data_dir[-2:])[-1]
	model.load_weights(weight_dir+data_dir+weights)
	for x in incrange(0,width,128,32):
		for y in incrange(0,height,128,32):
			for z in incrange(0,depth,128,32):
				ret[z:z+128,y:y+128,x:x+128,:] = np.maximum(ret[z:z+128,y:y+128,x:x+128,:],segment_128cube(imgarr[z:z+128,y:y+128,x:x+128]))
	weights = os.listdir(cwd+weight_dir+data_dir)[-1]
	model.load_weights(weight_dir+data_dir+weights)
	return np.argmax(ret, axis=3)

def segment_cubed(sitkImg):
	(width, height, depth) = sitkImg[0].GetSize()
	imgarr = sitk.GetArrayFromImage(sitkImg[0])
	ret = np.zeros((depth,height,width,11))
	i = 0
	if depth >= 128:
		for x in incrange(0,width,128,64):
			for y in incrange(0,height,128,64):
				for z in incrange(0,depth,128,32):
					i += 1
					if i%10 is 0:
						print(i)
					ret[z:z+128,y:y+128,x:x+128,:] = np.maximum(ret[z:z+128,y:y+128,x:x+128,:],segment_128cube(imgarr[z:z+128,y:y+128,x:x+128]))
	else:
		print("Resampling did not work")
	print("{} cubes used".format(i))
	ret = np.argmax(ret,axis=3)
	#bb = bbox(ret)
	#ret[bb[0]:bb[1]+1,bb[2]:bb[3]+1,bb[4]:bb[5]+1] = segment_specific(ret[bb[0]:bb[1]+1,bb[2]:bb[3]+1,bb[4]:bb[5]+1])
	ret = sitk.GetImageFromArray(ret.astype(np.uint16))
	ret.SetOrigin(sitkImg[0].GetOrigin())
	ret.SetSpacing(sitkImg[0].GetSpacing())
	ret.SetDirection(sitkImg[0].GetDirection())
	print("\tSaving to {}".format(cwd+pr_dir+sitkImg[1]))
	sitk.WriteImage(ret, pr_dir+sitkImg[1])
	return ret



def resample(img, size, is_label):
	ori_spacing = img.GetSpacing()
	ori_size = img.GetSize()
	
	out_spacing = [oldsize*space/newsize for newsize,oldsize,space in zip(size, img.GetSize(), img.GetSpacing())]
	
	sampler = sitk.ResampleImageFilter()
	sampler.SetOutputSpacing(out_spacing)
	sampler.SetSize(size)
	sampler.SetOutputDirection(img.GetDirection())
	sampler.SetOutputOrigin(img.GetOrigin())
	
	sampler.SetTransform(sitk.Transform())
	sampler.SetDefaultPixelValue(img.GetPixelIDValue())
	
	if is_label:
		sampler.SetInterpolator(sitk.sitkNearestNeighbor)
	else:
		sampler.SetInterpolator(sitk.sitkBSpline)
	return sampler.Execute(img)


def segment_list(lfiles):
	for i in lfiles:
		if os.path.isfile(cwd+gt_dir+i):
			print("\t{}".format(i))
			rawimg = sitk.ReadImage(cwd+gt_dir+i)
			if rawimg.GetDepth() < 128:
				sample = resample(rawimg, (512,512,128), False)
			else:
				sample = rawimg
			segment_cubed((sample,i))


ct_dir = "Data/rawdata/"	#Needs to be in current working directory
gt_dir = "Data/groundtruth/"
weight_dir = "model/weights2018-10-23-13/"
data_dir = "Dataset_1/"
input_shape = (128,128)
num_classes = 11
cwd = os.getcwd()+"/"

model = unet(input_shape + (1,), num_classes)
weights = os.listdir(cwd+weight_dir+data_dir)[-1]
print("Using model {} for segmentation".format(weights))
model.load_weights(cwd+weight_dir+data_dir+weights)
model.predict(np.zeros((1,128,128,1))) #warmup

pr_dir = cwd+weight_dir+"prediction1/"
if not os.path.isdir(pr_dir):
	os.mkdir(pr_dir)
print(pr_dir)
images = [i for i in sorted(os.listdir(cwd+ct_dir)) if i.endswith('.mhd') and not (i in os.listdir(pr_dir))]
print(images)
segment_list(images)


data_dir = "Dataset_2/"
model.load_weights(cwd+weight_dir+data_dir+weights)
model.predict(np.zeros((1,128,128,1))) #warmup

pr_dir = cwd+weight_dir+"prediction2/"
if not os.path.isdir(pr_dir):
	os.mkdir(pr_dir)
print(pr_dir)
images = [i for i in sorted(os.listdir(cwd+ct_dir)) if i.endswith('.mhd') and not (i in os.listdir(pr_dir))]

segment_list(images)

