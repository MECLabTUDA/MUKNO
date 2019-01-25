import keras
import SimpleITK as sitk
import numpy as np
import os
import itertools
from u_net import get_unet_128 as unet




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
	
def segment_128cube(arr):
	ret = np.zeros((128,128,128,11))
	for slice in range(128):
		prediction = model.predict(arr[slice,:,:].reshape(1,128,128,1)) #look at x
		ret[slice,:,:] = np.maximum(prediction, ret[slice,:,:])
		
		prediction = model.predict(arr[:,slice,:].reshape(1,128,128,1)) #look at y
		ret[:,slice,:,:] = np.maximum(prediction, ret[:,slice,:,:])
		
		prediction = model.predict(arr[:,:,slice].reshape(1,128,128,1)) #Comment in for z direction
		ret[:,:,slice,:] = np.maximum(prediction, ret[:,:,slice,:])
	return ret

#TLDR gives us all indices without skipping the upmost edge and without stepping over
#Gives a list of indices that go from s to e in increments of ss with a minimum distance to e equal to i including the maximal value e-i
def incrange(s,e,i,ss):
	ret = s
	while ret+i < e:
		yield ret
		ret += ss
	yield e-i 

#Does Not give the bounding box atm
def bbox(img):
	img = img==5 #Boolean Matrix with true where the img was 5
	N = img.ndim
	out = []
	for ax in itertools.combinations(range(N), N - 1):
		nonzero = np.any(img, axis=ax)
		out.extend(np.where(nonzero)[0][[0, -1]])
	return tuple(out)

#Splits the image into cubes of equal size 128
def segment_cubed(sitkImg):
	(width, height, depth) = sitkImg.GetSize()
	imgarr = sitk.GetArrayFromImage(sitkImg)
	ret = np.zeros((depth,height,width,11))
	i = 0
	if depth >= 128:
		for x in incrange(0,width,128,64): 	#The last number dictates the stepsize. Change it for more sampling but WAY more time ##64 is enough with a well trained net
			for y in incrange(0,height,128,64):
				for z in incrange(0,depth,128,64):
					print(x,y,z)
					i += 1
					ret[z:z+128,y:y+128,x:x+128,:] = np.maximum(ret[z:z+128,y:y+128,x:x+128,:], segment_128cube(imgarr[z:z+128,y:y+128,x:x+128])) #Voting for overlap Currently highest wins it all
	print("{} cubes used".format(i))
	ret = np.argmax(ret,axis=3) #Gives us the index for the highest voted  which corresponds to the label
	
	#print(bbox(ret)) For the bounding box approach with specialized net. Not currently working
	ret = sitk.GetImageFromArray(ret.astype(np.uint16)) #converts to the needed format
	ret.SetOrigin(sitkImg[0].GetOrigin())
	ret.SetSpacing(sitkImg[0].GetSpacing())
	ret.SetDirection(sitkImg[0].GetDirection())
	#sitk.WriteImage(ret, pr_dir+sitkImg[1]) #Do not save here. We need to restore the size
	return ret


def segment_list(lfiles):
	for i in lfiles:
		print("\t{}".format(i))
		rawimg = sitk.ReadImage(cwd+ct_dir+i)
		if rawimg.GetDepth() < 128:
			sample = resample(rawimg, (512,512,128), False) #uniform minimum size
		else:
			sample = rawimg
		sample = segment_cubed(sample)
		sample = resample(sample, rawimg.GetSize(), True) #This is our labeled prediction
		sitk.WriteImage(sample, pr_dir+i)
		del rawimg
		del sample
	print("Segmented {} volumes".format(len(images)))


ct_dir = "Data/rawdata/"	#Needs to be in current working directory
gt_dir = "Data/groundtruth/"
data_dir = "Dataset_1/"
input_shape = (128,128)
num_classes = 11
cwd = os.getcwd()+"/" #Gives the folder the script was started from

weight_dir = "model/"+sorted(os.listdir(cwd+"model"))[-1]+"/" #Should choose the newest net
print(weight_dir)

model = unet(input_shape + (1,), num_classes)
weights = os.listdir(cwd+weight_dir+data_dir)[-1] #give the last element in case of multiples
print("Using model {} for segmentation".format(weights))
model.load_weights(cwd+weight_dir+data_dir+weights)
model.predict(np.zeros((1,128,128,1))) #warmup for possible multithreading

pr_dir = weight_dir+"prediction1/"#cwd+weight_dir+"prediction1/"
if not os.path.isdir(pr_dir):
	os.mkdir(pr_dir)
print(pr_dir)
images = [i for i in sorted(os.listdir(cwd+ct_dir)) if i.endswith('.mhd') and not (i in os.listdir(pr_dir))] #Get the name of all files ending with ".mhd" in a list 

segment_list(images) #Segment all in this list

#Do it again for the other data set
data_dir = "Dataset_2/"
model.load_weights(cwd+weight_dir+data_dir+weights)
model.predict(np.zeros((1,128,128,1))) #warmup

pr_dir = cwd+weight_dir+"prediction2/"
if not os.path.isdir(pr_dir):
	os.mkdir(pr_dir)
print(pr_dir)
images = [i for i in sorted(os.listdir(cwd+ct_dir)) if i.endswith('.mhd') and not (i in os.listdir(pr_dir))]

segment_list(images)

