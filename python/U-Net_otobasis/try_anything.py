import numpy as np

if __name__ == "__main__":
	slices     = np.zeros((512,128,512))
	print ("size:       {}".format(slices.shape ))
	img = []
	for slice in slices:
		img.append(slice)
	img = np.stack(img, axis=1)
	img.reshape(512, 512, 128)
	print ("size:       {}".format(img.shape ))