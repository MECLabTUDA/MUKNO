import SimpleITK as sitk
import threading
import os
import h5py

# ===========================================================================
# \brief convenience function to serializes img_infos 
def serialize_img_infos(img_infos):
	sizes      = []
	spacings   = []
	origins    = []
	directions = []
	for info in img_infos:
		sizes.append(info[0])
		spacings.append(info[1])
		origins.append(info[2])
		directions.append(info[3])
	return (sizes, spacings, origins, directions)
	
# ===========================================================================
def deserialize_img_infos(sizes, spacings, origins, directions):
	img_infos = []
	for a,b,c,d in zip(sizes, spacings, origins, directions):
		info = []
		info.append(a)
		info.append(b)
		info.append(c)
		info.append(d)
		img_infos.append(info)
	return img_infos

# ===========================================================================
# \brief
def get_filenames(demandFirstDataSet, directory) :
	files = [f for f in os.listdir(directory) if f.endswith('.mhd')]
	files = sorted(files)
	if demandFirstDataSet:
		files = files[1::2] # start at one skip every other one
	else:
		files = files[0::2] # start at 0 skip every other one
	return files
	
# ===========================================================================
# \brief just load in the data
def get_data_set(filename):
	file = h5py.File(filename.strip(), "r")
	images = file["images"][()]
	labels = file["labels"][()]
	return images, labels
	
# ===========================================================================
# \brief just load in the data
def get_data_set_for_chorda(filename):
	file = h5py.File(filename.strip(), "r")
	images = file["images"][()]
	all_labels = file["labels"][()]
	slices = []
	for set in all_labels:
		slices.append(set[:,:,5]) # chorda label
	slices   = np.stack(slices)
	labels = utils.to_categorical(slices, 2)
	return images, labels

# ===========================================================================
# \brief
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


class threadsafe_iter:
    """Takes an iterator/generator and makes it thread-safe by
    serializing call to the `next` method of given iterator/generator.
    """
    def __init__(self, it):
        self.it = it
        self.lock = threading.Lock()

    def __iter__(self):
        return self

    def next(self):
        with self.lock:
            return next(self.it)


def threadsafe_generator(f):
    """A decorator that takes a generator function and makes it thread-safe.
    """
    def g(*a, **kw):
        return threadsafe_iter(f(*a, **kw))
    return g