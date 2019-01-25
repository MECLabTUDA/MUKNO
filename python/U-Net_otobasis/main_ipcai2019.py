from create_xy_samples import create_xy_samples
from create_xz_samples import create_xz_samples
from create_yz_samples import create_yz_samples

from train_xy_init_model import train_xy_init_model as train_xy
from train_xz_init_model import train_xz_init_model as train_xz
from train_yz_init_model import train_yz_init_model as train_yz

from test_xy_init_model import test_xy_init_model as test_xy
from test_xz_init_model import test_xz_init_model as test_xz
from test_yz_init_model import test_yz_init_model as test_yz
from test_combined_models import test_combined_models as test_combined
from test_combined_models import test_combined_models_plus_chorda as test_combined_plus_chorda

# ===========================================================================
# \brief Performs the complete deep learning part
#
# - Creates two data sets (.h5-files) from the CT and label images 
#	- data set 1 -> every second file from the start
#	- data set 2 -> every second file from the start + 1 
#   - each consists of long list of slices for fast and easy access
#
# - trains 12 networks 
#	- for axial, coronal and sagittal direction
#	- for generals nets (num_classes = 11) and chorda nets (num_classes = 2)
#	- for dataset 1 and 2
# - predicts with these nets on the respective data set
# - performs majority voting in the end
#
# - manual settings:
# 	- flags (do_xxx) can be set to indicate what should be used when calling this script.
# 	- some files define local string constants for input and output folders 
#		- search for hard coded path names containing "IPCAI2019"
#		- there should be only two types of hard coded folders (root_dir and raw_data_dir, always the same)
if __name__ == "__main__":
	do_create_xy = False
	do_create_xz = False
	do_create_yz = False
	if do_create_xy:
		create_xy_samples()
	if do_create_xz:
		create_xz_samples()
	if do_create_yz:
		create_yz_samples()
		
	do_train_xy = False
	do_train_xz = False
	do_train_yz = False
	num_classes = 11
	epochs_xy 	= 50
	epochs_z  	= 30
	if do_train_xy:
		model_date_xy = train_xy(num_classes, epochs_xy)
	else:
		model_date_xy = "2018-11-13_17-13-06_newMulti" #"2018-11-05_13-07-23_pc2646"
	if do_train_xz:
		model_date_xz = train_xz(num_classes, epochs_z)
	else:
		model_date_xz = "2018-11-14_09-44-12_newMulti" #"2018-11-05_16-07-37_pc2646"
	if do_train_yz:
		model_date_yz = train_yz(num_classes, epochs_z)
	else:
		model_date_yz = "2018-11-14_15-20-34_newMulti" #"2018-11-05_23-27-23_pc2646"
		
	do_test_xy = False
	do_test_xz = False
	do_test_yz = False
	if do_test_xy:
		test_xy(num_classes, model_date_xy)
	if do_test_xz:
		test_xz(num_classes, model_date_xz)
	if do_test_yz:
		test_yz(num_classes, model_date_yz)
		
	do_test_combined = False
	if do_test_combined:
		test_combined(model_date_xy, model_date_xz,model_date_yz)
		
	do_train_xy_chorda = False
	do_train_xz_chorda = False
	do_train_yz_chorda = False
	num_classes = 2
	epochs_z  	= 30
	# xy
	if do_train_xy_chorda:
		model_date_xy_chorda = train_xy(num_classes, epochs_xy)
	else:
		model_date_xy_chorda = "2018-11-15_09-39-34_newChorda" # "2018-11-02_11-10-26_chorda"
	# xz
	if do_train_xz_chorda:
		model_date_xz_chorda = train_xz(num_classes, epochs_z)
	else:
		model_date_xz_chorda = "2018-11-15_11-48-22_newChorda" #"2018-11-02_13-20-43_c_e30"
	# yz
	if do_train_yz_chorda:
		model_date_yz_chorda = train_yz(num_classes, epochs_z)
	else:
		model_date_yz_chorda = "2018-11-15_16-57-01_newChorda" #"2018-11-05_21-07-32_binary_crossentropy_e30"
		
	do_test_xy_chorda = True
	do_test_xz_chorda = False
	do_test_yz_chorda = False
	if do_test_xy_chorda:
		test_xy(num_classes, model_date_xy_chorda)
	if do_test_xz_chorda:
		test_xz(num_classes, model_date_xz_chorda)
	if do_test_yz_chorda:
		test_yz(num_classes, model_date_yz_chorda)
	
	do_test_combined_chorda = False
	if do_test_combined_chorda:
		test_combined_plus_chorda("2018-11-06_last_result_multi", model_date_xy_chorda, model_date_xz_chorda, model_date_yz_chorda)