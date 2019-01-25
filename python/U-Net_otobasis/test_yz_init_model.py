import os
import predict_xz_yz as predict
	
# ===========================================================================
# \brief 
#
def test_yz_init_model(num_classes, model_date) :
	print("=========== Predicting YZ ===============")
	root_dir   		= "C:/users/jfauser/IPCAI2019/ModelData/"
	model_name 		= "YZ-Unet-Init"
	model_base_dir 	= root_dir + model_name + "/"
	model_dir  		= model_base_dir + model_date + "/"
	raw_data_dir   	= "D:/IPCAI2019_Otobasis/Data/groundtruth/"
	out_dir    		= model_dir + "predictions/"
	if not os.path.isdir(out_dir):
		os.mkdir(out_dir)
	# ======================= Model 1, Predict Data Set 2 =======================
	use_model_one = True
	if use_model_one:
		predict_first_dataset = False;
		predict.predict_on_dataset_yz(model_base_dir, model_dir, raw_data_dir, out_dir, predict_first_dataset, num_classes)
	
	# ======================= Model 2, Predict Data Set 1 =======================
	use_model_two = True
	if use_model_two:
		predict_first_dataset = True;
		predict.predict_on_dataset_yz(model_base_dir, model_dir, raw_data_dir, out_dir, predict_first_dataset, num_classes)