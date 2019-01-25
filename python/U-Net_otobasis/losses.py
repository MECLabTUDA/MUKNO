from keras.losses import binary_crossentropy
import keras.backend as K
import tensorflow as tf

def dice_coeff(y_true, y_pred):
    smooth = 1.
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    score = (2. * intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)
    return score


def dice_loss(y_true, y_pred):
    loss = 1 - dice_coeff(y_true, y_pred)
    return loss


def bce_dice_loss(y_true, y_pred):
    loss = binary_crossentropy(y_true, y_pred) + dice_loss(y_true, y_pred)
    return loss


def weighted_dice_coeff(y_true, y_pred, weight):
    smooth = 1.
    w, m1, m2 = weight * weight, y_true, y_pred
    intersection = (m1 * m2)
    score = (2. * K.sum(w * intersection) + smooth) / (K.sum(w * m1) + K.sum(w * m2) + smooth)
    return score


def weighted_dice_loss(y_true, y_pred):
    y_true = K.cast(y_true, 'float32')
    y_pred = K.cast(y_pred, 'float32')
    # if we want to get same size of output, kernel size must be odd number
    if K.int_shape(y_pred)[1] == 128:
        kernel_size = 11
    elif K.int_shape(y_pred)[1] == 256:
        kernel_size = 21
    elif K.int_shape(y_pred)[1] == 512:
        kernel_size = 21
    elif K.int_shape(y_pred)[1] == 1024:
        kernel_size = 41
    else:
        raise ValueError('Unexpected image size')
    averaged_mask = K.pool2d(
        y_true, pool_size=(kernel_size, kernel_size), strides=(1, 1), padding='same', pool_mode='avg')
    border = K.cast(K.greater(averaged_mask, 0.005), 'float32') * K.cast(K.less(averaged_mask, 0.995), 'float32')
    weight = K.ones_like(averaged_mask)
    w0 = K.sum(weight)
    weight += border * 2
    w1 = K.sum(weight)
    weight *= (w0 / w1)
    loss = 1 - weighted_dice_coeff(y_true, y_pred, weight)
    return loss


def weighted_bce_loss(y_true, y_pred, weight):
    # avoiding overflow
    epsilon = 1e-7
    y_pred = K.clip(y_pred, epsilon, 1. - epsilon)
    logit_y_pred = K.log(y_pred / (1. - y_pred))

    # https://www.tensorflow.org/api_docs/python/tf/nn/weighted_cross_entropy_with_logits
    loss = (1. - y_true) * logit_y_pred + (1. + (weight - 1.) * y_true) * \
                                          (K.log(1. + K.exp(-K.abs(logit_y_pred))) + K.maximum(-logit_y_pred, 0.))
    return K.sum(loss) / K.sum(weight)


def weighted_bce_dice_loss(y_true, y_pred):
    y_true = K.cast(y_true, 'float32')
    y_pred = K.cast(y_pred, 'float32')
    # if we want to get same size of output, kernel size must be odd number
    if K.int_shape(y_pred)[1] == 128:
        kernel_size = 11
    elif K.int_shape(y_pred)[1] == 256:
        kernel_size = 21
    elif K.int_shape(y_pred)[1] == 512:
        kernel_size = 21
    elif K.int_shape(y_pred)[1] == 1024:
        kernel_size = 41
    else:
        raise ValueError('Unexpected image size')
    averaged_mask = K.pool2d(
        y_true, pool_size=(kernel_size, kernel_size), strides=(1, 1), padding='same', pool_mode='avg')
    border = K.cast(K.greater(averaged_mask, 0.005), 'float32') * K.cast(K.less(averaged_mask, 0.995), 'float32')
    weight = K.ones_like(averaged_mask)
    w0 = K.sum(weight)
    weight += border * 2
    w1 = K.sum(weight)
    weight *= (w0 / w1)
    loss = weighted_bce_loss(y_true, y_pred, weight) + (1 - weighted_dice_coeff(y_true, y_pred, weight))
    return loss

# ===========================================================================
# \brief weighted crossentropy loss for multi class segmentation
# 
# check  https://stackoverflow.com/questions/44560549/unbalanced-data-and-weighted-cross-entropy/44563055#44563055
def class_weighted_cross_entropy(y_true, y_pred):
	num_classes = 11
	w_background 	= 0.1 # background
	w_carotid 		= 0.1 # carotid artery
	w_jugular 		= 0.1 # jugular vein
	w_facialis 		= 0.1 # facial nerve
	w_cochlea 		= 0.1 # cochlea
	w_chorda		= 0.1 # chorda
	w_ossicles		= 0.1 # ossicles
	w_ssc			= 0.1 # semicircular canals
	w_iac			= 0.1 # internal auditory canal
	w_aqueduct		= 0.0 # vestibular aqueduct
	w_eac  			= 0.1 # external auditory canal
	class_weights = tf.constant([[w_background, w_carotid, w_jugular, w_facialis, w_cochlea, w_chorda, w_ossicles, w_ssc, w_iac, w_aqueduct, w_eac]])
	# this will give you one hot code for your prediction
	onehot_labels = tf.one_hot(tf.argmax(y_pred, axis=-1), depth=num_classes, axis=-1)
	# deduce weights for batch samples based on their true label
	weights = tf.reduce_sum(class_weights * onehot_labels, axis=1)
	print("weights 				{}".format(weights))
	# compute your (unweighted) softmax cross entropy loss
	unweighted_losses = tf.nn.softmax_cross_entropy_with_logits_v2(labels=onehot_labels, logits=y_true)
	print("unweighted_losses 	{}".format(unweighted_losses))
	# apply the weights, relying on broadcasting of the multiplication
	weighted_losses = unweighted_losses * weights
	print("weighted_losses 		{}".format(weighted_losses))
	# reduce the result to get your final loss
	loss = tf.reduce_mean(weighted_losses)
	print("loss  				{}".format(loss))
	return loss
	
# ===========================================================================
# \brief weighted crossentropy loss for multi class segmentation
# 
# check  https://stackoverflow.com/questions/51361402/weighted-categorical-cross-entropy-semantic-segmentation
def weighted_cross_entropy(y_true, y_pred):
    def weighted_categorical_cross_entropy(y_true, y_pred):
        w = tf.reduce_sum(y_true)/tf_cast(tf_size(y_true), tf_float32)
        loss = w * tf.nn.softmax_cross_entropy_with_logits_v2(labels=onehot_labels, logits=y_true)
        return loss
    return weighted_categorical_cross_entropy

# ===========================================================================
# \brief weighted crossentropy loss for multi class segmentation
# 
# this time according to https://stackoverflow.com/questions/44560549/unbalanced-data-and-weighted-cross-entropy
def class_weighted_cross_entropy_2(y_true, y_pred):
    s = tf.shape(y_true)

    # if number of output classes  is at last
    number_classses = s[-1]

    # this will give you one hot code for your prediction
    clf_pred = tf.one_hot(tf.argmax(y_pred, axis=-1), depth=number_classses, axis=-1)

    # extract the values of y_pred where y_pred is max among the classes
    prediction = tf.where(tf.equal(clf_pred, 1), y_pred, tf.zeros_like(y_pred))

    # if one hotcode == 1 then class1_prediction == y_pred  else class1_prediction ==0
    class1_prediction = prediction[:, :, :, 0:1]
    # you can compute your loss here on individual class and return the loss,
	# just for simplicity i am returning the class1_prediction
    return class1_prediction

# ===========================================================================
# \brief weighted crossentropy loss for multi class segmentation
# 
# this time according to https://gist.github.com/wassname/ce364fddfc8a025bfab4348cf5de852d
def class_weighted_cross_entropy_3(weights):
		weights = K.variable(weights)
		def loss(y_true, y_pred):
			# scale preds so class probs of each sample sum to 1
			y_pred /= K.sum(y_pred, axis=-1, keepdims=True)
			# clip to prevent nan and inf
			y_pred = K.clip(y_pred, K.epsilon(), 1 - K.epsilon())
			# compute loss
			loss = y_true * K.log(y_pred) * weights
			loss = -K.sum(loss, -1)
			return loss
		return loss