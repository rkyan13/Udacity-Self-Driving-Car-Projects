"""
Reference :
Udacity Term3 : Project 2 : Semantic Segmentation: Lessons
1) Lesson  9: Fully Convolutional Networks
2) Lesson 10: Scene Understanding
   especially/specifically
   Concept 09: FCN-8 - Encoder
   Concept 10: FCN-8 - Decoder
   Concept 11: FCN-8 - Classification & Loss
3) Lesson 11: Inference Performance
4) Lesson 12: Elective Project :Semantic Segmenation
   Concept-2-Project Q&A:  Aaron Brown's instruction video
   Video @ https://www.youtube.com/watch?time_continue=576&v=5g9sZIwGubk

 Note: kernel_constraint & bias_constraint seem to be valid arguments on the tensorflow on my laptop, but for some reason the tensorflow in the workspace
 doesn't seem to like them.Hence I've commented them out

"""


#!/usr/bin/env python3
import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests
import time


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """

    vgg_tag                    = 'vgg16'
    vgg_input_tensor_name      = 'image_input:0'
    vgg_keep_prob_tensor_name  = 'keep_prob:0' #dropout keep probability
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    # TODO: Implement function

    #----------------------------------------------------------------------------------------------------------------------------------------------------
    # KSW comments: We are actually extracting appropriate layers of the Encoder here
    #----------------------------------------------------------------------------------------------------------------------------------------------------
    # --> based on Lesson 12: Elective Project :Semantic Segmentation/Concept-2-Project Q&A:
    #     Aaron Brown's instructions approx ~2 minutes, ~4 minutes into the video)
    # ---> For more information on Encoder see: Lesson 10:Scene-Understanding/Concept9-FCN-8-Encoder
    #      also see the FCN-8 paper : https://people.eecs.berkeley.edu/~jonlong/long_shelhamer_fcn.pdf
    #----------------------------------------------------------------------------------------------------------------------------------------------------

    # i) Use tf.saved_model.loader.load to load the model and weights
    # sess     : The TensorFlow session to restore the graph variables into (i.e the session you are running)
    # vgg_tag  : Set of string tags to identify the required MetaGraphDef.
    #          "vgg_tag" : should have been the tag used when saving the variables using the SavedModel save() API.
    # vgg_path : The path+folder in which the SavedModel protocol buffer and variables to be loaded are located
    # output   : The MetaGraphDef protocol buffer loaded in the provided session ('sess').
    #          : Ksw thinks that the metaGraph is loaded as the default_graph and hence it requires no output variable to store the graph
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    # ii) The tf.saved_model.loader.load above has loaded the appropirate graph in default_graph
    vgg_graph       = tf.get_default_graph()

    # iii) Load all the appropriate tensor-layers from the loaded graph
    vgg_input_layer = vgg_graph.get_tensor_by_name(vgg_input_tensor_name)
    vgg_keep_prob   = vgg_graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    vgg_layer3      = vgg_graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    vgg_layer4      = vgg_graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    vgg_layer7      = vgg_graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return vgg_input_layer, vgg_keep_prob, vgg_layer3, vgg_layer4, vgg_layer7

tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
    #----------------------------------------------------------------------------------------------------------------------------------------------------
    #KSW: We are actually coding up part of the Encoder and all of the Decoder here
    #----------------------------------------------------------------------------------------------------------------------------------------------------
    #i) some hyper parameters
    param_stddev         = 0.01
    param_l2_regularizer = 1e-3

    #----------------------------------------------------------------------------------------------------------------------------------------------------
    # ii) Part of Encoder: Create 1X1 convolutions from vgg_layer3_out, vgg_layer4_out, vgg_layer7_out
    #     The majority of the encoder is in the pretrained vgg16 model from above load_vgg function
    # ---> Replace the fully connected layers of VGG16 by 1X1convolutions: These are the vgg_layer3_1x1, vgg_layer4_1x1, & vgg_layer7_1x1
    # ---> See  Lesson 10:Scene-Understanding/Concept9-FCN-8-Encoder for syntax of 1X1 convolution
    # ---> Lesson 12: Elective Project :Semantic Segmentation/Concept-2-Project Q&A:  Aaron Brown's instructions approx ~8.30 minutes into the video)
    # ---> setup kernel_regularizer based on Aaron Brown's instruction
    #----------------------------------------------------------------------------------------------------------------------------------------------------

    # Create 1X1 convolutions from vgg_layer3_out,
    vgg_layer3_1x1 = tf.layers.conv2d(inputs      = vgg_layer3_out,
                                      filters     = num_classes,
                                      kernel_size = 1,
                                      strides     = (1, 1),  #default
                                      padding     = 'same',
                                      kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                      kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                      #kernel_constraint    = None,                   #default
                                      use_bias             = True,                   #default
                                      bias_initializer     = tf.zeros_initializer(), #default
                                      bias_regularizer     = None,                   #default
                                      #bias_constraint      = None,                   #default
                                      activity_regularizer = None,                   #default
                                      name                 = 'vgg_layer3_1x1')

    # Create 1X1 convolutions from vgg_layer4_out
    vgg_layer4_1x1 = tf.layers.conv2d(inputs      = vgg_layer4_out,
                                      filters     = num_classes,
                                      kernel_size = 1,
                                      strides     = (1, 1),  #default
                                      padding     = 'same',
                                      kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                      kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                      #kernel_constraint    = None,                   #default
                                      use_bias             = True,                   #default
                                      bias_initializer     = tf.zeros_initializer(), #default
                                      bias_regularizer     = None,                   #default
                                      #bias_constraint      = None,                   #default
                                      activity_regularizer = None,                   #default
                                      name                 = 'vgg_layer4_1x1')

    # Create 1X1 convolutions from vgg_layer7_out
    vgg_layer7_1x1 = tf.layers.conv2d(inputs      = vgg_layer7_out,
                                      filters     = num_classes,
                                      kernel_size = 1,
                                      strides     = (1, 1),  #default
                                      padding     = 'same',
                                      kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                      kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                      #kernel_constraint    = None,                   #default
                                      use_bias             = True,                   #default
                                      bias_initializer     = tf.zeros_initializer(), #default
                                      bias_regularizer     = None,                   #default
                                      #bias_constraint      = None,                   #default
                                      activity_regularizer = None,                   #default
                                      name                 = 'vgg_layer7_1x1')




    #----------------------------------------------------------------------------------------------------------------------------------------------------
    # iii) Decoder: Upsample by creating Deconvolution/Transpose Convolution Layers'.
    #              Also create skip connections
    # ---> The first 2 Deconvolutions doubles the previous layer's size
    # ---> The third Deconvolutions also the final layer) increases the previous layer's size by 8 times
    # ---> See  Lesson 10:Scene-Understanding/FCN-8-Decoder to figure out the kernel_size, stride ,
    #      number_filters to use (num_filters = num_classes) and also to see which layer to connect to what in skip connections
    # ---> Lesson 12: Elective Project :Semantic Segmentation/Concept-2-Project Q&A:  Aaron Brown's instructions
    # ---> setup kernel_regularizer based on Aaron Brown's instruction
    #----------------------------------------------------------------------------------------------------------------------------------------------------

    # Deconv1(doubles previous layers's size) & Skip1
    deconv1_upX2 = tf.layers.conv2d_transpose(inputs        = vgg_layer7_1x1,
                                              filters     = num_classes,
                                              kernel_size = 4,
                                              strides     = (2, 2), #no longer default
                                              padding     = 'same',
                                              activation           = None,
                                              kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                              kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                              #kernel_constraint    = None,                   #default
                                              use_bias             = True,                   #default
                                              bias_initializer     = tf.zeros_initializer(), #default
                                              bias_regularizer     = None,                   #default
                                              #bias_constraint      = None,                   #default
                                              activity_regularizer = None,                   #default
                                              name                 = 'deconv1_upX2')

    skip1 = tf.add(deconv1_upX2, vgg_layer4_1x1, name='skip1')

    # Deconv2(doubles previous layers's size) & Skip2
    deconv2_upX2 = tf.layers.conv2d_transpose(inputs      = skip1,
                                              filters     = num_classes,
                                              kernel_size = 4,
                                              strides     = (2, 2), #no longer default
                                              padding     = 'same',
                                              activation           = None,
                                              kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                              kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                              #kernel_constraint    = None,                   #default
                                              use_bias             = True,                   #default
                                              bias_initializer     = tf.zeros_initializer(), #default
                                              bias_regularizer     = None,                   #default
                                              #bias_constraint      = None,                   #default
                                              activity_regularizer = None,                   #default
                                              name                 = 'deconv2_upX2')

    skip2 = tf.add(deconv2_upX2, vgg_layer3_1x1, name='skip2')

    # Deconv3(The final layer): This makes the output of the deconvolution 8 times as big as the previous layer
    deconv3_upX8 = tf.layers.conv2d_transpose(inputs       = skip2,
                                              filters     = num_classes,
                                              kernel_size = 16,
                                              strides     = (8, 8), #no longer default
                                              padding     = 'same',
                                              activation           = None,
                                              kernel_initializer   = tf.random_normal_initializer(stddev= param_stddev),
                                              kernel_regularizer   = tf.contrib.layers.l2_regularizer(param_l2_regularizer),
                                              #kernel_constraint    = None,                   #default
                                              use_bias             = True,                   #default
                                              bias_initializer     = tf.zeros_initializer(), #default
                                              bias_regularizer     = None,                   #default
                                              #bias_constraint      = None,                   #default
                                              activity_regularizer = None,                   #default
                                              name                 = 'deconv3_upX8')

    #Note deconv3_upX8 = nn_last_layer
    return deconv3_upX8

tests.test_layers(layers)


def optimize(nn_last_layer, correct_labels_ph, learning_rate_ph, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function

    # i) Logits: Output tensor is 4D nn_last_layer so we have to reshape it to 2D:
    # logits are perfect for softmax
    # based on Lesson 10:Scene-Understanding/Concept11-FCN-8-Classification & Loss
    logits                 = tf.reshape(nn_last_layer, (-1, num_classes))

    # ii) The labels are also 4D and need to be reshaped to 2D
    # based on Lesson 12: Elective Project :Semantic Segmentation/Concept-2-Project Q&A:  Aaron Brown's instructions approx ~17 minutes into the video)
    shape_corrected_labels = tf.reshape(correct_labels_ph, (-1, num_classes))

    # iii) Define the loss-function= cross_entropy_loss
    # based on Lesson 10:Scene-Understanding/Concept11-FCN-8-Classification & Loss
    cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits= logits, labels= shape_corrected_labels)
    #You wrap this function with reduce_mean(), which computes the mean of elements across dimensions of a tensor.(lol so many operations)
    cross_entropy_loss     = tf.reduce_mean(cross_entropy)

    # iv) Use Adam Optimizer
    # based on Lesson 12: Elective Project :Semantic Segmentation/Concept-2-Project Q&A:  Aaron Brown's instructions approx ~17 minutes into the video)
    #Some of the most popular optimization algorithms used are the Stochastic Gradient Descent (SGD), ADAM and RMSprop.
    #Depending on whichever algorithm you choose, you’ll need to tune certain parameters, such as learning rate or momentum.
    #In this case, you pick the ADAM optimizer, for which you need to define the learning rate
    adam_optimizer = tf.train.AdamOptimizer(learning_rate= learning_rate_ph)

    # v) The actual training operation : Use the adam_optimizer to minimize the cross_entropy loss
    train_op  = adam_optimizer.minimize(cross_entropy_loss)

    # vi) Returning these in the order provided by Udacity above in comments section
    return logits, train_op, cross_entropy_loss

tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_images_ph,
             correct_labels_ph, keep_prob_ph, learning_rate_ph):
    """
    Train neural network and print out the loss during training.
    :param sess               : TF Session
    :param epochs             : Number of epochs
    :param batch_size         : Batch size
    :param get_batches_fn     : Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op           : TF Operation to train the neural network
    :param cross_entropy_loss : TF Tensor for the amount of loss
    :param input_images_ph    : TF Placeholder for input images
    :param correct_labels_ph  : TF Placeholder for label images
    :param keep_prob_ph       : TF Placeholder for dropout keep probability
    :param learning_rate_ph   : TF Placeholder for learning rate
    """
    # TODO: Implement function
    #  See Lesson 12: Elective Project :Semantic Segmenation /Concept-2-Project Q&A:  Aaron Brown's instruction video ~19.30minutes
    sess.run(tf.global_variables_initializer())


    print("Beginning Training...... Total #Epochs = ", epochs, "batch_size =", batch_size)
    start_time = time.time()
    #Begin for loop1: iterate through Epochs-------------------------------------------------------------------------------
    for epoch_num in range(epochs):
        print("\n----------------------- EPOCH # :  ",epoch_num+1,"----------------------------------")
        loss_cumulative = []
        loss_avg        = 0
        batch_num    = 0
        # Begin for loop2: iterate through Batches--------------------------------------------------------------------------
        # get_batches_fn, returns: Batches of training data : np.array(images), np.array(gt_images)
        for images_array, labels_array in get_batches_fn(batch_size):
            batch_num  += 1
            _, loss = sess.run([train_op, cross_entropy_loss],
                               feed_dict={input_images_ph   : images_array,
                                          correct_labels_ph : labels_array,
                                          keep_prob_ph      : 0.5,
                                          learning_rate_ph  : 0.00001})
            loss_cumulative.append(loss)
            loss_avg = loss_avg + loss
            print("Batch-Num # = ", batch_num , " : Loss =", loss)
        #EOF for loop2: iterate through Batches: for image, label in get_batches_fn(batch_size):----------------------------

        loss_avg = loss_avg/float(batch_num)
        print("\n Epoch # = ", epoch_num+1, "Average-Loss-Across-Batches(in this epoch) =", loss_avg)
        print("\n Loss-cumulative =", loss_cumulative)
        elapsed_time = time.time() - start_time
        print("\n Time Elapsed(HH:MM:SS) ", time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))
    #End for loop1: iterate through Epochs : for epoch_num in range(epochs): -------------------------------------------------
    print("\n Training Complete !!!! :D :D :D")

    return

tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)  # KITTI dataset uses 160x576 images
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # i) TODO: Build NN using load_vgg, layers, and optimize function
        vgg_input_layer, vgg_keep_prob, vgg_layer3, vgg_layer4, vgg_layer7 \
                                             = load_vgg(sess, vgg_path)
        nn_last_layer                        = layers(vgg_layer3, vgg_layer4, vgg_layer7, num_classes)

        correct_labels_ph                    = tf.placeholder(tf.int32, [None, None, None, num_classes], name='correct_label')
        learning_rate_ph                     = tf.placeholder(tf.float32, name='learning_rate')
        logits, train_op, cross_entropy_loss = optimize(nn_last_layer, correct_labels_ph, learning_rate_ph, num_classes)



        # ii) TODO: Train NN using the train_nn function
        #ksw-comments-Note: the train_nn takes 4 placeholders as inputs
        # a&b)  correct_labels_ph, learning_rate_ph : For this I have created placeholder with the necessary datatype, shape etc .
        #       correct_labels_ph(4Dtensor)---gets_passed_to----->train_nn ---gets_passed_to----->optimize(where it is reshaped to 2D tensor),
        # Hence correct_labels_ph shape is  [None, None, None, num_classes]
        # c&d)  input_images_ph,keep_prob_ph : I should be creating placeholder for these too, but for some reason these are loaded
        #       from pretrained VGG16: using load_vgg above. I thought only tensors/layers which have weights are loaded from pretrained models ??
        #       my reasoning is input_images_ph,keep_prob_ph even when loaded from load_vgg only have the placeholder information (similar to tf.placeholder ....)
        #       the tf.placeholder(type, name, shape, etal. ) instead of me having to create it , comes directly from saved vgg model
        NUM_EPOCHS = 50
        BATCH_SIZE = 5
        train_nn(sess               = sess,
                 epochs             = NUM_EPOCHS,
                 batch_size         = BATCH_SIZE,
                 get_batches_fn     = get_batches_fn,
                 train_op           = train_op,
                 cross_entropy_loss = cross_entropy_loss,
                 input_images_ph    = vgg_input_layer,
                 correct_labels_ph  = correct_labels_ph,
                 keep_prob_ph       = vgg_keep_prob,
                 learning_rate_ph   = learning_rate_ph)



        # iii) TODO: Save inference data using helper.save_inference_samples
        #  helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)
        # SESSION FOR TRAINING
        #---> the train_nn above launches a session for training within a framework of epochs and batches.
        #---> it evaluates train_op and cross_entropy_loss
        #---> the feed_dict is input_images, labels, dropu-keep-probability and learning rate
        #---> _, loss = sess.run([train_op, cross_entropy_loss], feed_dict={input_images_ph: images_array, correct_labels_ph : labels_array, keep_prob_ph : 0.5, learning_rate_ph:0.0001})

        # SESSION FOR TESTING
        #---> the helper.save_inference_samples launches a session for testing (so  framework of epochs and batches not necessary).
        #---> it evaluates softmax on logits (instead of evaluating the trainin-operation and loss function)
        #---> the feed_dict is onnly input_images & drop-out-keep-probability: (labels and learning rate not required)
        #---> im_softmax = sess.run([tf.nn.softmax(logits)],{keep_prob: 1.0, image_pl: [image]})
        helper.save_inference_samples(runs_dir    = runs_dir,
                                      data_dir    = data_dir,
                                      sess        = sess,
                                      image_shape = image_shape,
                                      logits      = logits,
                                      keep_prob   = vgg_keep_prob,
                                      input_image = vgg_input_layer)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
