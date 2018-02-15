# Deploy a Caffe model in YARP

Contents:

* [Get ready for this tutorial](#get-ready-for-this-tutorial)
* [Complete the tutorial](#complete-the-tutorial)
* [Run the module on the robot and with different models](#run-the-module-on-the-robot-and-with-different-models)

## Get ready for this tutorial

In this tutorial we will go through an example of a simple YARP module that calls Caffe APIs in order to use a trained CNN model (we will use the one you trained in the assignment) and test it on a stream of images coming from a camera.

We suppose that you already followed instructions provided for the [tutorial](https://github.com/vvv-school/tutorial_dl-tuning#get-ready-for-the-afternoon), and are all set up with the VM and the `dl-lab` directory where to clone this repository (for this tutorial the iCW dataset is not needed).

We also suppose that you already completed the [assignment](https://github.com/vvv-school/assignment_dl-tuning) and have a trained network model.

#### Get the code

Clone this repository:

```sh
$ cd $ROBOT_CODE/dl-lab
$ git clone https://www.github.com/vvv-school/tutorial_dl-deployment.git
```

#### Compile the code

Compile the module that is provided with the repository:

```sh
$ cd tutorial_dl-deployment
$ mkdir build
$ cd build
$ ccmake ../
$ make
$ make install
```

When compiling, check that things are set correctly, in particular:

- check that `Caffe_DIR` points to Caffe installation (in the VM this is already set through the corresponding env variable in `~/.bashrc-dev`)
- check that all other variables are set correctly (`OpenCV_DIR` and `YARP_DIR`, `ICUBcontrib_DIR`)
- check that the the output of `make install` is like the following:

```sh
-- Install configuration: "Release"
-- Installing: /home/icub/robot-install/bin/objectRecognizer
-- Set runtime path of "/home/icub/robot-install/bin/objectRecognizer" to "/home/icub/robot-install/lib:/opt/ros/kinetic/lib:/home/icub/robot-code/caffe/build/install/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib"
-- Installing: /home/icub/robot-install/share/ICUBcontrib/contexts/objectRecognizer/objectRecognizer.ini.template
-- Installing: /home/icub/robot-install/share/ICUBcontrib/contexts/objectRecognizer/deploy_imDataLayer_caffenet.prototxt.template
-- Installing: /home/icub/robot-install/share/ICUBcontrib/applications/objectRecognizer.xml
```

This means that the `objectRecognizer` executable and the three configuration files have been correctly copied in the installation directories of iCub Contrib's modules.

## Complete the tutorial

Since the code is already there and installed, we only need to set some configuration files that are used by the `objectRecognizer` YARP module to locate the trained model and use it correctly.

To do this, we start by importing the context of the module:

~~~
$ yarp-config context --import objectRecognizer
~~~

Follow step-by-step instructions below to configure each file.

#### Configure the `.prototxt` file

We first need to configure a `.prototxt` file which defines the architecture of the network model that we are going to use. This file will be associated to the model weights, that we obtained with fine-tuning (the `final.caffemodel` file, do you remember?) when passed to Caffe APIs by the YARP module.

To this end, we could have used the `deploy.prototxt`. However, using this file implies that all pre-processing operations that must be done on the image before feeding it to the network (like subtracting the mean image of the training set, extracting a crop, converting it to a `caffe::Blob`, etc.) are implemented by the user. You can see an example of this in the [classify_image_list_vvv.cpp](https://github.com/vvv-school/tutorial_dl-tuning/blob/master/scripts/src/classify_image_list_vvv.cpp) script that we used in the tutorial and assignment on fine-tuning.
In our case, we should have implemented these operations in the YARP module, after reading the image from a port and before passing the data to Caffe APIs.

However, this is prone to errors and a small error in the preprocessing can have a great impact on performance. To this end, since in our case we can use `OpenCV` and deal with `cv::Mat` images, we opted instead to use the `MemoryData` layer ([link](http://caffe.berkeleyvision.org/tutorial/layers/memorydata.html)) provided by Caffe. This layer offers an interface that takes directly a `cv::Mat` as input, and converts it to a `caffe::Blob` by applying internally the preprocessing operations that we mentioned.

To this end, in this repository we provided a template `.prototxt` file that uses a `MemoryData` layer. We now only have to check and adapt it to have identical structure to the model that we trained.

To do this, you can make a copy of the provided template, and work on the copy:

~~~
$ cd ~/.local/share/yarp/contexts/objectRecognizer
$ cp deploy_MemoryData_caffenet.prototxt.template deploy_MemoryData_caffenet.prototxt
$ gedit deploy_MemoryData_caffenet.prototxt
~~~

1. At line 10, replace the following:

    ~~~
    mean_file: "/path/to/train_mean.binaryproto"
    ~~~

    with the correct absolute path to this file on your system (without using env variables). This is the binary file containing the mean image of the training set that we created and used during the training in the assignment. If you use the VM and followed instructions, it should be:

    ~~~
    mean_file: "/home/icub/robot-code/dl-lab/assignment_dl-tuning/cat_4categories/mean.binaryproto"
    ~~~

2. At line 216, set the `num_output` parameter of the `fc8N` layer to the correct number of classes that you model discriminates (4 in the task of the assignment).

#### Configure the `.ini` file

We now have to tell the YARP module `objectRecognizer` where to load the weights of the network and the definition file that we just configured.

To do this we modify the following file:

~~~
$ cd ~/.local/share/yarp/contexts/objectRecognizer
$ cp objectRecognizer.ini.template objectRecognizer.ini
$ gedit objectRecognizer.ini
~~~

And set:

~~~
prototxt_file /path/to/deploy_MemoryData_caffenet.prototxt
~~~

To the following value:

~~~
prototxt_file /home/icub/.local/share/yarp/contexts/objectRecognizer/deploy_MemoryData_caffenet.prototxt
~~~

And finally:

~~~
label_file /path/to/labels.txt
caffemodel_file /path/to/final.caffemodel
~~~

To the following values:
~~~
label_file /home/icub/robot-code/dl-lab/assignment_dl-tuning/cat_4categories/images_lists/labels.txt
caffemodel_file /home/icub/robot-code/dl-lab/assignment_dl-tuning/cat_4categories/all-3/final.caffemodel
~~~

Where we recall that you should have created the two files above while completing the assignment.

#### Run the module!

We are now ready to run the YARP module:

1. Ensure that the camera of you laptop is accessible from the VM. This can be done when the VM is running. In the following we try to provide some instructions depending on the software that you are using:

    - For `Parallels Desktop`, under the `Devices` tab, select `USB & Bluetooth`, `Configure`, and then check the box `Share Mac camera with Linux`.

    - For `Virtual Box`, under the `Devices` tab, select `USB` and then `USB Settings...`. In the opened panel, check that `USB 2.0` is selected and that the entry related to your camera is enabled.

    - For `VMWare Client`, under the `Virtual Machine` tab, select `Removable Devices`, then select the entry of the laptop camera and check `Connect`.

2. Lunch the YARP Server:
    ~~~
    $ yarp server
    ~~~

3. Lunch that YARP Manager:
    ~~~
    $ yarpmanager
    ~~~

4. Lunch a YARP Run called `/lh`:
    ~~~
    $ yarprun --server /lh
    ~~~

5. In the YARP Manager, open the `Object_Recognizer` application that you should see listed in the tab to the left, run and connect all modules.

At this point you should see two YARP Views: one is displaying the image streamed by the camera, with a green bounding box and the predicted label written over it, and the other one is displaying the scores produced by the network (the `prob` blob that we extract, which is also outputted by the module at each frame, you can see this if you launch the `objectRecognizer` from the terminal).

The network is classifying the image area inside the green bounding box. Restricting the region of interest to a smaller region, instead of classifying the full image, is very useful since the presence of noise in the background is reduced. You can play with the side of the region and see if enlarging it (eventually also placing two different objects inside the region!) deteriorates predictions. To do this, send commands to an RPC port in the following way:

~~~
$ yarp rpc /objectRecognizer/human:io
>>help
Responses:

  set crop_mode fixed      : sets the square crop at the center
  set crop_mode centroid   : sets the square crop on the object
  set crop_mode roi        : sets the rectangular crop around the object

  set radius <value>       [ int>0  ]: sets the square radius if 'centroid' or 'fixed' mode is on

  get radius               : provides the radius of the square ROI, if 'radius' mode is on
  get crop_mode            : tells if 'fixed', 'centroid', 'roi' mode is on
>>get radius
Response: 256
>>set radius 80
Response: [ack]
>>get radius
Response: 80
~~~

## Run the module on the robot and with different models

In this part we provide some indications to run the module on the robot and to use different network models.

#### Run on the robot

In this simple application we did not consider more elaborated approaches to localize the region of interest, and fixed it at the image center. However, this `objectRecognizer` module can also take inputs from other modules, for instance the [dispBlobber](https://github.com/robotology/segmentation/tree/master/dispBlobber), which segments the closest blob in the disparity map and provides its coordinates as output. This allows to classify a varying area on the image, which can be easily controlled by moving the object of interest in front of the camera. Clearly, this implies that an RGB-D sensor, or a stereo camera like the one on the iCub, provides the disparity map :-).

If you run your `objectRecognizer` on the robot, together with the `dispBlobber`, you can connect the following ports:

~~~
$ yarp connect /dispBlobber/roi/left:o /objectRecognizer/roi:i
$ yarp connect /dispBlobber/blobs/left:o /objectRecognizer/centroid:i
~~~

in order to get the stream of coordinates as input. By setting, via RPC port, the modality of the `objectRecognizer` to `centroid` or  `roi` (instead of `fixed` as it is by default), you can then get the desired result.

#### Run with different models

This module can run with any valid Caffe model, provided that the needed configuration/weight files are available.

1. For example, you can configure the module in order to use the network that we trained in the [tutorial](https://github.com/vvv-school/tutorial_dl-tuning), rather than the one trained in the assignment. To do this, you can go through the steps that we described in this README, and configure the `objectRecognizer.ini` file to point to the related `final.caffemodel` and `labels.txt` files, and to a `deploy_MemoryData_caffenet.prototxt` that you will have set with a different number of outputs (2 in the case of the tutorial) and the image mean of the training set used in that case.

2. Furthermore, you can repeat the [assignment](https://github.com/vvv-school/assignment_dl-tuning) by training on a task of your choice (remember that the train/val/test image sets are configured in the [imageset_config.yml](https://github.com/vvv-school/assignment_dl-tuning/blob/master/cat_4categories/imageset_config.yml) file), and deploy the resulting model in YARP with the provided `objectRecognizer`.

3. Finally, if you want to play more with these things, you can also acquire your own image sequences, train a network on them (by using the scripts that we provided in the [assignment](https://github.com/vvv-school/assignment_dl-tuning)) and, finally, deploy it in YARP with the `objectRecognizer`. To do this, you can follow these general instructions (and ask for help if needed!):

    1. Run a `yarpdev --device opencv_grabber`, like we did to stream images in YARP from your laptop's camera
    2. Connect its output to a [yarpdatadumper](http://www.yarp.it/yarpdatadumper.html) to record image sequences on disk
    3. Process the sequences (manually, or with a simple script that you can try to write) to match the iCW dataset format. Note that if you want to change the directory tree apart from changing the names of the categories, you have to modify accordingly not only the [imageset_config.yml](https://github.com/vvv-school/assignment_dl-tuning/blob/master/cat_4categories/imageset_config.yml) file, but also the two files in [python-utils](https://github.com/vvv-school/assignment_dl-tuning/tree/master/scripts/python_utils).
    4. Adapt the [train_and_test_net.sh](https://github.com/vvv-school/assignment_dl-tuning/blob/master/cat_4categories/train_and_test_net.sh) script to use (i) your own dataset instead of iCW and (ii) the network configuration files (that you will have previously configured considering the new classification task). To this end, consider that, once you have generated your own train/val/test image lists, all the rest of the pipeline is quite unchanged.
