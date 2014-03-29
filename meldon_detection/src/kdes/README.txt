
This is a live camera demo for object recognition using kernel descriptors.
It supports Linux (tested under Ubuntu 12.04 x64) and Windows (x86,
Visual Studio 10). Current version 1.2.3.

This program is being developed by Xiaofeng Ren at Intel Labs, and free for
non-commercial uses. See COPYRIGHT.txt.

We will not be able to provide extensive technical support for the software or
detailed explanations for the algorithms.  Nonetheless, if you have questions,
contact xiaofeng.ren@intel.com.


---INSTALL (Linux)---

The binaries included are compiled on Ubuntu 12.04 x64.

To build and run this program, you will need three external libraries:
OpenCV-2.3, Eigen, matio (the latter two included).

To install OpenCV, please see (note you need to enable V4L for live demo)

http://opencv.willowgarage.com/wiki/

Once you have done so, run ./install-dependencies.sh and it should install both
Eigen and matio.

To build demo programs, do

cd ./KernelDescriptors_CPU
make


---INSTALL (Windows)---

The binaries included are compiled on Windows 7 with Visual Studio 10 (it's
win32 release build).

To build it yourself, open the Visual Studio 10 Project in
./KernelDescriptors_CPU/vs10_project/KernelDescriptors_CPU/

Aside from OpenCV, you need to install the (binary) boost library for Windows 
http://www.boostpro.com/download/

The VS10 project assumes the boost library installed under C:\SDK\boost. You
may need to unzip a few files in boost -- check the compiling errors.

To run, copy KernelDescriptors_CPU.exe to
./KernelDescriptors_CPU\vs10_project\Current_Build

Note you probably need to specifically install a driver for your webcam (such
as for Logitech ones).


---RUN DEMO---

The main demo program is kernelmain (or KernelDescriptors_CPU.exe in Windows)

It has two modes: (1) is live demo modes, trying to read from a webcam and
classify the object that occupies the center of the frame.  (2) is an offline
mode that goes through all the images in ./testim. Note I have removed the
grabcut mode, it's slow and does not work very well anyway.

The live demo now uses a running average of classification scores by default,
making the predictions much more stable than before. Use the 'm' key to toggle
between the running average mode and the single-frame classification mode.

The included modelgkdes.mat is a model of 12 classes (trained using 50 images
per class from ImageNet):

    'apple'
    'banana'
    'box'
    'cellphone'
    'coffee_mug'
    'key'
    'keyboard'
    'laptop'
    'marker'
    'soda_can'
    'wallet'
    'water_bottle'

Best recognition experience is achieved when you focus the camera view on the
object from a typical angle, as example images in ./KernelDescriptors_CPU/testim/,
since the models are trianed on such ImageNet style object photos.

Another program computekdes takes a directory and a descriptor type (gradient
or color), computes kernel descriptors for all the images in the directory, and
outputs to a text file.

computekdes is a standalone program that computes kernel descriptors (of a
single type) for all images in a directory, and save in one text file. For
example, to compute gradient kdes for all the JPEG images in ./testim

./computekdes ./testim/ ./testim_kdes0.txt 0

Currently there are two depth descriptors implemented: depth gradient kernel
descriptor (type=3), and surface normal kernel descriptor (type=4). They can be
called in offline mode from computekdes, such as

./computekdes ./testim_depth/ ./testim_depth_kdes3.txt 3
./computekdes ./testim_depth/ ./testim_depth_kdes4.txt 4

by going through all files with suffix "_depthcrop.png". The surface normal
descriptor also requires an associated "_loc.txt" which provides the center
location of the cropped patch in the original RGB-D frame (640x480). The
cropped depth images in ./testim_depth are samples from the much larger
UW-Intel RGB-D dataset

http://www.cs.washington.edu/rgbd-dataset/


---TRAIN YOUR OWN MODEL---

The demo uses the model file (matlab format) modelgkdes.mat in
./KernelDescriptors_CPU/model (for Windows build,
./KernelDescriptors_CPU/vs10_project/Current_Build/model/).

To train your own model, you would need to

1) compute kernel descriptors (e.g. using computekdes)
2) linearly scale the descriptors for training data (see scaletrain.m and
        scaletest.m in ./KernelDescriptors_CPU/model, use 'linear' option)
3) train linear SVM on scaled descriptors (e.g. in liblinear)
4) in matlab/octave, replace the following variables in modelgkdes
        modelgkdes.svm.nr_class		;   number of classes
        modelgkdes.svm.Label		;   (1:nr_class)
        modelgkdes.svm.w		;   weights from linear SVM
        modelgkdes.svm.minvalue		;   from scaletrain.m
        modelgkdes.svm.maxvalue		;   from scaletrain.m
        modelgkdes.svm.classname	;   names of classes


---REFERENCE---

For the kernel descriptor framework and image descriptors,

Kernel Descriptors for Visual Recognition
Liefeng Bo, Xiaofeng Ren, Dieter Fox
Neural Information Processing Systems (NIPS), 2010

For depth kernel descriptors:

Depth Kernel Descriptors for Object Recognition
Liefeng Bo, Xiaofeng Ren, Dieter Fox
International Conference on Intelligent Robots and Systems (IROS), 2011

