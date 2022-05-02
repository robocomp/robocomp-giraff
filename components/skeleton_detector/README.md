# skeleton_detector
Intro to component here

To run this component you need to install the following libraries:

https://developer.nvidia.com/nvidia-tensorrt-8x-download  (the version that goes with your IS and CUDA version )
sudo pip3 install nvidia-pyindex
sudo pip3 install nvidia-tensorrt
sudo apt install libnvinfer-dev

https://github.com/NVIDIA-AI-IOT/trt_pose (follow instruction)
Download the weights files that you need from that site and store them in skeleton-detector (densenet121_baseline_att_256x256_B)


## Configuration parameters
As any other component, *skeleton_detector* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <skeleton_detector's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/skeleton_detector config
```
