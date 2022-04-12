# giraff_dsr
Intro to component here

To run this agent you need to follow these steps:

- Install this library in your machine (~/software): 
    git clone https://github.com/danielaparker/jsoncons
  mkdir build; cd build; cmake ..; make; sudo make install

- Clone this library:
    git clone https://github.com/zeromq/cppzmq
  copy these two files: zmq_addon.hpp and zmq.hpp to /usr/local/include

Goto to your Coppelia directory and cd to CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/cpp
  - mkdir build; cd build; cmake ..; make
  - copy these files: 
        ~/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/cpp/RemoteAPIClient.h to /usr/local/include
        ~/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/cpp/RemoteAPIObjects.h to /usr/local/include 
        ~/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/cpp/build/libRemoteAPIClient.a to /usr/local/lib




## Configuration parameters
As any other component, *giraff_dsr* needs a configuration file to start. In
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
cd <giraff_dsr's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/giraff_dsr config
```
