# LUCI Transforms Package #

This is the package responsible for all sensor to base_link transformation topics.

## How to build and run this package ##

### Dependencies ###
There are numerous dependencies needed to run and build this package including ROS2 and fakeroot. Rather then have to manually install all of these on your system a development docker image was made and is hosted on jfrog. To download it run `docker run -d -it -p 8765:8765 luci.jfrog.io/ros2-sdk-docker-local/luci-sdk-development-image:latest`. This will download the latest image made by the [luci-sdk repo](https://github.com/lucimobility/luci-ros2-sdk). This image should have all of the needed dependencies excluding any custom ROS packages made such as `luci_messages`, you will still need to get those yourself. 

In order to use the image please follow the instructions in the [SDK repo](https://github.com/lucimobility/luci-ros2-sdk/blob/main/docs/1_Getting-Started/2_docker.md) for connecting to a docker image 


You will also need to add an ssh key to get access to the luci_messages repo. Instructions for that can be found in the development folder of the [SDK repo](https://github.com/lucimobility/luci-ros2-sdk).

Once you are setup with the development image you can start building the package.

As mentioned above the gRPC node created in this repo relies on the `luci_messages` package as a dependency. That means in order to build the gRPC package you must first build and source the `luci_messages` package. Clone the `luci_messages` repo to your dev image and build it. For more detailed instructions please refer to its [repo](https://github.com/lucimobility/luci-ros2-msgs).

Once you have the proper dependencies installed and/or sourced you can begin building the gRPC package. 

### Build Steps ###
1. cd into the directory `/luci-ros2-transforms/` (should be next to the package.xml file)
2. `colcon build`
3. `source install/setup.bash`

### Options for running ###
There are two things that can be launched with this packages:
1. A launch file to launch all entire robot with meshes, TF etc.
    - You can run this using `ros2 launch luci_transforms display.launch.py`. This will launch the entire robot model in rviz. 
2. A tf node to spin up all of LuciSandbox sensor tf's:
    - You can run this using `ros2 run luci_transforms luci_dev_kit_tf_node`
    - You can also run this using `ros2 launch luci_transforms luci_dev_kit.launch.xml`

NOTE: There is also a `build-package.sh` script available in this repo that will not just build the ROS package but also build a .deb file for install. This is not usually needed when developing but is an option to run instead of the commands above `./build-package.sh`. This is the same script called by the github actions to automate the release process. 

## Releasing new version ##
When a new version of this package is ready to be released there are a couple steps to follow. It is important to note that most of the process is automated for convenience and the process should be just a couple of button clicks.

### Steps ### 
1. Update release version
    - This should be its own separate PR and should only update the package.xml `<version> </version>` tag. 
    - LUCI follows [semver](https://semver.org/) style versioning so MAJOR.MINOR.PATCH versions are expected.
    - It is okay to not put out versions until multiple changes have happened to the code. 
2. Once the version increment is merged you simply need to create an official release in github. Make sure you make the release version the same as what is now in `package.xml`. We have chosen to keep github release and package version in sync.
    - This should trigger an action to auto run called `Create and Sign Package` which you can monitor in the github actions panel. This should grab the released code, build it, make an installable .deb file, gdb sign it and push it to jrog artifactory. 

If everything went smoothly congratulations the new package will be released and publicly distributable.


<b>NOTE: Once a PR is merged into the `main` branch the docs site in the `next` version will update with it that evening.</b>


## Implementation ## 
More detailed implementation docs can be found here [Implementation](docs/luci_transforms_package.md)
