# Catkin Workspace for ROS(Robot Operation System)

ROS provides a powerful build and package management system called Catkin. Willow Garage (the early developers of ROS) named Catkin after the flowers on the willow trees surrounding their office. A Catkin workspace is essentially a directory where Catkin packages are built, modified and installed.

Similar to workspaces, which hold a wide variety of Catkin packages, Catkin packages are nothing more than directories containing a variety of resources which when considered together constitute some sort of useful module. Catkin packages may contain source code for nodes, useful scripts, configuration files and more.

In the sections you will see how to create a Catkin workspace, add packages to it, manage inner package dependencies and lastly, how to successfully compile everything.

# 1. Create a Catkin Workspace

#### Step 1: mkdir -p  ~/catkin_ws/src

All of the ROS related code you develop throughout this course will reside in your catkin workspace. You only need to create and initialize the workspace once.  First, create the top level catkin workspace directory and a sub-directory named **src** (pronounced source). The top level directory’s name is arbitrary, but is often called **catkin_ws** (an abbreviation of catkin_workspace), so we will follow this convention. You can create these two directories with a single command: 

	mkdir -p ~/catkin_ws/src

#### Step 2: cd ~/catkin_ws/src

Next, navigate to the src directory with the cd command:

	cd ~/catkin_ws/src

#### Step 3: catkin_init_workspace

Now you can initialize the catkin workspace:

	catkin_init_workspace

<p align="center">
<img src="./img/1.png" alt="catkin_init_workspace" />
<p align="center">
