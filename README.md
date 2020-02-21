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

<p align="left">
<img src="./img/1.png" alt="catkin_init_workspace" />
<p align="left">

Let’s list the contents of the current directory to see what changed.

	ls -l
Notice that a symbolic link (**CMakeLists.txt**) has been created to **/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake**

<p align="left">
<img src="./img/2.png" alt="ls" />
<p align="left">
	
#### Step 4: cd ~/catkin_ws
Return to the top level directory:

	cd ~/catkin_ws

#### Step 5: catkin_make
build the workspace.
Note: you must issue this command from within the top level directory (i.e., within catkin_ws NOT catkin_ws/src)

	catkin_make

While it is not essential that you have a deep understanding of what the catkin build system is, particularly if you are doing most of your development work in Python, it is helpful to learn about it. The curious reader is encouraged to read the [ROS wiki](http://wiki.ros.org/catkin/conceptual_overview). After the command is executed you will notice the output of the build processes being echoed to your display. When it has finished you should see the following lines at the end of the output:

	-- BUILD_SHARED_LIBS is on
	-- Configuring done
	-- Generating done
	-- Build files have been written to: /home/robo/catkin_ws/build
	####
	#### Running command: "make -j2 -l2" in "/home/robo/catkin_ws/build"
	####
	robo@robo-virtual-machine:~/catkin_ws$

But what else has changed? Use the **ls** command again to see what is new.

<p align="left">
<img src="./img/3.png" alt="ls" />
<p align="left">
	
You now have two new directories: **build** and **devel**. **The aptly named build directory is the build space for C++ packages** and for the most part you will not interact with it. **The devel directory does contain something of interest, a file named setup.bash**. This setup.bash script must be sourced before using the catkin workspace. 
