# 2 ROS Hello world

At a high level, this tutorial has the following parts:
1. Spin up the container from the previous part
2. Create a ros workspace and package
3. Save your work

## 1 Spin up the container from the previous part
- If you followed all the steps from the previous part, you should be able to spin up the container by typing `docker run -it -p 5900:5900 --name ros_container ros/with_vnc`.
    - If you get a  "The name is already in use", run `docker container rm ros_container`
        - If you try `docker container rm ros_container` and it complains about the container running, run `docker kill ros_container`, then `docker container rm ros_container` again.

## 2 Create a ROS workspace and package
1. Let's work in the `home` directory: type `cd home`. 
2. Make a folder for the workspace: `mkdir catkin_ws && cd catkin_ws`.
3. Make a `src` folder (that catkin_make will look for in the next step): `mkdir src`
4. Prepare the workspace using `catkin_make`.
    - Typically, a new workspace for each project is good to keep things in separate piles.
5. Tell your shell that you want to use this workspace using the command `source devel/setup.bash`.
6. Head into the `src` directory and take a look: `cd src && ls`. You should find a file called `CMakeLists.txt`.
7. Now, make a package called `catkin_create_pkg usrc_tutorials std_msgs rospy`.
    - `usrc_tutorials` is the name of the package and `std_msgs` and `rospy` are dependencies.
8. Go into the package and take a look around: `cd usrc_tutorials && ls`.
    - You should see a `CMakeLists.txt`, a `package.xml` and a `src`.
9. Make a `scripts` folder: `mkdir scripts`.
    - By convention, the `src` folder is used for compilable source code (C++), whereas `scripts` is for non-compiled scripts like python. You don't have to follow convention if you're in a rush.
10. In a separate (host) terminal, cd to the location of this readme and copy in our tutorial file: `docker cp hello_world.py ros_container:/home/catkin_ws/src/usrc_tutorials/scripts/hello_world.py`. 
11. Use this separate terminal to activate a second shell in the container: `docker exec -it ros_container /bin/bash`
12. In your second shell, source the `setup.bash` again: `source home/catkin_ws/devel/setup.bash`.
13. In the second shell, enter `roscore &`.
    - `roscore` is the core process of ROS which handles all the baseline communications.
14. In your second shell again, type `rosrun usrc_tutorials hello_world.py`.
    - This tells ros to run the script `hello_world.py` in the package `usrc_tutorials`.
    - If you get the error `Cannot find executable hello_world in usrc_tutorials`, run `chmod +x /home/catkin_ws/src/usrc_tutorials/scripts/hello_world.py` to make the python file executable.
15. Back in your first shell, run `rostopic echo some_channel`. 
    - You should see a stream of `data: "Hello world!"` If you can, then it's working!
    - We will discuss where this is all coming from in the next tutorial.
16. Use `control C` to terminate the `rostopic echo` and your `rosrun usrc_tutorials hello_world.py`.

## 3 Save your work
We'll need to do all these steps before we begin developing in earnest, and because we're running in a container environment, we should save our changes.
1. Enter `echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc`
    - This tells our shell to run the command `source catkin_ws/...` each time we open a shell for us, so we don't have to do it later.
2. Save your container using `docker commit ros_container ros/with_ws` in a host terminal.
3. Clean up with `docker kill ros_container && docker container rm ros_container`.

And you're all set for next time.

## References
This tutorial was created with the following resources:
- http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- http://wiki.ros.org/catkin/Tutorials/CreatingPackage
- https://hackernoon.com/installation-of-vnc-server-on-ubuntu-1cf035370bd3