# 1 Installing with Docker

At a high level, this tutorial has the following parts:
1. Install docker
2. Get ROS on docker
3. Install VNC in the machine
4. Test everything works

This is somewhat different from running ROS in a production environment, but we'll have a separate tutorial for that if you want. 


## 1 Installing docker
- ROS was built for Linux, because linux is great for building lean, low cost, technically challenging things, like robot operating systems.
- However, this means it doesn't work on your regular MacOS or Windows operating systems.
- Thankfully, there's Docker, which spins up a linux environment on any operating system.

1. Download Docker: https://www.docker.com/products/docker-desktop
    - The community edition will be fine, no need to pay for it :D
    - If you're on windows, make sure to follow the steps to activate WSL. 

## 2 Installing ROS on docker
Docker uses 'containers', which are self-contained setups. We're going to use the ROS container.
1. Open a command window (terminal or cmd or powershell).
2. Type in `docker pull ros:melodic-robot-bionic`. Wait for the operation to complete.
3. Type in `docker run --name=ros_container -p 5900:5900 -it ros:melodic-robot-bionic`. This opens a shell where you can do ROS stuff.
    - `--name=ros_container` specifies the name, so you can come back to it later.
    - `-p 5900:5900` maps the port 6001 on the container to the port 6001 on your local computer.
    - `-it` means you want a shell and the container should stay open instead of trying to run in the background.
    - `ros:melodic-robot-bionic` is the name of the image from which the container is being created from.

## 3 Install VNC in your ROS machine
VNC is a virtual desktop viewer. Your docker container doesn't have access to your desktop because it's running with as few resources as it can, so you need to explicitly tell it to start a graphics display that you can view.
1. In your shell that you got from the previous part, type in `apt update`. 
    - (`apt` is how you get new software in linux. `update` tells it to check what software is available.)
2. Next, type in `apt install x11vnc xvfb firefox fluxbox`. 
    - `x11vnc` gets the vnc server package.
    - `xvfb` is a virtual frame buffer. Since we don't have a screen, we need a frame buffer to draw to.
    - `firefox` is firefox. The ROS package was so small they cut out plenty of essentials...
    - `fluxbox` is a lightweight window manager.
3. Start a virtual desktop, by typing `export DISPLAY=:0 && Xvfb -screen 0 1600x1200x24+32 &`.
    - `export DISPLAY=:0` tells the commands in the subsequent steps to use a common display (number 0).
    - The `&&` allows you to run a second command after the first.
    - `Xvfb -screen 0 1600x1200x24+32` starts the virtual desktop buffer. The screen option sets some colour options so the colours don't mess up later.
    - The final `&` means 'run in the background'.
3. Start the vnc server, by typing `x11vnc &`.
    - There will be a warning message about not having a password. This is fine - you're only talking between your computer and the container anyway.
    - After a few seconds when the output stops, hit ENTER again to regain control.
4. Start fluxbox by typing `fluxbox &`.
    - Again, after a few seconds wehn the output stops, hit ENTER to regain control.
5. Give the vnc something to show by typing `firefox &`.

Leave this container running for the next steps:
1. Install VNC viewer from https://www.realvnc.com/en/connect/download/viewer/. (It's free!)
2. Open VNC viewer. In the top bar, type `localhost:5900`.
    - Ignore the error message that comes up.
3. You should see a firefox browser!

## 4 Creating a workflow
For the subsequent lessons, your workflow will look like this:
1. Edit some files in the safety of your host operating system.
2. Create a copy of the base container with the vnc.
3. Upload files to your base container.
4. Open one or more shells to the container to run commands.
5. View the results using VNC.

Let's use this workflow to complete the last step of our installation.
1. Create a new terminal and CD to the directory of this readme.
2. Run `docker cp vnc.sh ros_container:/vnc.sh`
    - This copies the local file to your container. You can also do the other way around by swapping the arguments.
    - The vnc.sh file has all the commands necessary to run a vnc server from the command line that we did in the previous part, except for launching the browser.
3. Save your container as an image: `docker commit ros_container ros/with_vnc`.
4. It's time to say goodbye to our container :( Run `docker kill ros_container`, then run `docker container rm ros_container`.
    - If you don't run `docker container rm ros_container`, you won't be able to use the container name `ros_container` in the future.

And now, let's start everything up one last time:
1. Run `docker run -it -p 5900:5900 --name ros_container2 ros/with_vnc`
2. In the shell you created, run `./vnc.sh`
3. [new] Let's create a separate new shell that is running in the same container, with `docker exec -it ros_container2 /bin/bash`.
4. In this new shell, run `export DISPLAY=:0 && firefox &`.
5. Check your vnc with the same address as before `localhost:5900`.
6. Clean up with `docker kill ros_container2` and `docker container rm ros_container2`.

Congrats! You now have a container setup capable of running ROS!

## References
This tutorial was created with the following resources:
- http://wiki.ros.org/docker/Tutorials/Docker
- https://stackoverflow.com/questions/16296753/can-you-run-gui-applications-in-a-linux-docker-container