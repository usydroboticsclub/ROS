# 3 ROS Pub/Sub

At a high level, this tutorial has the following parts:
1. More ROS terminology
2. Making a Publisher/Subscriber pair
3. Using RQT to visualise the setup
4. A more complex setup
5. Some exercises

## 1 More ROS terminology
So far, we've learnt about the package structure of ROS:
- You have a **workspace** on each person's device.
- You create **packages** to contain units of work.

Next, we're going to discuss the ros operational environment.
- You've already met `roscore`: the core service behind ros.
- When you run a command like `rosrun usrc_tutorials hello_world.py`, you're creating a ros **node**.
- ros nodes can talk to each other using ros **topic**s. A topic is a channel that information can flow on.
    - When you write to a topic, this is known as **publish**ing, or being a **publisher**.
    - When you read from a topic, this is known as **subscribe**ing, or being a **subscriber**.
- Topics are great for information sharing, but for issuing commands, ROS also has **services**. 
    - One ros node can ask another to perform a service, and be notified when the service is completed.

## 2. Making a publisher and subscriber pair
1. Check out the code in the files `hello_world_pub.py` and `hello_world_sub.py`.
    - Notice how one is a publisher and one is a subscriber, and they both listen on the topic `some_channel`.
2. Start up the container from last time with `docker run -it -p 5900:5900 --name ros_container ros/with_ws`
3. In a separate host terminal, copy in the python files from this directory: `docker cp hello_world_sub.py ros_container:/home/catkin_ws/src/usrc_tutorials/scripts` and `docker cp hello_world_sub.py ros_container:/home/catkin_ws/src/usrc_tutorials/scripts`.
4. In your container, run `roscore &`. 
    - You will need to hit ENTER again to regain control when it is done.
5. Also run `rosrun usrc_tutorials hello_world_pub.py &` to start the publisher.
7. Now run `rosrun usrc_tutorials hello_world_sub.py`. You should see some messages coming out.

## 3. Using RQT to visualise the setup
1. Open another container shell: `docker exec -it ros_container /bin/bash`
2. Start the vnc server, since we're going to be doing something with the GUI: `source vnc.sh`
    - You will need to hit ENTER again to regain control when it is done.
    - For advanced shell users - we're using `source` here instead of `./vnc.sh` so that the `export` statement inside will set our desktop (otherwise rqt_graph will be confused.)
3. Install `rqt_graph`: `apt install ros-melodic-rqt-graph`
    - rqt_graph draws pictures showign you what your ros nodes are doing.
4. Run `rqt_graph`.
5. Open VNC Viewer on your host machine and point it at `localhost:5900` again. You should see a window.
6. Expand the window and zoom in (scroll). You should see a bubble `hello_world_pub_xxxxx` pointing towards `hello_world_sub_xxxx`.
    - The numbers are there because we set `anonymous=True` in our python files. This means that we can spawn multiple copies of `hello_world_sub`. Let's try that now!

## 4. A more complex setup
1. Open a new container shell using `docker exec -it ros_container /bin/bash`.
2. Run another subscriber: `rosrun usrc_tutorials hello_world_sub.py`.
3. Go back to your VNC. In the `rqt_graph` window, press the refresh button in the top left corner.What has changed?
4. Save your work - rqt_graph will continue to come in handy. `docker commit ros_container ros/with_rqt`
    - Don't forget to clean up when you're done with your exercises!

## 5. Some exercises
1. Check out what happens with `rqt_graph` when you run multiple publishers.
1. Check out what happens with `rqt_graph` when you run `rostopic echo some_channel`.
2. Change the dropdown in the top left corner of `rqt_graph` to Nodes/Topics (active). What changes?
3. Make a new publisher that publishes an increasing counter.
4. Make a ros node that listens on one channel and publishes the data to another channel.

Great! Everything is coming along. 

## References
This tutorial was created with the following resources (indirectly):
- http://wiki.ros.org/ROS/Tutorials