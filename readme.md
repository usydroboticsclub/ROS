# ROS tutorial


## Recording
The password for these recordings is p1*xvnUWRgaC

The below links will expire on 13-11-2020 and will become unavailable for download after this date.

Video (122.84 MB)

https://cloudstor.aarnet.edu.au/plus/s/FkDpeZvkzhkIJQX

Audio Only (14.96 MB)

https://cloudstor.aarnet.edu.au/plus/s/A3X79sd3cPxYcaQ


## What is ROS? Why should I use it?
ROS stands for Robot Operating System. In reality, it is not an operating system, so much as a communications framework between different programs.

So far, the programs we've encountered work by themselves. We assume that they interact with static files and read and write to them, without another program getting in the way. 

However, in robotics, there can be many different sensors and control modules involved, creating complexity. If you wanted to have a single program manage them all, it would have to be very big; and it would all have to be written in a single language, even though there may be a tradeoff between ease-of-use (python) and low-level communication (C). 

Additionally, you may have to code everything from scratch, which would take you a lot of time.

Instead, we can use ROS! ROS allows programs to talk to each other, making it possible to use C code, C++ code, python code, and more, all in the same projects. ROS also solves a lot of different issues and has a very rich community.

## Some terminology
ROS is made up of **nodes**. A node is either a compiled program or a script that does something; e.g. you might have a camera node that reads in from a camera, or a processing node that fetches information from a camera, or an output node that spins up a motor. 

Groups of nodes make up **packages**, which are simply collections to make it easier for people to download related toolkits. 

ROS has communication channels known as **topics**. A node can **subscribe** to one or more topics, and can **publish** to one or more topics. Each topic contains a specified type of **message**, which is a defined data structure. For example, there may be a topic called *memes* which contains *image* data.

A more advanced feature of ROS is a **service**. A ROS service is available to all channels and is a way of specifying actions that can be performed by a node. 

Another useful feature of ROS is its **parameter** server. These parameters can be changed quickly to deal with certain situations; for example if you move a robot from California to Sydney, you may edit its home location as a parameter.

## Ok, sure. How can I use ROS?
1. Get a linux machine. Windows subsystem for linux isn't good enough. Mac is most certainly not good enough.
2. Install ROS, by following these steps: http://wiki.ros.org/melodic/Installation/Ubuntu (This will involve a lot of typing on a terminal, so I hope you're comfortable with CLI!)
    1. You may want to install catkin as well, which we'll be using for this tutorial: http://www.ros.org/wiki/catkin#Installing_catkin
    2. You will probably also want the RQT tools, which we will also be using in these tutorials: http://wiki.ros.org/rqt
3. Read the tutorials: http://wiki.ros.org/ROS/Tutorials. But we'll walk through it here with you now!


## Tutorial
Today, we're going to learn: 
1. How to create a workspace.
2. How to make a new package.
3. How to subscribe to a topic.
4. How to publish to a topic.
5. How to debug our ROS project.
6. How to make a launch file
7. How to record and play back data in a bag file

## 1 Making a workspace
A workspace is where all your ros work will go. We've discussed how **nodes** make up **packages**; these **packages** go in a **workspace**.

To create a workspace, all we need to do is make a few directories, then run `catkin_make`:

```
mkdir workspace
cd workspace
mkdir src
catkin_make
```
You should see catkin's output, and once the dust has settled, run `ls` to make sure there are three folders in your workspace: src, build and devel.

Now there's one more thing to do, which is to tell catkin that we intend to work in this workspace. So type this command:

```
source devel/setup.bash
echo $ROS_PACKAGE_PATH
```
And we're good to go! In the output, you should see your working directory listed as part of the ROS_PACKAGE_PATH.
## 2 Making a new package
As mentioned earlier, a package contains multiple nodes. As with making a workspace, this is common enough a task that there is a script to make packages for us. Run:
```
cd ~/catkin_ws/src
catkin_create_pkg hello_world std_msgs rospy roscpp
```
Note the extra arguments in the end of the `catkin_create_pkg` command: hello_world, your package name; and the dependencies std_msgs, rospy and roscpp. There are plenty more things you can do when you're creating a package - you can read more about them here: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

Our package is pretty empty right now, so let's head inside and make a file to get ourselves started:
```
cd hello_world
cd src
```
Now, we can put either source code as in .c files here, or python files as in .py here. Let's go python because I'm more familiar with python. (You can check out C code in the ROS tutorials: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

Let's make a python file in our directory called `hello_world.py`, and fill it with this code:
```python
#!/usr/bin/env python
# ^ the above line is called a `shebang`, and it just tells the shell to use python as an interpreter.

import rospy
# rospy is a package, just like anything else!

def node():
    rospy.init_node('helloworld', anonymous=True) # anonymous=True means that our name can be changed and we can have multiple instances of the same node.
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("I'm alive!")
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
```
You may feel this is a lot for a hello world, but this is nothing - it gets a lot more complex from here!

How do we run our node? Let's go back to our first terminal, and run:
```
chmod +x hello_world.py # tell the system we're allowed to execute this file
roscore # start the ros core process
```
And now, open a new terminal, and run:
```
cd your_workspace_location
source devel/setup.sh # you'll need to do this a lot; if you're just working in a single workspace, add the source command to your .bashrc
rosrun hello_world hello_world.py
```
Note that we need to specify both the package name and the executable name in our rosrun, because multiple packages may have the same executable files.

And we're live!

## 3 Publishing to a topic
Now, I promised you that ROS would allow you to let your programs talk to each other. Let's give that a go, shall we?

Head back to `src/hello_world/scripts` and make a new file called `jkrowling.py`. In the file, add this code:

```python
#!/usr/bin/env python

import rospy

from std_msgs.msg import String

def node():
    rospy.init_node('jkrowling', anonymous=True)

    pub = rospy.Publisher('someChannelName', String, queue_size=10) # String is the datatype, and if the channel is busy then we'll hold up to 10 messages in our local buffer.
    sales = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        pub.publish("Did you know I've sold {} Harry Potter books?".format(sales))
        sales = sales + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
```
And let's stop our hello_world.py (but don't stop roscore!) and run:
```
chmod +x jkrowling.py # You only need to run this once, subsequently you can run rosrun directly
rosrun hello_world jkrowling.py
```

It appears that jkrowling is silent for now, but if we open up a third terminal and type in:
```
rostopic echo someChannelName
```
We can see that jkrowling is sending out messages through ros that we can access in another termial (and indeed, in another program!)

## 4 Subscribing to a topic
Of course, we would like to now do something with this information that is being sent over ROS, in another program that we've written.
Let's make another file now, called hpfan.py:
```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    jksPreciousWords = data.data
    bookcount=0
    for word in jksPreciousWords.split():
        if word.isdigit():
            bookcount=word
    rospy.loginfo("Oh my god. Apparently JK rowling has sold {} books!".format(bookcount))
    
def listener():

    rospy.init_node('hpfan', anonymous=True)

    rospy.Subscriber("someChannelName", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

```
And when we run it in a separate terminal, with jkrowling.py still in the background, we get:
```
Oh my god. Apparently JK rowling has sold 8339 books!
Oh my god. Apparently JK rowling has sold 8340 books!
Oh my god. Apparently JK rowling has sold 8341 books!
```
## 5 Debugging the project
Let's spin up a few more harry potter fans to create our own little harem:
```
source devel/setup.sh
rosrun hello_world hpfan.py
```
You can imagine spinning up multiple copies of a node that controls one leg of a four legged robot; or multiple camera publishers, for example, that publish to a single camera channel.

Now that we have a few nodes, say one of them goes down. How do we know which one is having problems? ROS comes with a toolkit called `rqt` to help us figure out what is going on. Let's run `rqt_graph`:

![ros graph](https://github.com/usydroboticsclub/ROS/raw/master/rosgraph.png "Adoring fans")

We can see which messages are going where. Great! Now, if one of hp's fans gets decommissioned, we will see that there are only two left over.
## 6 Making a launch file
Now, it would definitely be a pain to have to hand run all 50 nodes in a complicated robot setup; so we can create launch files to launch complex setups in one go. Let's make a launch file in a new folder `hello_world/launch` called `harem.launch`:

```xml
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <node pkg="hello_world" type="jkrowling.py" name="jkrowling"></node>
  <node pkg="hello_world" type="hpfan.py" name="fan1"></node>
  <node pkg="hello_world" type="hpfan.py" name="fan2"></node>
  <node pkg="hello_world" type="hpfan.py" name="fan3"></node>
</launch>
```
Then we can launch it using `roslaunch hello_world harem.lauch`. As an added bonus, it will run roscore for us if roscore is not started. Note you can also include nodes from other packages, as long as they're installed.

## 7 Recording and playing back data
There's one last trick we'll go through in this tutorial: recording data! Recording data is super useful when we want to re-simulate controllers or detectors based on data we've collected in the past; but nothing beats a real live test!

To demonstrate our recording capaiblity, let's get jk rowling to shout into an empty channel for some time:
```
rosrun hello_world jkrowling.py
```
But we'll also record her:
```
mkdir ../../recordings
cd ../../recordings
rosbag record -a
```
And after some time, let's turn those off. We'll see a new file [date].bag in our recordings directory; this is a .bag file and is compressed because real robot data comes a lot faster and a lot harder than a stream of tweets. We can't open the .bag file easily, but we can play it back. Let's first get a hpfan going:
```
rosrun hello_world hpfan.py
```
And now play our bag file:
```
rosbag play [date.bag]
```
You should see the hpfan jump to life. Great!

## More resources
ROS has a few more tricks up its sleeve, including **services**, **image handling**, **a whole stack of libraries**, **rosparam**, and even **operating across physical machines**. These can be very powerful features, and the first step to discover them is to check out the real ROS tutorials at the official website: https://wiki.ros.org. Happy coding!
