<div align="justify">

# 1 Fundamental ROS Concepts
Welcome to the very first lesson of this ROS MOOC! Before we dive into ROS, it is helpful to know something about how ROS works internally.

## Nodes
A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. Nodes provide modularity to robotic projects that use ROS. They are often written in C++ or Python. In this course, we will use Python to write them. ROS Noetic compatible with Python 3.x.

The use of nodes in ROS provides several benefits to the overall system. There is additional fault tolerance as crashes are isolated to individual nodes. Code complexity is reduced in comparison to monolithic systems. Implementation details are also well hidden as the nodes expose a minimal API to the rest of the graph and alternate implementations, even in other programming languages, can easily be substituted. 

### Node Tools
There are three main ways to inspect a ROS node:

- Simply list all currently running nodes in the terminal:
`rosnode list`

- Make a visual graph of all running nodes and their connections:
`rqt_graph`

- List information about a specific node in the terminal:
`rosnode info <node_name>`

In the following sections, we will learn much more about Nodes. For example, we will see that they can publish (write) to a topic and subscribe (read) from a topic.

## Topics
Topics are named buses over which nodes exchange messages. These messages are organized as data structures and can have different data types. Topics can be identified by their name and their type. You can think of topics as strongly typed message buses. Any node can connect to it to send or receive data, as long as they are the right type.

Basically, topics have anonymous publish/subscribe semantics, which decouples the production of information from its consumption. In general, nodes are not aware of who they are communicating with. Instead, nodes that are interested in data subscribe to the relevant topic; nodes that generate data publish to the relevant topic. There can be multiple publishers and subscribers to a topic. 

> `💡` **NOTE**
> 
> Topics are intended for unidirectional, streaming communication. Nodes that need to perform remote procedure calls, i.e. receive a response to a request, should use services instead. There is also the Parameter Server for maintaining small amounts of state. 

### Topic Tools
As with nodes, there are three ways to inspect topics:

- Display a list of all topics that are currently being exchanged between active nodes
`rostopic list`

- Display information about the data structure of a specific topic
`rostopic info <topic_name>`

- Print the current content of a topic in the terminal
`rostopic echo <topic_name>`

---

In a real robot application you will often have to deal with a large number of nodes and topics. It is important to know which nodes are talking to each other, and what topics are being used to pass the information (messages) between nodes.

---

> ## 🎯 Question 1.1.
> In robotics, we often have to interact with the real-world. One type of device we can use to interact with the environment is a sensor: we can sense a desired quantity from the environment and turn it into data. Are sensors an example of a ROS node, or ROS topic?
> 
> ✅ **Answer**: 
> <br/>
> An exapmle of a <u>**ROS Node**</u>, because a sensor processes information and then provides data. It does not transport it between nodes.

> ## 🎯 Question 1.2.
> Nodes and topics together form the backbone of any ROS application. It is important to know how they work together! How many nodes can publish to a single topic?
> 
> ✅ **Answer**: 
> <br/>
> **Any number** of nodes can publish, as long as the message has the **right type**. 

---

<br/>
<br/>


# 2 Build Your Own ROS Application
Now we have the knowledge about these ROS nodes and topics. The next step will be building your own nodes and topics.

There are four fundamental types of ROS nodes that can be used to build a ROS application:


- Publishers
- Subscribers
- Services
- Actions

First, we will focus on Publisher and Subscriber nodes.

## Publisher
A ROS node that generates information is called a publisher. A publisher sends information to nodes via topics. With robotics often these publishers are connected with sensors like cameras, encoders, etc.

If you use the `rosnode info` command you can see to which topics a node is connected, and if these are outbound or inbound connections.

``` python
# Node to publish a string topic.

import rospy
from std_msgs.msg import String

def simplePublisher():
    simple_publisher = rospy.Publisher('topic_1', String, queue_size = 10)
    rospy.init_node('node_1', anonymous = False)
    rate = rospy.Rate(1)
    
    # The string to be published on the topic.
    topic1_content = "my first ROS topic"
    
    while not rospy.is_shutdown():
    	simple_publisher.publish(topic1_content)
        rate.sleep()
        
if __name__== '__main__':
    try:
        simplePublisher()
    except rospy.ROSInterruptException:
        pass
```

## Subscriber
A ROS node that receives information is called a subscriber. It's subscribed to information in a topic and uses topic <u>**callback functions**</u>  to process the received information. With robotics, subscribers typically monitor system states such as triggering an alert when the robot reaches joint limits.


```python 
# Node to subscribe to a string and print the string on terminal.

import rospy
from std_msgs.msg import String

# Topic callback function.
def stringListenerCallback(data):
    rospy.loginfo(' The contents of topic1: %s', data.data)

def stringListener():
    rospy.init_node('node_2' , anonymous = False)
    
    rospy.Subscriber('topic_1' , String, stringListenerCallback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    stringListener()
```

## Publisher/Subscriber Example Codes
Now, run the scripts provided above in your terminal. After that, open a new terminal and have fun exploring your ROS nodes and topics! Note that when you run the subscriber node, you'll get to see the topic message in the log.  Use the `rqt_graph` command to see how `/node_1` and `/node_2` are connected through `/topic_1`.

<br />

<div align="center">
  <img src="./Figures/Fig-2-1-rqt-graph.png" alt="Pub/Sub Template RQT Graph" width="500"/>
</div>

<br />
<br />

> `🚨` **IMPORTANT**
> 
> REMEMBER to always launch the ROS Master using `roscore` executable. roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate.

Let's apply the knowledge gained from previous sections to verify if our nodes and topics have been successfully defined.

```bash
$ rosnode list
/node_1
/node_2
/rosout

$ rostopic list
/rosout
/rosout_agg
/topic_1
```

You can also peek at the connections and players linked to a topic using rostopic info. Check out that the message type of `/topic_1` is set as a standard message `std_msgs/String`. 

```bash
$ rostopic info /topic_1
Type: std_msgs/String

Publishers: 
 * /node_1 (http://riazati:32985/)

Subscribers: 
 * /node_2 (http://riazati:37091/)
```

Lastly, experiment with viewing the messages on the topic by employing `rostopic echo`. You can confirm that the output aligns with the `loginfo` message in the subscriber callback. Feel free to adjust various parameters in the code to observe the impacts, such as altering the `rate` in the publisher node to witness changes in the publishing frequency.

```bash
$ rostopic echo /topic_1
data: "Welcome to Hello (Real) World with ROS!!!"
---
data: "Welcome to Hello (Real) World with ROS!!!"
---
data: "Welcome to Hello (Real) World with ROS!!!"
---
data: "Welcome to Hello (Real) World with ROS!!!"
---
```

> `💡` **NOTE**
> 
> Do not forget the `/` before node and topic names! It is the ROS convetion to follow the namespace hierarchy of your package and node, using slashes `/` to separate the levels.

---

> ## 🎯 Question 2.1.
> Is the following statement True or False: A ROS Topic can be published without initializing a ROS Node.
> 
> ✅ **Answer**: 
> <br/>
> **False!** A ROS Topic can only be published from a ROS Node.

> ## 🎯 Question 2.2.
> Is the following statement True or False: A subscriber callback function is executed continuously, that is, it is processing all the time.
> 
> ✅ **Answer**: 
> <br/>
> **False!** It's not always processing, it will only process when new data is published.

---


# 3 ROS File System
ROS workspace (catkin workspace) consists of different subspaces. A workspace is a folder to organize ROS project files. ROS uses catkin, which is a build tool to compile source files into binary files. Your code goes into the `src` workspace folder and catkin manages the other ones. A catkin ROS workspace contains three main spaces:

- **src**: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 
                contains source code, this will be your main work folder.
- **devel**: &nbsp; 
                contains setup files for the project ROS environment.
- **build**: &nbsp;&nbsp; 
                contains the compiled binary files.

If you'd like to read some more about these spaces, you can do so [here](http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces). The following is the recommended and typical catkin workspace layout: 

```
  workspace_folder                  -- WORKSPACE
    ├─ src                          -- SOURCE SPACE
    │   ├── CMakeLists.txt          -- The 'toplevel' CMake file
    │   ├── package_1/
    │   │    ├── CMakeLists.txt
    │   │    ├── package.xml
    │   │    └── ...  
    │   └── package_n/
    │        ├── CATKIN_IGNORE      -- Optional empty file to exclude package_n from being
    │        ├── CMakeLists.txt
    │        ├── package.xml
    │        └── ...  
    ├─ build                        -- BUILD SPACE 
    │   └── CATKIN_IGNORE           -- Keeps catkin from walking this directory
    ├─ devel                        -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    │   ├── bin/
    │   ├── etc/
    │   ├── include/
    │   ├── lib/
    │   ├── share/
    │   ├── .catkin
    │   ├── env.bash
    │   ├── setup.bash
    │   ├── setup.sh
    │   └── ...
    ├─ install                      -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    │   ├── bin/
    │   ├── etc/
    │   ├── include/
    │   ├── lib/
    │   ├── share/
    │   ├── .catkin
    │   ├── env.bash
    │   ├── setup.bash
    │   ├── setup.sh
    │   └── ...
    └─ ... 
```

Now, Let's create and build a catkin workspace! First, create a new folder for your workspace:
```bash
$   mkdir -p ~/catkin_ws/src
```
Next, move to your newly created workspace using the command `cd catkin_ws`.
Thereafter, setup the correct ROS environment using the command `source /opt/ros/noetic/setup.bash`. Finally, we initialize catkin:

```bash
$   catkin init
$   catkin build
```

> `⚠️` **WARNING**
> 
> If you independently follow the tutorials on ROS Wiki, you might come across the command catkin_make. That is different to what we use in our course which is catkin. Don't try and mix the two! They are **NOT compatible**. You can read up on catkin [here](https://catkin-tools.readthedocs.io/en/latest/index.html).

ROS packages reside in the `src` space. In ROS, software is organised in ROS packages. A ROS package typically contains the following things:
- **CMakeList.txt**
- **package.xml** 
<br/> These two files indicate that the folder is a ROS package file. More on these two later in the course.
- **scripts/** 
<br/> This folder contains all Python scripts. We will only use Python in this course.
- **src/** 
<br/> This folder contains all C++ source files. We will not use these in this course.

<br/>

To create a new ROS package, we will use catkin:
```bash
$   cd <path_to_ros_ws>/src
$   catkin_create_pkg hrwros_week2 std_msgs
```

Please keep in mind that you don't need to place the source files of any dependencies, such as std_msgs in the line above. Instead, you can simply install their binaries with the rosdep command:

```bash
$   rosdep install <package_name>
```

You can also install all ROS package dependencies in one command:
```bash 
$ cd <path_to_ros_ws>/src
$ rosdep install --from-paths . --ignore-src -y
```

`src` is not the only space in your workspace: there is also the `devel` space. This contains all binary executables from your src spaces.

If you want more information on the folder structure of a catkin workspace, you can do so [here](http://wiki.ros.org/catkin/workspaces).

---

> ## 🎯 Question 3.1.
> We will provide you with four statements. Please select the following statements that are TRUE.
> <br/>
> **A.** &nbsp; A ROS workspace can be located anywhere in your home folder. 
> <br/>
> **B.** &nbsp; A ROS workspace which uses catkin as its build tool is called a catkin workspace. 
> <br/>
> **C.** &nbsp; Everything you need for your ROS application (drivers for sensors, cameras, etc) goes into your catkin workspace. 
> <br/>
> **D.** &nbsp; The command `mkdir -p new_ros_ws/src` creates a catkin workspace. 
> <br/>
> 
> ✅ **Answer**: 
> <br/>
> **(A) and (B) are correct!**
> A ROS workspace is just a folder on your Linux installation. The concept of a workspace is to only organize your ROS application development in one location. But, the workspace itself can reside anywhere in your home folder.
> When you use catkin as the build tool for your ROS workspace, it is called a catkin workspace. So, in essence a catkin workspace and ROS workspace are pretty much the same, just that catkin workspace is a bit explicit in saying what build tool you are using for the ROS workspace.

> ## 🎯 Question 3.2.
> It is not mandatory to create a folder inside the "src" space of your catkin workspace which in turn contains all your ROS packages. 
> 
> Is the above statement True of False?
>
> ✅ **Answer**: 
> <br/>
> **True!** Having a separate folder inside the src space is only for file organization sanity. It is not mandatory to create a "top-level" folder inside your catkin workspace that contains all your ROS packages. But, it is recommended practice.

---

</div>
