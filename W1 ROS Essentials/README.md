<div align="justify">

# ROS Essentials
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

> ## 🎯 Question 1
> In robotics, we often have to interact with the real-world. One type of device we can use to interact with the environment is a sensor: we can sense a desired quantity from the environment and turn it into data. Are sensors an example of a ROS node, or ROS topic?
> 
> ✅ **Answer**: 
> <br/>
> An exapmle of a <u>**ROS Node**</u>, because a sensor processes information and then provides data. It does not transport it between nodes.

> ## 🎯 Question 2
> Nodes and topics together form the backbone of any ROS application. It is important to know how they work together! How many nodes can publish to a single topic?
> 
> ✅ **Answer**: 
> <br/>
> **Any number** of nodes can publish, as long as the message has the **right type**. 

---

</div>
