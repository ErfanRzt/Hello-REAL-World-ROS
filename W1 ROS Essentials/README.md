<div align="justify">

# ROS Essentials
Welcome to the very first lesson of this ROS MOOC! Before we dive into ROS, it is helpful to know something about how ROS works internally.

## Nodes
A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. Nodes provide modularity to robotic projects that use ROS. They are often written in C++ or Python. In this course, we will use Python to write them. ROS Noetic compatible with Python 3.x.

## Topics
Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which decouples the production of information from its consumption. In general, nodes are not aware of who they are communicating with. Instead, nodes that are interested in data subscribe to the relevant topic; nodes that generate data publish to the relevant topic. There can be multiple publishers and subscribers to a topic. 

> `💡` **NOTE**
> 
> Topics are intended for unidirectional, streaming communication. Nodes that need to perform remote procedure calls, i.e. receive a response to a request, should use services instead. There is also the Parameter Server for maintaining small amounts of state. 

---

In a real robot application you will often have to deal with a large number of nodes and topics. It is important to know which nodes are talking to each other, and what topics are being used to pass the information (messages) between nodes.

> ## 🎯 Question 1
> In robotics, we often have to interact with the real-world. One type of device we can use to interact with the environment is a sensor: we can sense a desired quantity from the environment and turn it into data. Are sensors an example of a ROS node, or ROS topic?
> 
> ✅ **Answer**: 
> <br/>
> An exapmle of a <u>**ROS Node**</u>, because a sensor processes information and then provides data. It does not transport it between nodes.


</div>