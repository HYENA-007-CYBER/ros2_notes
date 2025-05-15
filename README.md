#  Week 1 - Basics of ROS 2

##  Objective
We need to learn about the basics of ROS2
- Creating a **publisher node** that sends integers.
- Creating a **subscriber node** that receives those integers, calculates their square, and prints the result.

---

##  Requirements
- **ROS 2 Version**: [ROS2-humble(jammy jellyfish)]
- **Operating System**: Ubuntu 22.04
- **Programming Language**: Python
  

---

##  What I Learned

###  ROS 2 Architecture
- ROS 2 is built on the concept of **nodes**, which are independent processes that perform computation.
- Nodes communicate with each other using **topics** through a **publish/subscribe** model.

###  Publisher Node
- A publisher continuously sends messages over a specified topic.
- I learned how to create a timer-based publishing loop to simulate streaming data.

### Subscriber Node
- A subscriber listens to a topic and executes a callback function when a message is received.
- I practiced performing operations (like squaring a number) on incoming data and logging results.

###  Creating a ROS 2 Python Package
- Learned how to create a package using `ros2 pkg create`.
- Configured the package with `setup.py` and `package.xml` to define dependencies and entry points.

###  Building and Running Nodes
- Used `colcon build` to compile the workspace.
- Learned to source the workspace environment correctly before running nodes.
- Ran nodes using `ros2 run`, each in separate terminals to simulate real-time communication.

###  Debugging & Common Issues
- Faced issues like mismatched topic names, incorrect message types, and forgetting to source the setup file.
- Gained confidence in using ROS 2 logging for debugging and tracing behavior.

---

##  Outcome
By the end of this task, I successfully built a ROS 2 system where:
- One node published a stream of numbers.
- Another node subscribed to those numbers, computed their square, and logged the output.

This exercise helped me solidify the basics of ROS 2 communication and build confidence in working with publisher-subscriber models.

---




