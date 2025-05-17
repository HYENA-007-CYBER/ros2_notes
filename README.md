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

###  ROS2 Framework
- ROS 2 is built on the concept of **nodes**, which are independent processes that perform computation
- Nodes communicate with each other using **topics** through a **publish/subscribe** model

### Creating a ROS2 workspace
Workspace is a folder that contains ROS  packages and allows us to build and run them together<br>
We need to create a **src/(source folder)** and **(build/ and install/ and log/) folders** that are auto generated during the build process
- We need to create our packages inside the src/ folder
- We need to use the **colcon build** command to build all the packages in our workspace
- we need to source the **install/setup.bash** file so that the terminal can recognize the packages and can run the nodes


### Creating a ROS2 Package

Packages are the basic unit for organizing software in ROS2. A package can contain nodes, libraries, configuration files, launch files, and more.  
Package sholud be created inside the **src/** folder of our ROS2 workspace.

- Use the `ros2 pkg create` command to create a new package inside the **src/** folder
- A package must have a unique name and follow naming conventions 
- We need to specify the build type (e.g., ament_python, ament_cmake) 
- We need to  define the  dependencies using the `--dependencies` flag

  #### For a ROS2 Python Package

A ROS2 Python package uses the `ament_python` build type and contains Python nodes and scripts.

### Key Points:
- Uses `setup.py` and `package.xml` for build and metadata
- Nodes are written in Python and placed in the main package directory

### Creating a ROS2 Package

A **ROS2 package** is the basic unit of organization in a ROS2 workspace. It contains everything needed for a module in a ROS system, such as:

- Node source files (Python or C++)
- Launch files
- Configuration files
- Dependencies
- Build and install instructions

All packages must be placed inside the `src/` folder of a ROS2 workspace. After creating packages, use `colcon build` to build the workspace and `source install/setup.bash` to use the packages in your terminal session.

---

#### Creating a Python Package in ROS2

A **Python package** in ROS2 uses the `ament_python` build type and contains nodes written in Python.

####  Steps:

1. **Navigate to our workspace's `src/` folder**  
    ```bash
    cd ~/ros2_ws/src/
    ```

2. **Create the Python package**  
    ```bash
    ros2 pkg create --build-type ament_python my_python_package --dependencies rclpy 
    ```

    This will:
    - Create a folder `my_python_package/`
    - Include `package.xml`, `setup.py`, and a `resource/` folder
    - Set up dependencies

3. **Add your Python node**
    Place your script in:  
    `my_python_package/my_python_package/my_node.py`

4. **Make your Python script executable**  
    ```bash
    chmod +x my_node.py
    ```

5. **Add entry points to `setup.py`**  
    Inside the `setup()` function, add:
    ```python
    entry_points={
        'console_scripts': [
            'my_node = my_python_package.my_node:main',
        ],
    },
    ```

6. **Build and source our workspace**  
    ```bash
    cd ~/ros2_ws/
    colcon build
    source install/setup.bash
    ```

---

#### Creating a C++ Package in ROS2

A **C++ package** in ROS2 uses the `ament_cmake` build type and contains nodes written in C++.

#####  Steps:

1. **Navigate to our workspace's `src/` folder**  
    ```bash
    cd ~/ros2_ws/src/
    ```

2. **Create the C++ package**  
    ```bash
    ros2 pkg create --build-type ament_cmake my_cpp_package --dependencies rclcpp 
    ```

    This will:
    - Create a folder `my_cpp_package/`
    - Include `package.xml`, `CMakeLists.txt`, and a `src/` folder
    - Set up dependencies

3. **Add your C++ node**
    Create your node file in:  
    `my_cpp_package/src/my_node.cpp`

4. **Edit `CMakeLists.txt` to build the node**
    Add the following:
    ```cmake
    add_executable(my_node src/my_node.cpp)
    ament_target_dependencies(my_node rclcpp std_msgs)

    install(TARGETS
      my_node
      DESTINATION lib/${PROJECT_NAME})
    ```

5. **Build and source your workspace**  
    ```bash
    cd ~/ros2_ws/
    colcon build
    source install/setup.bash
    ```

---




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




