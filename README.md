# French Butler Robot

## Project Overview

The Butler Robot project is designed to automate food delivery tasks within a restaurant environment. This autonomous robot efficiently navigates between the kitchen, dining tables, and its designated home position. It is equipped to handle various scenarios, such as order confirmations, multiple deliveries, and cancellations, ensuring a seamless and efficient operation.

## Key Features

- **Autonomous Navigation:** Efficient movement between predefined locations including the kitchen, tables, and the home position.
- **Order Confirmation Mechanism:** Robust handling of order confirmations with built-in timeout protocols.
- **Cancellation Management:** Effective handling of order cancellations during different phases of delivery.
- **Multi-Order Handling:** Optimized execution of multiple delivery tasks in a single trip.
- **Error and Timeout Handling:** Safeguards against task failures and idle conditions.

## System Requirements

- **Operating System:** Ubuntu with ROS Noetic
- **Programming Language:** Python 3

## Setup Instructions

1. **Install ROS Noetic:** Ensure that ROS Noetic is properly installed. Follow the detailed installation guide for your version of Ubuntu [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. **Create a ROS Workspace:**
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

3. **Copy Project Files:** Place the Butler Robot project files in the `src` directory of your workspace.
   ```sh
   cd ~/catkin_ws/src
   unzip /path/to/butler_robot.zip
   ```

4. **Build the Workspace:**
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

5. **Start the ROS Master:**
   ```sh
   roscore
   ```

6. **Source the Workspace:**
   ```sh
   source devel/setup.bash
   ```

7. **Launch the Butler Robot:** Run the launch file to start the system.
   ```sh
   roslaunch butler_robot butler_robot.launch
   ```

## Configuration Instructions

1. **Define Location Coordinates:** Configure the coordinates for the home position, kitchen, and tables in a YAML configuration file (e.g., `locations.yaml`).
   ```yaml
   home: [0.0, 0.0]
   kitchen: [5.0, 5.0]
   table_1: [10.0, 0.0]
   table_2: [10.0, 5.0]
   table_3: [10.0, 10.0]
   ```

## Technical Documentation

### ROS Nodes

The system architecture consists of multiple ROS nodes, each responsible for a specific functionality:

1. **robot_butler_node:** Manages the overall control flow, including task allocation, movement delegation, and coordination of confirmations and cancellations.
2. **robot_movement:** Handles autonomous navigation to designated locations using the ROS navigation stack.
3. **order_handler:** Manages the queue and processing of single and multiple orders efficiently.
4. **confirmation_handler:** Oversees the confirmation process at the kitchen and tables, implementing timeout protocols.
5. **cancellation_handler:** Coordinates the cancellation logic to ensure safe and appropriate actions during task interruptions.
6. **utils:** Provides utility functions for logging, configuration handling, and other shared operations.

### Project Directory Structure

```sh
butler_robot/
├── launch/
│   └── robot_butler.launch
├── src/
│   ├── robot_butler_node.py
│   ├── robot_movement.py
│   ├── order_handler.py
│   ├── confirmation_handler.py
│   ├── cancellation_handler.py
│   └── utils.py
├── CMakeLists.txt
├── LICENSE
└── package.xml
```

### Key Source Files

- **robot_butler_node.py:** Main control script for managing the robot's tasks.
- **robot_movement.py:** Contains functions for autonomous navigation.
- **order_handler.py:** Manages order queue and delivery operations.
- **confirmation_handler.py:** Handles confirmation processes with timeout mechanisms.
- **cancellation_handler.py:** Manages task cancellations efficiently.
- **utils.py:** Provides reusable functions for logging and configuration.

### Build and Launch Files

- **CMakeLists.txt:** Defines the build configuration for the ROS package.
- **package.xml:** Specifies the package dependencies and metadata.
- **robot_butler.launch:** Launches all necessary nodes and sets essential parameters.

## Functionality and Scenarios

The Butler Robot is designed to handle a variety of scenarios:

1. **Simple Delivery:** Navigate from home to the kitchen, then to a table, and return to the home position.
2. **Order Confirmation:** Wait for confirmation at the kitchen or table before proceeding to the next task.
3. **Timeout Handling:** Return to the home position if no confirmation is received within the specified time limit.
4. **Cancellation Handling:** Ensure the robot safely returns to the kitchen and then home if a task is canceled.
5. **Multi-Order Management:** Deliver orders to multiple tables in a single trip for efficient operation.
6. **Conditional Delivery:** Skip delivery to certain tables based on confirmation or cancellation conditions.

## Testing and Validation

**Unit Testing:** Validate individual functions for accuracy and robustness.

**Integration Testing:** Ensure seamless operation by testing the complete workflow in a simulated environment.

## Additional Notes

- Ensure all software dependencies are installed and configured correctly.
- Modify the source code as necessary to accommodate specific hardware configurations or project requirements.
- For hardware integration, consider additional configurations for motor control and sensor calibration.

This documentation serves as a comprehensive guide for developers and stakeholders to understand and deploy the Butler Robot system efficiently.


