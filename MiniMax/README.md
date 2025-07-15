# Maxine Robot

## Architecture

### Sensor package

The `sensors` package (directory) contains code relating to the robot's sensors.
All sensors inherit from the abstract `RobotSensor` class, which has a `get_reading()` function that returns the current sensor reading.

Currently implemented sensors:
- `KeyboardSensor` : Senses the keyboard input from the user
- `DistanceSensor` : Collection of ultrasonic sensors in multiple directions
- `CameraSensor`: Connects to OAK-D camera and runs pipelines

If we ever change a sensor,  we just need to change the relevant sensor class, and the changes will work everywhere in the code

### Action_Manager package

The `action_managers` package (directory) contains code relating to the actions that the robot can take.
All Action managers inherit from the abstract `ActionManager` class, which has a `perform_action(config)` function that tells the manager to perform an action.

currently implemented managers:
-  `DebugActionManager`: this is used when you are running a raspberry pi that does not have certain servos, it will print to the command line the action being taken instead of actually performing an action
- `FacialAnimationManager`: this renders animations on the face of the robot.
- `SoundManager`: this plays sounds that are stored on disk
- `SpeechManager`: this allows the robot to speak using a Text to Speech engine running on a separate thread
- `VelocityManager`: this allows the robot to move 

If we ever change a way the robot takes actions (eg directly run voltages to the wheelchair instead of using the joystick), we just need to change the manager class, and the changes will work everywhere in the code

### Multithreading package

The `multithreading` package (directory) contains code reltaing to multi threading.
There is a `LoopingThread` abstract class that all threads inherit from that runs each thread's `tick()` function infinitely.

For threads to exit gracefully (not bug out the console when exiting / error thrown), every threda is listenting to the `THREAD_STOP_EVENT` variable found in the `__init__.py` of the multithreading directory. 

This `__init__` file also contains a function decorator (look this up if not sure what it is) called `graceful_thread_exit`  which will handle the gracefull exit of all threads when an error is thrown or the program exits by telling all threads to stop via the `THREAD_STOP_EVENT` and joining all threads.

### types package

the `types` package stores all Enums (look this up if you're not sure what it is) use throughout the code.


### Robot package

The `robot` package (directory) contains code relating to the robot.
The `Robot` class just stores all the sensors and action managers that all behaviors will acess to control the robot.

The `RobotFactory` class creates a robot based on a configuration file. It tailors the sensors and managers based on this configuration file. An example configuration file is found in the `configs` directory.

For each manager, if debug is set to true, it creates a `DebugActionManager` instead of the correct manager as discussed above.

### Behaviors package

This is the meat of the code base. The `behaviors` package (directory) contains all the Decision Tree code that controls the robot. 

The directory contains multiple Behaviors that are shared by all modes. A Behavior is a node in the tree that gets executed. This node performs a task and either returns SUCESS or FAILURE.
All behaviors inherit from the `MaxineBehavior` class, which just gives all behaviors access to the robot object via the blackboard feature of `py_trees` (A blackboard is just a fancy way of sharing variables between all nodes in the tree without having to pass through variables in each contructor).

The following sub directories exist:

- `conditions` This contains all the conditions that are used in the tree. A condition is a Behavior that returns SUCESS if a condition is met or FAILURE if a condition fails. It allows for the tree to chose which path it should follow when combining these conditions with Selector and Sequence nodes.

- `idle_mode` Contains Behaviors specific to idle mode. The `__init__.py` file of `idle_mode` contains code that builds the idle mode sub tree

- `keyboard_mode` Contains Behaviors specific to Keyboard mode. The `__init__.py` file of `keyboard_mode` contains code that builds the keyboard mode sub tree

- `chase_mode` Contains Behaviours specific to the Chase mode. The `__init__.py` file of `chase_mode` contains code that builds the chase mode sub tree

there is also a `utils` file that contains code that is shared within each mode to build specific types of nodes.

The current state of the behavior tree can be seen in `root.png`


### DepthAI package

The `depth_ai` package contains code relating to depth ai pipelines.
All pipelines inherit from the abstract `DepthAiPipeline`, which contains helper functions to create various depthai nodes.
Each pipeline must implement the `configure()` method, which will build its pipeline.
Each pipeline must also implement the `get_output_queues` method, which returns the different ouput queues that the pipeline outputs. This will be used by the camera sensor to get the latest reading for each queue.
Each pipeline should also impelement its own reading class that inherits from the base `CameraReading` class, which behaviours in the tree will use to figure out what is going on with the current reading.

## Running the robot


The robot is run by running the `new_main.py` file. This file performs the following actions:
- Build a `Robot` object using the `RobotFactory` from a specified config file.
- Builds the Robot's behavior tree
- Continuously runs the behavior tree.
