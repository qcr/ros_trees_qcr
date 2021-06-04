# ROS Leaves Package

The ROS Leaves package is where we define all of the tasks our robots are capable of performing. Tasks are written in a robot agnostic way from a software perspective, with specific robot hardware capabilities being the only factor limiting a robot from specific tasks (we haven't yet figured how to make an armless mobile base pick up a coffee...).

A **task** refers to something distinct we want the robot to do, often requiring purposeful interweaving of many distinct components; components which often come from many different internal and external authors. Bringing software together from different research areas, with different authors, and different conventions is a process typically associated with a lot of pain. To manage this, we break completing a task up into some isolated concepts and software components which are described in detail below.

## What does it mean to define a solution to a task

Three components combine to form a solution for a robot task:

- **capabilities**: are what functionalities required from the robot to complete the task (e.g. moving the arm to a pose, driving to a pose, detecting objects, saying a string, etc.),
- **sub-behaviours**: define how a group of capabilities combine in a way that is reusable for a wide variety of tasks (examples: picking up an object, asking a question, going through a door, etc.), and
- **behaviours**: are the cumulative definition for how a robot can apply its available capabilities to complete a given task (e.g. voice-controlled manipulation demo on the tabletop panda arm, going and making a coffee for a user, emptying all of the garbage bins, etc.).

The distinction between sub-behaviour and behaviour can often be confusing, and is somewhat arbitrary. A good rule of thumb is if the behaviour you are creating is too general to be locked down to a single tree (i.e. can be parameterised and would apply to a number of different tasks), and it requires the combination of too many distinct re-usable parts to be a single robot capability, then it should be a sub-behaviour.

We use **behaviour trees**, a common tool in the AI community for over a decade (originally being developed for the Halo games), to define the behaviour required for a robot ti complete a task. Unsurprisingly given the name, behaviour trees define an entire agent behaviour using a tree structure consisting of nodes and edges. Nodes correspond to one of the agent's defined capabilities, and edges define a parent that controls when & how the node is called. More information on how behaviour trees work can be found in the resources section at the bottom of the page.

Behaviour trees have a number of advantages over Finite State Machines (FSM), which are what is typically employed in robot systems. The main advantage applicable for our robots is behaviour trees allow us to detach behaviour (how bits of functionality combine together to do something meaningful) from functionality (things a robot can do). Detaching behaviour from functionality means that robot capabilities can be re-used across any task, and complex behaviours can be created without worrying about implementation details, and developers can focus on one part at a time (i.e. giving a robot a useful capability, or making the robot solve a meaningful task). For example, the functionality to detect whether a door is open or closed can be implemented once, then re-used in a plethora of behaviours (waiting for a door to be opened, waiting for a door to be closed, checking if a door is closed, only opening the door if it is closed, etc.) simply by changing the structure of the tree.

We implement a solution for task with a behaviour tree by mapping the 3 components above to distinct parts of a tree:

- a **leaf** wraps a functionality of the robot, defining how the tree executes the capability and receives the result once complete,
- a **branch** is a re-usable tree sub-section encapsulating a parameterisable behaviour which could be reused across multiple different tasks, and
- a **tree**, as mentioned above, is the definition of how a robot can interleave its capabilities to solve a task.

Solving a task boils down to 3 tightly coupled steps which can be done in any order:

- Writing leaves for each of the required robot capabilities (this step requires you to break down the task into what capabilities the robot requires to complete the task)
- Defining re-usable branches to solve repeated / reusable sections of the task (sometimes your task may not require this, but it always helps to define branches if you think they would be useful for another task)
- Writing a tree for the behaviour that solves the task (often when completing this step, you will end up having to go back and add parts to the previous steps)

All this seems pretty abstract and "hand-wavey", but there is a full example below of how the above principles can be applied to solve a non-trivial task.

## Quick & Easy: Solving a task with a tree

Here's the 1,2,3 TL;DR for creating a tree to solve a task:

1. Create some leaves:

   ```python
   from ros_trees.leaves_ros import ActionLeaf, ServiceLeaf

   class DeriveSolution(ServiceLeaf):
       def __init__(self, *args, **kwargs):
           super(DeriveSolution, self).__init__(service_name="/derive_solution", *args, **kwargs)

   class ApplySolution(ActionLeaf):
       def __init__(self, *args, **kwargs):
           super(DeriveSolution, self).__init__(action_namespace="/apply_solution", *args, **kwargs)
   ```

2. Join the leaves up in a tree that solves a task:

   ```python
   from py_trees.composite import Sequence
   from ros_trees.trees import BehaviourTree

   my_tree = BehaviourTree("Solve Task", Sequence("Solve", [DeriveSolution(), ApplySolution()]))
   ```

3. Run the tree:

   ```python
   my_tree.run(hz=30)
   ```

Other helpful commands:

- Showing an interactive tree with `rqt`:

  ```
  rqt_py_trees
  ```

- Printing trees interactively in the terminal:

  ```
  rosrun py_trees_ros py-trees-tree-watcher
  ```

- Printing blackboard variable values interactively:

  ```
  rosrun py_trees_ros py-trees-blackboard-watcher
  ```

- Getting a static GraphViz graphic for your tree:

  ```python
  from ros_trees.trees import BehaviourTree
  # Create your tree in a variable called 'my_tree'
  my_tree.visualise()
  ```

## Conventions & best practices

Below are some conventions & best practices we encourage everyone to use. This package is meant to help everyone's robots, so doing things in a consistent & robust way will maximise the benefit for your colleagues:

- Use existing leaves and branches where possible (maximising reuse promotes greater consistency & robustness of our systems)
- A leaf should be as input-agnostic as possible (e.g. writing a leaf to only work with a 3x1 numpy array of Float64s when in reality it could work with any iterable of 3 numbers only hinders future use of your leaf)
- If you want your leaf or branch to be used by people, include it in the appropriate section of `ros_leaves.common_leaves.*` or `ros_leaves.common_branches.*`
- Extend functionality in your own class rather than messing with the base `Leaf` classes (including )

## Sharing is Caring: What ready-made leaves are available?

TODO.... include an auto generated table of existing leaf definitions

## Useful links & information

- [py_trees demos for understand tree behaviour](https://py-trees.readthedocs.io/en/release-0.6.x/demos.html)
- [py_trees_ros tutorials for behaviour trees on ROS - note: we use ros_trees instead of this package mostly](http://docs.ros.org/kinetic/api/py_trees_ros/html/tutorials.html)
- [a good generic behaviour trees tutorial](https://www.gamasutra.com/blogs/ChrisSimpson/20140717/221339/Behavior_trees_for_AI_How_they_work.php)
- [textbook on behaviour treees in robotics and AI with a thorough description of everything trees](https://www.semanticscholar.org/paper/Behavior-Trees-in-Robotics-and-AI%3A-An-Introduction-Colledanchise-%C3%96gren/9830c9d16293f8f87f998aa449143f0ed1554d1a)
