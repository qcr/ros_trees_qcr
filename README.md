# QUT Tasks Package

The QUT Tasks package is where we generically define all of the tasks that our robots are capable of performing. Tasks are written in a way that they are robot agnostic from a software perspective, with specific robot hardware capabilities being the only factor limiting a robot from specific tasks (we haven't figured out a way to make an armless mobile base pick up a coffee... yet). 

A **task** is used to refer to something distinct we want the robot to do, often requiring the purposeful interweaving of many distinct components (which often come from many different internal and external authors). Bringing together software from different research areas, with different authors, and different conventions typically has a lot of pain associated with the process. To manage this, we break the problem of completing a task up into some isolated concepts and software compnents which are described in detail below. 

## Defining a solution to a task

There are 3 types of components that combine to form a solution to a given robot task:

- **capabilities**: these are what functionalities that are required from the robot in order to complete the task (examples: moving the arm to a pose, driving to a pose, detecting objects, saying a string, etc.),
- **sub-behaviours**: define how a group of capabilities can be combined in a way that could be reused for a wide variety of potential tasks (examples: picking up an object, asking a question, going through a door, etc.), and
- **behaviours**: is the cumulative definition of how a robot can apply its available capabilities to complete a given task (e.g. voice-controlled manipulation demo on the tabletop panda arm, goinng and making a coffee for a user, emptying all of the garbage bins, etc.).

The distinction between a sub-behaviour and behaviour can often be confusing (and is somewhat arbitrary). It is helpful to remember that if the behaviour you are trying to create is too general to locked down to a single tree (i.e. can be parameterised and applies to a number of different tasks), and it requires the combination of too many distinct re-usable parts to be a single robot capability, then it should be a sub-behaviour.

To define how a robot can complete a task, we use **behaviour trees** which have been a common tool in the AI community for over a decade (originally being developed for the Halo games). Unsurprisingly given the name, behaviour trees define an entire agent behaviour using a tree structure consisting of nodes corresponding to one of the agent's defined capabilities. Behaviour trees have a number of advantages over Finite State Machines (FSM) which are typically used in robot system. The main advantage applicable for our robots is behaviour trees allow us to detach behaviour (how bits of functionality combine together to do something meaningful) from functionality (things a robot can do). Detaching behaviour from functionality means that robot capabilities can be re-used across any task, and complex behaviours can be created without worrying about implementation details. For example, the functionality to detect whether a door is open or closed can be implemented once, then re-used in the a plethora of behaviours (waiting for a door to be opened, waiting for a door to be closed, checking if a door is closed, only openning the door if it is closed, etc.) simply be changing the structure of the tree.

To implement a solution for a task with a behaviour tree, we map the 3 components above to distinct structures in the behaviour tree framework:

- a **leaf** wraps a functionality of the robot, defining how the tree can execute the capability and receive the result once the functionality has been performed,
- a **branch** defines a re-usable sub-section of a tree that encapsulates a parameterisable behaviour which could be reused across multiple different tasks, and
- a **tree**, as mentioned above, is the definition of how a robot can interleave its capabilities to solve a task.

From this transition from a task that a robot needs to solve, to formalising the components of a solution, then mapping behaviour tree structures to solution components, defining a behaviour tree to solve a task simply becomes the application of 3 steps: writing leaves to declare robot capabilities, defining re-usable behaviours with branches, and writing trees to solve specific tasks. The following sections take the example task of placing all of the visible bottles in the bin, and goes through building a tree from scratch to solve the task (minus the robot...).

## Part 1: Declaring a robot capability (writing a leaf)

Write it

## Part 2: Using branches to create re-usable behaviours from leaves (writing sub-behaviours)

Create it

## Part 3: Writing a behaviour tree for your task

Write it

## Auto Generated Table of Declared Capabilities

Script it

## Other useful information

Links for general information about behaviour trees
