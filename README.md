# QUT Tasks Package

The QUT Tasks package is where we define all of the tasks that our robots are capable of performing. Tasks are written in a way that they are robot agnostic from a software perspective, with specific robot hardware capabilities being the only factor limiting a robot from specific tasks (we haven't figured out a way to make an armless mobile base pick up a coffee... yet). 

A **task** is used to refer to something distinct we want the robot to do, often requiring the purposeful interweaving of many distinct components (which often come from many different internal and external authors). Bringing together software from different research areas, with different authors, and different conventions typically has a lot of pain associated with the process. To manage this, we break the problem of completing a task up into some isolated concepts and software components which are described in detail below. 

## What does it mean to define a solution to a task

There are 3 components that combine to form a solution to a given robot task:

- **capabilities**: these are what functionalities that are required from the robot in order to complete the task (e.g. moving the arm to a pose, driving to a pose, detecting objects, saying a string, etc.),
- **sub-behaviours**: define how a group of capabilities can be combined in a way that could be reused for a wide variety of potential tasks (examples: picking up an object, asking a question, going through a door, etc.), and
- **behaviours**: is the cumulative definition of how a robot can apply its available capabilities to complete a given task (e.g. voice-controlled manipulation demo on the tabletop panda arm, going and making a coffee for a user, emptying all of the garbage bins, etc.).

The distinction between a sub-behaviour and behaviour can often be confusing (and is somewhat arbitrary). It is helpful to remember that if the behaviour you are trying to create is too general to locked down to a single tree (i.e. can be parameterised and applies to a number of different tasks), and it requires the combination of too many distinct re-usable parts to be a single robot capability, then it should be a sub-behaviour.

To define how a robot can complete a task, we use **behaviour trees** which have been a common tool in the AI community for over a decade (originally being developed for the Halo games). Unsurprisingly given the name, behaviour trees define an entire agent behaviour using a tree structure consisting of nodes and edges. Nodes correspond to one of the agent's defined capabilities, and edges define a parent that controls when & how the node is called. 

Behaviour trees have a number of advantages over Finite State Machines (FSM), which are what is typically employed in robot systems. The main advantage applicable for our robots is behaviour trees allow us to detach behaviour (how bits of functionality combine together to do something meaningful) from functionality (things a robot can do). Detaching behaviour from functionality means that robot capabilities can be re-used across any task, and complex behaviours can be created without worrying about implementation details. For example, the functionality to detect whether a door is open or closed can be implemented once, then re-used in a plethora of behaviours (waiting for a door to be opened, waiting for a door to be closed, checking if a door is closed, only opening the door if it is closed, etc.) simply by changing the structure of the tree.

To implement a solution for a task with a behaviour tree, we map the 3 components of a solution from above to distinct structures in the behaviour tree framework:

- a **leaf** wraps a functionality of the robot, defining how the tree can execute the capability and receive the result once the functionality has been performed,
- a **branch** defines a re-usable sub-section of a tree that encapsulates a parameterisable behaviour which could be reused across multiple different tasks, and
- a **tree**, as mentioned above, is the definition of how a robot can interleave its capabilities to solve a task.

With all this said and done, solving a task boils down to 3 key steps which are tightly coupled require no particular order:

- Writing leaves for each of the required robot capabilities (this step requires you to break down the task into what capabilities will the robot require to complete the task)
- Defining re-usable branches to solve repeated / reusable sections of the task (sometimes your task may not require this, but it always helps to define branches if you think another task could use the branch later on)
- Writing a tree for the behaviour that solves the task (often when completing this step, you will end up having to go back and add parts to the previous two steps)

All this at this stage seems pretty abstract and "hand-wavey", but there is a full example below LINK of how the above principles can be applied to solve a non-trivial task.

## Quick & Easy: Solving a task with a tree

TODO 

## Conventions & Requirements

Before getting to the example, we declare here best practices, conventions, and requirements when creating trees in this package. As mentioned above, these task solutions require the collaboration of many many authors so it is important to stick to similar approaches as much as possible

TODO.... include an auto generated table of existing leaf definitions

## Example: Placing all visible bottles in the bin

For this example, consider a robot that needs to place all bottles that it can see from its default position into a nearby bin. To solve this task, the example below steps through the process of creating a behaviour tree from scratch. The formal description of the task we are trying to solve is:

    A tabletop manipulator needs to place all of the bottles in its visible workspace into the adjacent bin. The manipulator has the following ROS capabilities to available to assist in completing the task:

    - A ROS Service for getting a synced pair of RGB & depth images with the service name `/service/get_synced_rgbd`,
    - A ROS Service for detecting bottles which returns a list of detected bottles with the service name `/service/detect_bottles`,
    - A ROS Action Server for actuating the gripper running in the action namespace `/action/actuate_gripper`, and
    - ROS Action Servers for moving the gripper to a pose (available in `/action/move_gripper/pose`) and to named locations `workspace` & `bin` (available in `/action/move_gripper/location`).

Remember the ordering of how we approach the task is not important, but for this example we will start with the most accessible part: wrapping the robot capabilities defined above in leaves.

### Part 1: Declaring a robot capability (writing a leaf)

The good news is that qut_trees LINK has already implemented leaves for you; all you have to do is instantiate, override, and expand the existing leaves as needed to meet the needs of the capability you are trying to provide. Full documentation for base leaves available can be found in the package README LINK, but we will go through the relevant details here as we step through the example.

In most cases, all that's required to create a leaf for your capability is to simply provide a couple of arguments to an appropriate class constructor. For example, creating a leaf to actuate the gripper is as simple as:

```python
from qut_trees.leaves_ros import ActionLeaf

actuate_gripper_leaf = ActionLeaf("Actuate Gripper", action_namespace='/action/actuate_gripper')
```

All that's needed is a name for your leaf, and the namespace where the ROS Action Server exists! There are a lot of assumptions being made under the hood in the qut_trees package that: may not always meet your use case, so it is important to understand these assumptions for cases where you need to adjust them. 

The first assumption being made is around inputs and outputs. Leaves typically need some sort of input (e.g. a leaf moving a robot to a pose needs the goal pose as input), provide some sort of output (e.g. a leaf for checking if the robot's e-stop is on will output the e-stop state), or both (e.g. a leaf detecting grasp poses may take in RGB-D images as input & output data representing the found grasp poses). To control how a leaf handles input & output there are a number of constructor arguments defined in the `Leaf` class. A much too broad summary is that **load_\*** parameters control input, & **save\*** parameters control output. Full details of how these parameters work is again in qut_trees README.

Let's rollback the input assumption to address one of our use cases: opening & closing the gripper. To 'automagically' handle input & output the default behaviour of leaves is to try and form input data from whatever was last saved from a previous leaf, & save output data if the **save** argument is set. For opening the gripper we want leaves with static input data; i.e. a leaf for opening the gripper will always have the same input data commanding the ROS Action Server. To provide static input data instead of attempting to dynamically load the input at runtime, we use the **load_value** constructor argument:

```python
from qut_msgs.msg import ActuateGripperGoal
from qut_trees.leaves_ros import ActionLeaf

open_gripper_leaf = ActionLeaf("Actuate Gripper", action_namespace='/action/actuate_gripper', load_value=ActuateGripperGoal(mode=ActuateGripperGoal.MODE_STATE, state=ActuateGripperGoal.STATE_OPEN))
```

TODO stuff about msgs

Up to this point we have defined leaves as creating a single instance of a class from qut_trees. This works for leaves that will only be used once, but if we want to make leaves that can be used multiple times in a tree, re-used across different trees, expanded upon, override default leaf behaviour, provide custom functions, and lots more, implementing leaves as class definitions utilising inheritance makes a lot more sense. So, to create re-usable leaves for opening & closing the gripper we end up with the following:

```python
from qut_msgs.msg import ActuateGripperGoal
from qut_trees.leaves_ros import ActionLeaf


class _ActuateGripper(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(_ActuateGripper,
              self).__init__(action_namespace='/action/actuate_gripper',
                             *args,
                             **kwargs)


class OpenGripper(_ActuateGripper):
    OPEN_GOAL = ActuateGripperGoal(mode=ActuateGripperGoal.MODE_STATE,
                                   state=ActuateGripperGoal.STATE_OPEN)

    def __init__(self, *args, **kwargs):
        super(OpenGripper, self).__init__("Open Gripper",
                                          load_value=OpenGripper.OPEN_GOAL,
                                          *args,
                                          **kwargs)


class CloseGripper(_ActuateGripper):
    CLOSE_GOAL = ActuateGripperGoal(mode=ActuateGripperGoal.MODE_STATE,
                                    state=ActuateGripperGoal.STATE_CLOSE)

    def __init__(self, *args, **kwargs):
        super(CloseGripper, self).__init__("Close Gripper",
                                           load_value=CloseGripper.CLOSE_GOAL,
                                           *args,
                                           **kwargs)
```

Adding leaves for moving to poses & named locations is the same process as above:

```python
from qut_trees.leaves_ros import ActionLeaf

class MoveGripperToPose(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(MoveGripperToPose,
              self).__init__("Move gripper to pose",
                             action_namespace='/action/move_gripper/pose',
                             *args,
                             **kwargs)


class MoveGripperToLocation(ActionLeaf):

    def __init__(self, *args, **kwargs):
        super(MoveGripperToLocation,
              self).__init__("Move gripper to location",
                             action_namespace='/action/move_gripper/location',
                             *args,
                             **kwargs)
```

Now that we have created some re-usable leaves, next up let's add leaves to get bottle detections out of images from the robot hardware. To do this first call the ROS service for getting synced RGB-D images, then pass the result into the bottle detection service, and store all detections in a persistent location that we can later loop through to bin each bottle individually. Making a leaf for the synced RGB-D and bottle detection services is very similar to our leaves above, we just use a Service Leaf instead:

```python
from qut_trees.leaves_ros import ServiceLeaf

class GetSyncedRgbd(ServiceLeaf):

    def __init__(self, *args, **kwargs):
        super(GetSyncedRgbd,
              self).__init__("Get Synced RGBD",
                             service_name='/service/get_synced_rgbd',
                             save=True,
                             *args,
                             **kwargs)


class DetectBottles(ServiceLeaf):

    def __init__(self, *args, **kwargs):
        super(DetectBottles,
              self).__init__("Detect bottles",
                             service_name='/service/detect_bottles',
                             save=True,
                             *args,
                             **kwargs)

```

Enabling the **save** flag means that both leaves will save their results such that the next leave will be able to get the data simply by loading the "last result". For the case of this task, we don't want to save the list of detected bottles in last result, but instead save them to a key so they don't get overwritten when another leaf saves to "last result". We could hardcode the key name in the `DetectBottles()` class here, but then everyone who ever uses the detector leaf will have no control over where the data is saved. Instead, we will set the key name at the instance level in our task when we create our tree (i.e. call `DetectBottles(save_key=<MY_KEY_NAME>)` when creating the leaf instance for our tree).

Lastly, we need to make some basic leaves which don't need ROS to print a list of detected objects and pop (i.e. consume) an item from a list. We have already looked at the input and output parts of a leaf lifecycle, but for these tasks we have to look at the processing part of the lifecycle: defining how a leaf gets a result from the input by providing a `result_fn` to the constructor. For the `ActionLeaf` & `ServiceLeaf` classes we used above their `result_fn` is already written for us which calls the Action Server / Service with the leaf input data & return the response as the leaf output data. 

```python
import qut_trees.data_management as dm
from qut_trees.leaves import Leaf

class PrintObjects(Leaf):

    def __init__(self, *args, **kwargs):
        super(PrintObjects, self).__init__("Print Objects",
                                           result_fn=self._print_objects,
                                           *args,
                                           **kwargs)

    def _print_objects(self):
        if self.loaded_data is None or not self.loaded_data:
            print("The detector found no objects!")
        else:
            print(
                "The detector found %d objects at the following coordinates:" %
                len(self.loaded_data.objects))
            for o in self.loaded_data.objects:
                print(
                    "\tObject of pixel dimensions %dx%d @ top left coordinates: (%d,%d)"
                    % (o.width, o.height, o.left, o.top))

        return None


class PopFromList(Leaf):

    def __init__(self, pop_position=0, *args, **kwargs):
        super(PopFromList, self).__init__("Pop from list",
                                          result_fn=self._pop_item,
                                          *args,
                                          **kwargs)
        self.pop_position = pop_position

    def _pop_item(self):
        item = self.loaded_data.pop(self.pop_position)
        if self.load_key is not None:
            dm.set_value(self.load_key, self.loaded_data)
        else:
            dm.set_last_value(self, self.loaded_data)
        return item
```

The print leaf does BLAH, the other leaf does BLAH. And that's it, we now have all the leaves we need to complete this task. While this may have seemed a long process, remember that we have done this from scratch with no shared leaves available to help us in. In practice, most of the leaves you need will already be implemented and simply a `import qut_tasks.leaves.<leaf_type>` call away!

### Part 2: Using branches to create re-usable behaviours from leaves (writing sub-behaviours)




Step through the BinItem branch which will take an argument for a function to get the pose of the item

Could see how we could group the actions of getting synced RGB-D, detecting an object, and printing the objects into a branch as well that lets you choose which detector you would like to use.

### Part 3: Writing a behaviour tree for your task

Talk about putting it all into a tree. Link off to structure, control flow in a tree, etc. all work, then finish the explanation with...

PRETTY PICTURE OF TREE

Lead in to the python code (skip showing all of the leaf definitions for brevity...)

```python

my tree
```

## Other useful information

Links for general information about behaviour trees
