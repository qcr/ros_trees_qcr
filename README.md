# ROS Package for QCR-specific ROS Trees extensions

This package is where we define all of our QCR-specific extensions to the [ROS Trees package](https://github.com/qcr/ros_trees). This can include things like:

- leaves that are locked to specific action server namespaces and classes
- leaves tied to a specific robot configuration
- branches depending on QCR-specific leaves
- leaves or branches that shouldn't be made available to a wider audience
- extensions that don't make sense for all potential users of ROS Trees.

The general goal is for anything that can be generalised to live in the "common leaves" library of [ROS Trees]. This encourages higher-quality leaves, and improved usability on new robot platforms.

If you are looking to get started on using behaviour trees on robots, there are a number of resources available:

- The [README for ROS Trees](https://github.com/qcr/ros_trees) has a comprehensive summary of everything available in the ecosystem
- A [getting started tutorial](https://github.com/qcr/ros_trees/wiki/Getting-Started) goes through creating a bottle binning behaviour tree from scratch
- The process if problem solving with behaviour trees is described in [this guide](https://github.com/qcr/ros_trees/wiki/Solving-Problems-With-Trees), including TL;DR instructions
- There are also a [list of suggested resources](https://github.com/qcr/ros_trees/wiki/Solving-Problems-With-Trees#useful-links-and-information) at the end of the guide
