# Toy Jail Robot Demo
[![Build Status](https://travis-ci.org/bbux/clutter_butter.svg?branch=master)](https://travis-ci.org/bbux/clutter_butter)
[![Coverage Status](https://coveralls.io/repos/github/bbux/clutter_butter/badge.svg?branch=master)](https://coveralls.io/github/bbux/clutter_butter?branch=master)
---

## Overview

Final Project - UMD Software Development for Robotics - Spring 2017

Created using SIP ([Solo Iterative Processess](http://www.cs.wayne.edu/rajlich/SlidesSE/18%20example%20of%20sip.pdf))

[Backlog](https://docs.google.com/spreadsheets/d/1wChuRU8l6yA1EAUHQB64F89dBjVsw7t1enh5LzcWQo4/edit#gid=1120123239)

[Presentation](https://docs.google.com/a/terpmail.umd.edu/presentation/d/1-kTNRjpu_Ld7y-KJ0uFyO3MeERlu_CQIDL3VVJ_XrMY/edit?usp=sharing)

This project is for a robot that can be used to assist parents in one of the most frustrating and tedious tasks known to man, teaching their children to clean up after themselves.  The example use case is a messy room full of toys.  The goal of the robot would be to identify all of the toys and to move them to a cleanup jail.  The toys would then stay in jail for a predetermined interval before being rereleased into the wild.  The robot will help to reinforce in children the need to clean up after themselves and save parents the time and energy needed to instill good discipline themselves
. 
### Design
#### Nodes
MapWalker – Traverses the map identifying Targets that need to be moved to the jail. Sends Target Centroids to PushPlanner addTarget service.  Stays in run mode until whole map has been traversed and all Targets identified.  Then it goes into idle state releases control to the PushExecutor node.

PushPlanner – Initialized with coordinates of Jail.  Receives Targets centroids and calculates path plans to push them to the jail.  Offers services for adding Targets, updating Targets (used during pushing), listing known Targets, and getting path plans. Subscribes to /target_loc topic for updating Target locations

PushExecutor – Node for executing the push plans.  Stays in idle state until MapWalker node has finished identifying Targets.  Recieves push plans from the PushPlanner node.  Publishes updated Target location to /target_loc topic.  If push plan fails, requests a new or updated push plan.

OdomTracker – Wraps /odom Odometry topic in a service so that it can be requested during a blocking call

### Dependencies
 * ROS Indigo
 * navigation modules for nav messages
 * gazebo turtlebot pakcages
 * geometry modules for velocity publishing
 * google teset for unit tests
 
## License

Licensed under the [MIT License](https://opensource.org/licenses/MIT)
 
## Building

```
# into catkin workspace src/ folder
git clone https://github.com/bbux/clutter_butter.git

# from catkin workspace root
catkin_make 
```

## Demo
The main demo consists of one launch file and on shell script.  The launch file loads all of the nodes needed for the demo.  The shell script sends the necessary service calls to initialize the PushPlanner and start the Push Executor.  To run the demo:

```
# this will launch gazebo, and the three core nodes
roslaunch clutter_butter two_toy_world.launch.xml include_gazebo:=1

# in a separate terminal
demo/run-two-toys.sh 
```

## Tests

### Unit Tests

To run the unit tests:  

```
catkin_make tests && catkin_make run_tests_clutter_butter
```

The results should be a text visualization describing the test results.


## Doxygen Documentation

To generate the doxygen documentation from the root directory:

```
doxygen doxygen.config
```

This will create the documentation in the docs directory.

## TODO
 * Integrate with the move_base package for nav
 * Use /camera for obstacle detection
 * Add anti-tamper protections (tazer?)
 * Create the MapWalker module to walk around and find targets/toys to jail
 * Add hardware to pick up and throw toys into the jail
 * Add object identification, so pet dog is not mistaken for a toy and pushed into the jail
 * Add support for multi-agent clean up


