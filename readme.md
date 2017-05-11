# Toy Jail Robot Demo
[![Build Status](https://travis-ci.org/bbux/clutter_butter.svg?branch=master)](https://travis-ci.org/bbux/clutter_butter)
[![Coverage Status](https://coveralls.io/repos/github/bbux/clutter_butter/badge.svg?branch=master)](https://coveralls.io/github/bbux/clutter_butter?branch=master)
---

## Overview

Final Project - UMD Software Development for Robotics - Spring 2017

Created using SIP ([Solo Iterative Processess](http://www.cs.wayne.edu/rajlich/SlidesSE/18%20example%20of%20sip.pdf))

[Backlog](https://docs.google.com/spreadsheets/d/1wChuRU8l6yA1EAUHQB64F89dBjVsw7t1enh5LzcWQo4/edit#gid=1120123239)

[Presentation](https://docs.google.com/a/terpmail.umd.edu/presentation/d/1-kTNRjpu_Ld7y-KJ0uFyO3MeERlu_CQIDL3VVJ_XrMY/edit?usp=sharing)

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


