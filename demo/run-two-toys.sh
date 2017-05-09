#!/bin/bash

# add yellow toy target
rosservice call /add_target '[1, 0, 0]'
# add green toy target
rosservice call /add_target '[2, 1, 0]'

# give time for the plans to be created
sleep 2

# turn the executor on
rosservice call /set_executor_state 1
