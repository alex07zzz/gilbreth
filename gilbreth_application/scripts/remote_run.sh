#!/bin/bash
echo "Start Demo"
echo "Moving Conveyor Belt"
rosservice call /gilbreth/conveyor/control "state:
  power: 15.0" 
echo "Start Spawning Objects"
rosservice call /start_spawn

