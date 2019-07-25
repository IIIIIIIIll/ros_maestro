# ros_maestro
Connect maestro board from pololu to ROS

## Environment
Besure to test your board using the Maestro control center
You need to install all the pre-reqs listed in the README of their control software. 
For the no longer available `winforms2.0` package, simply use the suggested replacement package by `apt`

## Testing
Tested with Linux Mint 18.03(Ubuntu 16.03), using ROS kinectic  

## Usage
After setting up ROS

`rosrun ros_maestro ros_maestro_node` this will start the maestro driver that listens to the `maestro_command` topic. It expects a string as input. The first two digits will be channel number(pad with 0 if necessary), the next four digit will be the speed/location for the motor/servo

A example publisher is also included

## Credit 
The maestro_driver is a simple ROS wrapper of the cross plaform C program listed on Pololu's document. Feel free to take a look at the original one(https://www.pololu.com/docs/0J40/5.h.1). 
