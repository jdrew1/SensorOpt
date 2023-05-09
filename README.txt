Deep Deterministic Policy Gradient solution to the problem of optimally placing LiDAR sensor(s) on a vehicle. 

About:
============================================================================

'main.py' creates and trains a ddpg network based on the hyperparameters in lines 13-45
'TensorflowInterface.py' contains all of the ddpg methods and is where network topology can be changed.
'CarlaInterface.py' contains all of the methods which directly inteface with the CARLA simulator, as well as a few debug methods to create some of the visualizations seen in the report.
'CarlaWrapper.py' is a layer which wraps the CarlaInterface functions in a python Gym environment for use with the tensorflow network.
'debugVisualizer.py' opens an open3d viewport of a set of 3d points. The open3d window can be closed by pressing the spacebar.

For convience during debugging, the python script is capable of opening and closing carla automatically via the 'macos' variable in 'main.py' on line 143

To reduce computational overhead during training, the CARLA simulator can be set to no-render mode via the 'no_render' in the Carla wrapped environment ('main.py' on line 18)


Dependencies:
============================================================================
- Carla Autonomous Driving Simulator : http://carla.org
- Tensorflow : https://www.tensorflow.org

The python code also has the following imports:
carla
gym
math
matplotlib
numpy
open3d
os
random
tensorflow
time

Acknowledgements:
============================================================================
This algorithm and accompanying paper were written as a Bachelor's Thesis for the Hochschule Ravensburg-Weingarten in Baden-Württemberg, Germany.
Special thanks to Mr. Felix Berens and Dr. Stefan Elser for their help and guidance in completing this project.

Note: the algorithm is still a prototype. In its current iteration, the network is sensitive to the exact hyperparameters used during training.
Iteration and collaboration is, however, greatly encouraged as long as proper credit is given! 
Please fork from this repository, as I am eager to see any future developments.

For all legal purposes, the software is provided under the MIT license, as follows:
============================================================================

Copyright 2023 Jordan Ambs

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
