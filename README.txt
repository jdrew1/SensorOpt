'main.py' loads or trains a ddpg network based on the hyperparameters in lines 13-45
'TensorflowInterface.py' contains all of the ddpg methods and is where network topology can be changed.
'CarlaWrapper.py' is an interfacing layer which wraps the CarlaInterface functions in a python gym Environment for use with the tensorflow network.
'CarlaInterface.py' contains all of the methods which direclty inteface with the carla simulator, as well as a few debug methods to create some of the visualizations seen in the report.
'debugVisualizer.py' opens an open3d viewport of a set of 3d points.

For convience during debugging, the python script is capable of opening and closing carla automatically via the 'macos' variable in 'main.py' on line 143