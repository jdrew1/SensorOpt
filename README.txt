--Readme.txt


====================About====================
=============================================
Program to Optimize Sensor Placement of Autonomous Driving Vehicles
Using Neural Networks and Generative Adversarial Networks.

This is a program created by Jordan Ambs as a Bachelor Thesis for the
University of Applied Sciences Ravensburg-Weingarten (a.k.a RWU)

====================Build====================
=============================================
STEP 1 *****IMPORTANT*****
----------
This program uses the CARLA open-source simulator, first build and install this software.
The simulator can be found at: https://carla.org


STEP 2
----------
Ensure the required dependencies exist in the "lib" folder
    See the "Dependencies" section for a list of all submodules
Make sure that the carla environment is accessible
    (Recommended: use the same conda environment used to build Carla Simulator)

STEP 4
----------
In the file "CMakeLists.txt", change the
    --set(Python3_ROOT_DIR "")--
    to
    --set(Python3_ROOT_DIR "/Users/Shared/anaconda/xxxxxxxx")--
to the path where python is installed on your machine.
If you are using an anaconda environment, make sure the carla PythonAPI is installed and the environment is active.

STEP 4
----------
Next, create a new subdirectory in which to build
    'mkdir build'

STEP 5
----------
Navigate to the build directory
    'cd build'

STEP 6
----------
Finally, build
    'cmake ..'
    'make'

====================Dependencies====================
====================================================
-Eigen3
-Python3.8 (recommended through conda)
-RapidJSON

--End of Readme.txt