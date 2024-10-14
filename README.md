# A playground for serial robotic manipulators

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains), that also gives a solution to the obstacles avoidance problem for static obstacles located in a planar workspace. It was developed for the needs of my diploma thesis during my electrical engineering studies.

The program uses the classic Denavit - Hartenberg parameterization for the robots kinematics. The modified Denavit - Hartenberg parameterization and the (more modern) Product of Exponentials (POE) method are not addressed here.

## Running instructions

To start the program, run the python file "robotic_manipulators_playground.py". The GUI can be run in both Windows and Linux. The Python libraries needed are written below:

Python libraries needed:
- To be installed (using pip for example):
    - roboticstoolbox
    - swift
    - matplotlib (matplotlib.pyplot, matplotlib.colors)
    - mpl_toolkits (mpl_toolkits.mplot3d)
    - cv2
    - pillow (Image, ImageColor)
    - numpy
    - scipy (scipy.ndimage)
    - spatialmath
    - spatialgeometry
    - shapely (shapely.geometry)
    - skimage
    - trimesh
    - pyserial
    - requests
- Native:
    - tkinter (tkinter.ttk, tkinter.simpledialog, tkinter.messagebox, tkinter.colorchooser)
    - itertools
    - string
    - copy
    - os
    - shutil
    - time
    - threading

Also, the library hntf2d is required. It can be found in the GitHub repository https://github.com/maxchaos/hntf2d, where the installation instructions are described. As it is referred there: "This software package provides an implementation of a harmonic transformation that maps any compact, multiply connected planar domain onto the unit disk; each of the holes that the original domain may have are collapsed onto distinct points inside its image.". It is a work related to the paper: ```P. Vlantis, C. Vrohidis, C. P. Bechlioulis and K. J. Kyriakopoulos, "Robot Navigation in Complex Workspaces Using Harmonic Maps," 2018 IEEE International Conference on Robotics and Automation (ICRA), 2018, pp. 1726-1731, doi: 10.1109/ICRA.2018.8460695```.

There is a problem with the swift library that may have been solved by now, or not. The issue has been identified in the robotics-toolbox-python repository on GitHub: https://github.com/petercorke/robotics-toolbox-python/issues/383. The problem arises when running the swift library on Windows. Specifically, there is a discrepancy in how file paths are handled between Linux and Windows systems. The root of the issue lies in how the self.path variable is processed within the SwiftRoute.py file. The current implementation attempts to adjust the path by retaining the initial / character, which works fine on Linux but leads to incorrect path formatting on Windows. To address the problem on Windows, a simple adjustment can be made in the SwiftRoute.py file of the swift library. Specifically, update the block of code by modifying self.path[9:] to self.path[10:].

So, for linux it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    # print(f"Retrieving file: {self.path[10:]}")
    self.path = urllib.parse.unquote(self.path[9:])
    self.send_file_via_real_path()
    return
```

And for windows it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    # print(f"Retrieving file: {self.path[10:]}")
    self.path = urllib.parse.unquote(self.path[10:])
    self.send_file_via_real_path()
    return
```

## Thor robotic arm

The Thor robotic arm is an open-source project, with the entire construction process and control code freely available (http://thor.angel-lm.com/). It was fully designed by Spanish robotics engineer Ángel Larrañaga Muro (https://www.linkedin.com/in/angellarranagamuro/) and has been continuously developed since 2016, both by the creator himself and through contributions from the global community. Its supporting structure (or body) consists of 3D-printable parts. Thor is an open kinematic chain (serial robotic manipulator) with 6 degrees of freedom (6 DOF). All of its joints are rotational, arranged in a yaw-pitch-pitch-yaw-pitch-yaw configuration (or yaw-roll-roll-yaw-roll-yaw, depending on the perspective of the x and y axes), starting from the base and extending to the end-effector.

![image](https://github.com/user-attachments/assets/dabc133e-9def-4f9e-9445-99ecb8fac507)


## Camera calibration

You may need to calibrate your own camera, if you want to use the related program features. In order to do so, you need to capture new images with your camera and put (only) them inside the "camera_calibration_images" folder. The images should contain a 10x7 chessboard pattern of arbitary dimensions, like the one shown below. You have to take multiple images of the chessboard (around 20 is probably fine), in different positions and orientations. After that, you can press the proper button in the GUI, in order to compute the new camera's intrinsic matrix and distortion coefficients.

![WIN_20240717_13_17_36_Pro](https://github.com/user-attachments/assets/d9bf27a9-6098-4213-95ef-22b623b1d0a7)
