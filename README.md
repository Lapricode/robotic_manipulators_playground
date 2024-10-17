# A playground for serial robotic manipulators

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains). It's a light edition of a bigger project located in the repository https://github.com/Lapricode/robotic_manipulators_playground. The inspiration for this work came from the open source project "Thor (3D Printable 6DOF Robotic Arm)", that you may explore going to the link http://thor.angel-lm.com. As such, the GUI was first built to support mainly the Thor robotic arm, but it evolved to a more generic program for the simulation of any serial robotic arm up to 12 DOFs. It is supported on both Windows and Linux systems.
Thor robotic arm is an open-source project, with the entire construction process and control code freely available (http://thor.angel-lm.com). Its supporting structure (or body) consists of 3D-printable parts. It is a serial robotic manipulator with 6 degrees of freedom (6 DOF). All of its joints are rotational, arranged in a yaw-pitch-pitch-yaw-pitch-yaw configuration (or yaw-roll-roll-yaw-roll-yaw, depending on the perspective of the x and y axes), starting from the base and extending to the end-effector. In the picture below you can see the fully constructed Thor robotic arm, its joints frames used for the kinematics analysis and the corresponding classic DH parameters.

![image](https://github.com/user-attachments/assets/f9536b42-0e1a-4463-8bb5-6335701df5cf)

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
    self.path = urllib.parse.unquote(self.path[9:])
    self.send_file_via_real_path()
    return
```

And for windows it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    self.path = urllib.parse.unquote(self.path[10:])
    self.send_file_via_real_path()
    return
```

## Camera calibration

You may need to calibrate your own camera, if you want to use the related program features. In order to do so, you need to capture new images with your camera and put (only) them inside the "camera_calibration_images" folder. The images should contain a 10x7 chessboard pattern of arbitary dimensions, like the one shown below. You have to take multiple images of the chessboard (around 20 is probably fine), in different positions and orientations. After that, you can press the proper button in the GUI, in order to compute the new camera's intrinsic matrix and distortion coefficients.

![WIN_20240717_13_17_36_Pro](https://github.com/user-attachments/assets/d9bf27a9-6098-4213-95ef-22b623b1d0a7)

## GUI features

### Visualizing the robotic manipulators and their environment - canvas area

![image](https://github.com/user-attachments/assets/c0b0a2a0-c37f-4db5-876b-8c223946f196)

### Designing and simulating the robotic manipulators - 1st menu

![image](https://github.com/user-attachments/assets/9b0d6aa5-9686-4df7-af13-19c497f77515)

### Analysing the kinematics of the robotic manipulators - 2nd and 3rd menus

The program uses the classic Denavit - Hartenberg parameterization

![image](https://github.com/user-attachments/assets/f87f2005-1aa6-4041-8070-1a22ca7fc524)

### Controlling the robotic manipulators - 4th menu

In the picture below, there is the original Asgard GUI from the "Thor" open source project (the left side, it can be found here: http://thor.angel-lm.com/documentation/control-software) and my suggestion (the right side). The menu is heavily inspired from the Asgard structure.

![image](https://github.com/user-attachments/assets/4da05154-e8e3-43e2-ae6d-dde2ab3aeadb)

### Capturing frames using the camera - 5th menu


### Creating the workspace obstacles - 6th menu


### Solving the obstacles avoidance problem - 7th menu

