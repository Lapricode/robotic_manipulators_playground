# robotic_manipulators_playground

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains), that also gives a solution to the obstacles avoidance problem.

The program uses the classic Denavit - Hartenberg parameterization. The modified Denavit - Hartenberg parameterization and the (more modern) Product of Exponentials (POE) method are not addressed here.

The program can be run in both Windows and Linux.

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

Also, the library hntf2d is required.

## Camera calibration

You may need to calibrate your own camera, if you want to use the related program features. In order to do so, you need to capture new images with your camera and put (only) them inside the "camera_calibration_images" folder. The images should contain a 10x7 chessboard pattern of arbitary dimensions, like the one shown below. You have to take multiple images of the chessboard (around 20 is probably fine), in different positions and orientations. After that, you can press the proper button in the GUI, in order to compute the new intrinsic matrix and distortion coefficients.

![WIN_20240717_13_17_36_Pro](https://github.com/user-attachments/assets/d9bf27a9-6098-4213-95ef-22b623b1d0a7)

## Swift simulation

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
