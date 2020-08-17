# omni_proc_ros

<a href="LICENSE" ><img src="https://img.shields.io/github/license/1487quantum/omni_proc_ros?style=for-the-badge"/></a>

Omnidirectional camera processing ROS package via image transport.

![](assets/mask_rqt.png)

## Installation

Git clone the repository into your workspace (e.g. catkin_ws).
```bash
$ cd catkin_ws/src
$ git clone https://github.com/1487quantum/omni_proc_ros.git
```

After that, recompile the workspace.
```bash
$ catkin_make
```

## Launch files
Launch the omni_proc_ros node via *omni_mono.launch*.
```bash
$ roslaunch omni_mono_proc omni_mono.launch
```

## Calibration

> **Note:** The calibration executables have to be compiled seperately, and would be available in the *build* directory after compilation.

### Compilation

Update the `compile_calib.sh` permission & run the script to compile the calibration executable files: *omni_calib* & *omni_calib_stereo*.

```bash
$ chmod +x compile_calib.sh
$ ./compile_calib.sh
```

To run the executable:
```bash
$ cd build
$ ./omni_calib [Parameters]
```

More details are availble below.

### omni_calib [Mono]

Performs camera calibration with the provided imagelist file, which uses the `xml` format. Ensure that the full image path is entered instead of the relatie path.

```bash
$ ./omni_calib [IMG_LIST]  [CHECKBOARD_HORIZONTAL_POINTS]   [CHECKBOARD_VERTICAL_POINTS]  [SQUARE_WIDTH (mm)]
```
- **IMG_LIST**: List of images to be used for calibration. (A sample could be found in the `sample` directory.)
- **CHECKBOARD_HORIZONTAL_POINTS**: Number of horizontal points on checker, count by edges of square. 
- **CHECKBOARD_VERTICAL_POINTS**: Number of vertical points on checker, count by edges of square. 
- **SQUARE_WIDTH**: Size of checkerboard square, measured in millimetres (mm).

> **Note:** Ensure that the both checkerboard horizontal & vertical points are more than 2, else the calibration wouldn't work!

### omni_calib_stereo

Performs camera calibration with the provided imagelist file, which uses the `xml` format. Ensure that the full image path is entered instead of the relatie path.

```bash
$ ./omni_calib_stereo [IMG_LIST_LEFT]  [IMG_LIST_RIGHT]  [CHECKBOARD_HORIZONTAL_POINTS]   [CHECKBOARD_VERTICAL_POINTS]  [SQUARE_WIDTH (mm)]
```
- **IMG_LIST_LEFT**: List of *left* images to be used for calibration. (A sample could be found in the `sample` directory.)
- **IMG_LIST_RIGHT**: List of *right* images to be used for calibration. (A sample could be found in the `sample` directory.)
- **CHECKBOARD_HORIZONTAL_POINTS**: Number of horizontal points on checker, count by edges of square. 
- **CHECKBOARD_VERTICAL_POINTS**: Number of vertical points on checker, count by edges of square. 
- **SQUARE_WIDTH**: Size of checkerboard square, measured in millimetres (mm).

> **Note:** Ensure that the both checkerboard horizontal & vertical points are more than 2, else the calibration wouldn't work!

