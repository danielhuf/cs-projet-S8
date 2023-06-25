## Kinect Usage Instructions

### Windows

To use Kinect on Windows, follow these steps:

1. Download the necessary components.
2. Import the Kinect and OpenCV packages while configuring the C++ environment. You can refer to the tutorial [here](https://www.instructables.com/Kinect-SDK-Hello-World/).
3. Use the `main.cpp` file located in the `kinect` folder.
4. After running the program, three windows will appear showing the color image, depth image, and registered color image.
5. Press 's' to automatically save the color image, depth image, and colored 3D point cloud. Press 't' to save only the color image. Press 'q' to exit the program. The files will be saved in the same folder.

### Linux

To use Kinect on Linux, follow these steps:

1. Configure the environment by following the tutorial [here](https://naman5.wordpress.com/2014/06/24/experimenting-with-kinect-using-opencv-python-and-open-kinect-libfreenect/).（If you use it on Ubuntu, please use version 20.04 or below.）
2. Calibrate the camera and modify the parameters in the `calibkinect.py` file within the `Kinect on Linux` folder, specifically the `uv_matrix()` and `xyz_matrix()` functions.

The files in the `Kinect on Linux` folder provide usage examples and an interface to obtain point clouds.

### Models

The `models` folder contains templates for point cloud matching and applications to generate templates. To create your own templates:

1. Use modeling software such as Solidworks to create the model.
2. Export the model as a `ply`, `obj`, or `stl` file.
3. Use the `pcl_mesh_sampling.exe` tool in PowerShell. Run `pcl_mesh_sampling.exe -h` to view the usage instructions.

### Point Cloud Processing

The following files provide various operations for point cloud files:

- `frame_change.py`
- `image_processing.py`
- `main.py`
- `pcd_processing.py`

Additionally, `exemple.py` provides an example of registering a cup. The `main()` function in `main.py` offers an example workflow that includes camera calibration, selecting a work plane, object recognition, and registration. And they are done on the toy building blocks. The `kinect` folder contains sample images and point cloud files, you can use them to test.

Please note that the above instructions are provided as a reference and may require adjustments based on your specific setup and requirements.
