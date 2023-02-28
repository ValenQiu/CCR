# Camera Calebration

Here is the steps to calebrate the camera：

   1) Print out the `checkerboard.png`. The checkerboard we use is `10X7`.
   2) Use the code `take_photos.py`, press `J` to take photo and press `Q` to quit. It is recommended to take at least 20 photos.
   3) Run the code `camera_calibration.py` to calevrate the camera. Make sure the specification fo the checkerboard is the same as the printed board in the code:
   ```python
   w = 9   # 10 - 1
   h = 6   # 7  - 1
   
   objp = objp*18.1  # 18.1 mm, size of each square 
   ```
   4) Save the output of the `camera_calibration.py`, and input them into the `camera_distortion.py` to check whether the result is good.
   ```python
dist = np.array([-0.4473624, 0.27176798, -0.00062173, -0.0019694, -0.09317617])
newcameramtx = np.array([[1.07438647e+03, 0.00000000e+00, 5.86387042e+02],
                        [0.00000000e+00, 1.06347595e+03, 3.29518190e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
mtx = np.array([[1.07588078e+03, 0.00000000e+00, 5.87202624e+02],
                [0.00000000e+00, 1.07555501e+03, 3.19733956e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
   ```
   Or
   ```python
   fx = 1.07588078e+03
    fy = 1.07555501e+03
    cx = 5.87202624e+02
    cy = 3.19733956e+02
    k1 = -0.4473624
    k2 = 0.27176798
    p1 = -0.00062173
    p2 = -0.0019694
    k3 = -0.09317617
    cameraMatrix = np.array([[fx, 0, cx],
                             [0, fy, cy],
                             [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2, k3])
   ```
