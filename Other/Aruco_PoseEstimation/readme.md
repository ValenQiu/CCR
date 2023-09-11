# Relative Pose Estimated by arUco Marker

The code `relativePose.py` is the class for using arUco code to estimate the pose.

Using two markers, with different ID. 
The marker can be downloaded with the link: https://chev.me/arucogen/

When using the code, make sure the IDs of the base and the tracked marker is the same with the markers you use.
```    python
    # Base ID:
    base_id = 99
    # Track ID:
    track_id = 11
```

The dictionary used in the code is 6X6, 250, if want to change it, remember to change the dictionary in the code as well.
```python
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
```
The return of the code is the rotation vector and the transmit vector of the tracked marker relative to the base marker.
