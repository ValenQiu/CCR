import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Define arc length
arc_length1 = 15  # Length of the first arc
arc_length2 = 15  # Length of the second arc

# Define the curvature and phi values
angles1 = np.linspace(0, np.pi/2, 3)
curvatures1 = angles1 / arc_length1
angles2 = np.linspace(0, np.pi/2, 3)
curvatures2 = angles2 / arc_length2
phis1 = np.linspace(-np.pi, np.pi, 7)
phis2 = np.linspace(-np.pi, np.pi, 4)

def rotation_matrix(axis, angle):
    """Create a rotation matrix for a given axis ('x', 'y', 'z') and an angle in radians."""
    if axis not in ['x', 'y', 'z']:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    
    rotation = R.from_euler(axis, angle, degrees=False)
    return rotation.as_matrix()

# Create the figure and 3D axis
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(1, 1, 1, projection='3d')

# Loop through each curvature for the first arc
for curvature1 in curvatures1:
    if curvature1 == 0:
        # If the curvature is 0, the arc is a straight line
        x1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        y1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        z1 = np.linspace(0, arc_length1, 100)
    else:
        # Calculate the start and end angles for the arc
        end_angle1 = arc_length1 * curvature1
        theta1 = np.linspace(0, end_angle1, 100)
        x1 = np.cos(theta1) / curvature1
        y1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        z1 = np.sin(theta1) / curvature1

    for phi1 in phis1:
        R_z_phi = rotation_matrix('z', phi1)

        # Rotate the arc using the 3D rotation matrix
        segment1_rotated = np.dot(R_z_phi, [x1, y1, z1])
        
        # Shift the arc to start at (0, 0)
        segment1_rotated[0] -= segment1_rotated[0][0]
        segment1_rotated[1] -= segment1_rotated[1][0]
        segment1_rotated[2] -= segment1_rotated[2][0]
        
        ax.plot(segment1_rotated[0], segment1_rotated[1], segment1_rotated[2], color='blue', linewidth=1, alpha=0.3)

# Repeat similar logic for the second arc if needed...

# Set limits and labels
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_zlim(0, 35)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Arcs Visualization')

plt.show()