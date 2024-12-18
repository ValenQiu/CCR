import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter, FFMpegWriter

# Define the curvature and phi values
# curvature1 = np.linspace(0.03, -0.03, 100)
# curvature2 = np.linspace(0.03, -0.03, 100)
# phi1 = np.zeros_like(curvature1)
# phi2 = np.zeros_like(curvature2)
curvature1 = np.linspace(0.03, 0.03, 100)
curvature2 = np.linspace(0.03, 0.03, 100)
phi1 = np.linspace(0, 2 * np.pi, 100)
phi2 = np.linspace(0, 2 * np.pi, 100)

# Define arc length
arc_length1 = 15  # Length of the first arc
arc_length2 = 15  # Length of the second arc

# Animation update function for updating the backbone position
def cal_tdcr(arc_length1, curvature1, phi1, arc_length2, curvature2, phi2):
    if curvature1 == 0:
        # If the curvature is 0, the arc is a straight line
        start_angle1 = 0
        end_angle1 = start_angle1
        # If the curvature is 0, the arc is a straight line
        x1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        y1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        z1 = np.linspace(0, arc_length1, 100)
    else:
        # Otherwise, calculate the start and end angles
        start_angle1 = 0
        end_angle1 = arc_length1 * curvature1
        theta1 = np.linspace(start_angle1, end_angle1, 100)
        x1 = np.cos(theta1) / curvature1
        y1 = np.zeros_like(np.linspace(0, arc_length1, 100))
        z1 = np.sin(theta1) / curvature1

    # Shift the arc to start at (0, 0)
    x1 = x1 - x1[0]
    y1 = y1 - y1[0]
    z1 = z1 - z1[0]

    # print('first shift of the first segment: ', [x1[0], y1[0], z1[0]])

    R1 = np.array([[np.cos(phi1), -np.sin(phi1), 0],
                [np.sin(phi1),  np.cos(phi1), 0],
                [0,            0,            1]])

    # Rotate the arc using the 3D rotation matrix
    segment1_rotated = np.dot(R1, [x1, y1, z1])
    x1 = segment1_rotated[0]
    y1 = segment1_rotated[1]
    z1 = segment1_rotated[2]

    # print("beginning of the first segment: ", [x1[0], y1[0], z1[0]])
    # print("end of the first segment: ", [x1[-1], y1[-1], z1[-1]])

    if curvature2 == 0:
        # If the curvature is 0, the arc is a straight line
        start_angle2 = end_angle1
        end_angle2 = start_angle2
        # If the curvature is 0, the arc is a straight line
        x2 = np.zeros_like(np.linspace(0, arc_length2, 100))
        y2 = np.zeros_like(np.linspace(0, arc_length2, 100))
        z2 = np.linspace(0, arc_length2, 100)
        # Rotate the second segment by 30 degrees along the y-axis
        angle = start_angle2
        rotation_matrix = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
        rotated_coords = np.dot(rotation_matrix, [x2, y2, z2])
        x2 = rotated_coords[0]
        y2 = rotated_coords[1]
        z2 = rotated_coords[2]
    else:
        # Otherwise, calculate the start and end angles
        start_angle2 = end_angle1
        end_angle2 = start_angle2 + arc_length2 * curvature2
        theta2 = np.linspace(start_angle2, end_angle2, 100)
        x2 = np.cos(theta2) / curvature2
        y2 = np.zeros_like(np.linspace(0, arc_length2, 100))
        z2 = np.sin(theta2) / curvature2

    # Shift the arc to start at (0, 0)
    x2 = x2 - x2[0]
    y2 = y2 - y2[0]
    z2 = z2 - z2[0]

    # print("beginning of the second segment: ", [x2[0], y2[0], z2[0]])
    # Define the 3D rotation matrix for rotation around the z-axis
    Rz = np.array([[np.cos(phi2), -np.sin(phi2), 0],
                [np.sin(phi2),  np.cos(phi2), 0],
                [0,            0,            1]])

    # Rotate the arc using the 3D rotation matrix
    segment2_rotated = np.dot(Rz, [x2, y2, z2])
    x2 = segment2_rotated[0]
    y2 = segment2_rotated[1]
    z2 = segment2_rotated[2]

    # Shift the arc to the end of the first segment
    x2 = x2 + x1[-1]
    y2 = y2 + y1[-1]
    z2 = z2 + z1[-1]

    # print('x2: ',x2)
    # print('y2: ', y2)
    # print('z2: ', z2)

    segment1_end = [x1[-1], y1[-1], z1[-1]]
    segment2_end = [x2[-1], y2[-1], z2[-1]]

    return x1, y1, z1, x2, y2, z2, segment1_end, segment2_end

# Create the figure and subplots
fig = plt.figure(figsize=(16, 9))

# 3D subplot
ax3d = fig.add_subplot(221, projection='3d')
line1, = ax3d.plot([], [], [], color='blue', linewidth=2)
line2, = ax3d.plot([], [], [], color='green', linewidth=2)
path1, = ax3d.plot([], [], [], color='red', alpha=0.5, linewidth=2)
path2, = ax3d.plot([], [], [], color='red', alpha=0.5, linewidth=2)

# Set limits and labels for the 3D plot
ax3d.set_xlim(-15, 15)
ax3d.set_ylim(-15, 15)
ax3d.set_zlim(0, 35)
ax3d.set_xlabel('X')
ax3d.set_ylabel('Y')
ax3d.set_zlabel('Z')
ax3d.set_title('3D Arc Animation')

# Set aspect ratio for 3D plot
ax3d.set_box_aspect([1, 1, 1])  # Aspect ratio is 1:1:1

# 2D projection subplots
ax_xy = fig.add_subplot(222)
line1_xy, = ax_xy.plot([], [], color='blue', linewidth=2)
line2_xy, = ax_xy.plot([], [], color='green', linewidth=2)
ax_xz = fig.add_subplot(223)
line1_xz, = ax_xz.plot([], [], color='blue', linewidth=2)
line2_xz, = ax_xz.plot([], [], color='green', linewidth=2)
ax_yz = fig.add_subplot(224)
line1_yz, = ax_yz.plot([], [], color='blue', linewidth=2)
line2_yz, = ax_yz.plot([], [], color='green', linewidth=2)

# Set limits and labels for the 2D plots
ax_xy.set_xlim(-15, 15)
ax_xy.set_ylim(-15, 15)
ax_xy.set_xlabel('X')
ax_xy.set_ylabel('Y')
ax_xy.set_title('X-Y Projection')
ax_xy.set_aspect('equal', adjustable='box')  # Set equal aspect ratio

ax_xz.set_xlim(-15, 15)
ax_xz.set_ylim(0, 35)
ax_xz.set_xlabel('X')
ax_xz.set_ylabel('Z')
ax_xz.set_title('X-Z Projection')
ax_xz.set_aspect('equal', adjustable='box')  # Set equal aspect ratio

ax_yz.set_xlim(-15, 15)
ax_yz.set_ylim(0, 35)
ax_yz.set_xlabel('Y')
ax_yz.set_ylabel('Z')
ax_yz.set_title('Y-Z Projection')
ax_yz.set_aspect('equal', adjustable='box')  # Set equal aspect ratio

# Initialize function for animation
def init():
    curvature_init = 0
    phi_init = 0
    x1, y1, z1, x2, y2, z2, end1, end2 = cal_tdcr(arc_length1, curvature_init, phi_init, arc_length2, curvature_init, phi_init)
    line1.set_data(x1, y1)
    line1.set_3d_properties(z1)
    line2.set_data(x2, y2)
    line2.set_3d_properties(z2)
    path1.set_data([], [])
    path1.set_3d_properties([])
    path2.set_data([], [])
    path2.set_3d_properties([])

    line1_xy.set_data(x1, y1)
    line2_xy.set_data(x2, y2)

    line1_xz.set_data(x1, z1)
    line2_xz.set_data(x2, z2)

    line1_yz.set_data(y1, z1)
    line2_yz.set_data(y2, z2)

    return line1, line2, path1, path2, line1_xy, line2_xy, line1_xz, line2_xz, line1_yz,line2_yz

# Update function for animation
def update(frame):
    curv1 = curvature1[frame]
    curv2 = curvature2[frame]
    phi1_val = phi1[frame]
    phi2_val = phi2[frame]
    
    x1, y1, z1, x2, y2, z2, end1, end2 = cal_tdcr(arc_length1, curv1, phi1_val, arc_length2, curv2, phi2_val)
    
    # Update the 3D lines
    line1.set_data(x1, y1)
    line1.set_3d_properties(z1)
    line2.set_data(x2, y2)
    line2.set_3d_properties(z2)

    # Update paths
    if frame == 0:
        path1.set_data(end1[0], end1[1])
        path1.set_3d_properties(end1[2])
        path2.set_data(end2[0], end2[1])
        path2.set_3d_properties(end2[2])
    else:
        x1_data, y1_data, z1_data = path1.get_data_3d()
        x1_data = np.append(x1_data, end1[0])
        y1_data = np.append(y1_data, end1[1])
        z1_data = np.append(z1_data, end1[2])
        path1.set_data_3d(x1_data, y1_data, z1_data)

        x2_data, y2_data, z2_data = path2.get_data_3d()
        x2_data = np.append(x2_data, end2[0])
        y2_data = np.append(y2_data, end2[1])
        z2_data = np.append(z2_data, end2[2])
        path2.set_data_3d(x2_data, y2_data, z2_data)

    # Update projections
    line1_xy.set_data(x1, y1)
    line2_xy.set_data(x2, y2)

    line1_xz.set_data(x1, z1)
    line2_xz.set_data(x2, z2)

    line1_yz.set_data(y1, z1)
    line2_yz.set_data(y2, z2)

    return line1, line2, path1, path2, line1_xy, line2_xy, line1_xz, line2_xz, line1_yz,line2_yz

# Create the animation
num_frames = len(curvature1)
ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, interval=100, blit=True, repeat=True)

# Show the plot
plt.show()

# Save as GIF
# ani.save('circle.gif', writer=PillowWriter(fps=15))
# # Save as MP4
# ani.save('circle.mp4', writer=FFMpegWriter(fps=15))