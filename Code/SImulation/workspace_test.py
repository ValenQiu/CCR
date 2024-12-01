import numpy as np
import matplotlib.pyplot as plt

# Enable interactive mode
# plt.ion()

# Define the arc parameters
arc_length1 = 15
arc_length2 = 15
angles1 = np.linspace(-np.pi/2, np.pi/2, 51)
curvatures1 = angles1 / arc_length1
angles2 = np.linspace(-np.pi/2, np.pi/2, 100)
curvatures2 = angles2 / arc_length2

contour_x = []
contour_y = []

# Create the figure and axis with a fixed size
fig, ax = plt.subplots(figsize=(5, 5))  # Set the figure size

for i, curvature1 in enumerate(curvatures1):
    # Calculate the start and end angles of the arc
    if curvature1 == 0:
        # If the curvature is 0, the arc is a straight line
        theta1 = np.linspace(0, 0, 100)
        y1 = np.linspace(0, arc_length1, 100)
        x1 = np.zeros_like(y1)
        end_x1, end_y1 = 0, arc_length1  # End point of the straight line
    else:
        # Create the arc coordinates
        theta1 = np.linspace(0, arc_length1 * curvature1, 100)
        x1 = np.cos(theta1) / curvature1
        y1 = np.sin(theta1) / curvature1
        
        # Shift the arc to start at (0, 0)
        x1 = x1 - x1[0]
        y1 = y1 - y1[0]
        
        # Calculate the end point of the arc
        end_x1 = x1[-1]
        end_y1 = y1[-1]

    # Plot the arc
    ax.plot(x1, y1, color='blue', linewidth=2)

    for j, curvature2 in enumerate(curvatures2):
        if curvature2 == 0:
            start_angle2 = theta1[-1]
            end_angle2 = start_angle2
            
            x2 = np.zeros_like(np.linspace(0, arc_length2, 100))
            y2 = np.linspace(0, arc_length2, 100)
            
            # Rotate the second segment
            angle = start_angle2
            rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                        [np.sin(angle), np.cos(angle)]])
            rotated_coords = np.dot(rotation_matrix, [x2, y2])
            x2 = rotated_coords[0] + end_x1
            y2 = rotated_coords[1] + end_y1
            
        else:
            theta2 = np.linspace(theta1[-1], theta1[-1] + arc_length2 * curvature2, 100)
            x2 = np.cos(theta2) / curvature2
            y2 = np.sin(theta2) / curvature2
            
            # Shift the arc
            x2 = x1[-1] + (x2 - x2[0])
            y2 = y1[-1] + (y2 - y2[0])

        # Calculate tip positions
        tip_x = x2[-1]
        tip_y = y2[-1]
        
        # Clamp the tip positions to desired limits
        tip_x = np.clip(tip_x, -25, 25)
        tip_y = np.clip(tip_y, 0, 50)

        # Debugging output
        print(f"Tip Position: ({tip_x}, {tip_y})")

        contour_x.append(tip_x)
        contour_y.append(tip_y)

        # Plot the arc
        ax.plot(x2, y2, color='green', linewidth=1, alpha=0.1)
        # plt.pause(0.01)  # Allow the plot to update

# Plotting points
ax.scatter(contour_x, contour_y, marker='o', color='red', s=3)

# Set the aspect ratio and limits
ax.set_aspect('equal')
ax.set_xlim(-25, 25)
ax.set_ylim(0, 50)

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Arcs with Tangent Dotted Lines')

# Show the plot
plt.show()