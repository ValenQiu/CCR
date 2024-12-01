import numpy as np
import matplotlib.pyplot as plt

def circular_path(theta_fixed, phi_range, L=300, num_points=100):
    """Generate circular path points"""
    theta = np.ones(num_points) * theta_fixed
    phi = np.linspace(phi_range[0], phi_range[1], num_points)
    
    x = np.zeros(num_points)
    y = np.zeros(num_points)
    z = np.zeros(num_points)
    
    for i in range(num_points):
        x[i] = (L/theta[i]) * (1-np.cos(theta[i])) * np.cos(phi[i])
        y[i] = (L/theta[i]) * (1-np.cos(theta[i])) * np.sin(phi[i])
        z[i] = (L/theta[i]) * np.sin(theta[i])
        
    return x, y, z, theta, phi

def calculate_jacobian(theta, phi, L=300):
    eps = 1e-10
    if abs(theta) < eps:
        theta = eps
        
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    
    dx_dtheta = L * (-cos_theta*cos_phi/theta**2 + sin_theta*cos_phi/theta - (1-cos_theta)*cos_phi/theta**2)
    dx_dphi = -L * (1-cos_theta)*sin_phi/theta
    
    dy_dtheta = L * (-cos_theta*sin_phi/theta**2 + sin_theta*sin_phi/theta - (1-cos_theta)*sin_phi/theta**2)
    dy_dphi = L * (1-cos_theta)*cos_phi/theta
    
    dz_dtheta = L * (cos_theta/theta - sin_theta/theta**2)
    dz_dphi = 0
    
    J = np.array([[dx_dtheta, dx_dphi],
                  [dy_dtheta, dy_dphi],
                  [dz_dtheta, dz_dphi]])
    
    return J

def track_path():
    # Parameters
    L = 300
    num_points = 100
    max_iterations = 100
    convergence_threshold = 1e-6
    
    # 1. Create circular path
    theta_fixed = np.pi/3
    phi_range = [0, 2*np.pi]
    path_x, path_y, path_z, _, path_phi = circular_path(theta_fixed, phi_range, L, num_points)
    
    # 2. Initialize arrays to store results
    path_theta_output = np.zeros(num_points)
    path_phi_output = np.zeros(num_points)
    
    # Set initial position and psi
    current_theta = theta_fixed
    current_phi = path_phi[0]
    path_theta_output[0] = current_theta
    path_phi_output[0] = current_phi
    
    # 3-6. Track the path
    for i in range(1, num_points):
        # Target position
        target_pos = np.array([path_x[i], path_y[i], path_z[i]])
        
        # Current position
        current_pos = np.array([(L/current_theta)*(1-np.cos(current_theta))*np.cos(current_phi),
                               (L/current_theta)*(1-np.cos(current_theta))*np.sin(current_phi),
                               (L/current_theta)*np.sin(current_theta)])
        
        # Iterative correction using Jacobian
        for iter in range(max_iterations):
            # Calculate position error
            pos_error = target_pos - current_pos
            
            if np.linalg.norm(pos_error) < convergence_threshold:
                break
                
            # Calculate Jacobian and its pseudoinverse
            J = calculate_jacobian(current_theta, current_phi, L)
            J_pinv = np.linalg.pinv(J)
            
            # Calculate delta psi
            delta_psi = J_pinv @ pos_error
            
            # Update current psi
            current_theta += delta_psi[0]
            current_phi += delta_psi[1]
            
            # Update current position
            current_pos = np.array([(L/current_theta)*(1-np.cos(current_theta))*np.cos(current_phi),
                                  (L/current_theta)*(1-np.cos(current_theta))*np.sin(current_phi),
                                  (L/current_theta)*np.sin(current_theta)])
        
        # Store results
        path_theta_output[i] = current_theta
        path_phi_output[i] = current_phi
    
    return path_theta_output, path_phi_output, path_x, path_y, path_z

# Run the tracking and plot results
if __name__ == "__main__":
    theta_out, phi_out, x_path, y_path, z_path = track_path()
    L = 300
    path_x_output = (L/theta_out) * (1-np.cos(theta_out)) * np.cos(phi_out)
    path_y_output = (L/theta_out) * (1-np.cos(theta_out)) * np.sin(phi_out)
    path_z_output = (L/theta_out) * np.sin(theta_out)
    
    # Plot results
    fig = plt.figure(figsize=(15, 5))
    
    # Plot theta
    ax1 = fig.add_subplot(121)
    ax1.plot(theta_out, label='theta')
    ax1.plot(phi_out, label='phi')
    ax1.set_title('Joint Angles vs Point Index')
    ax1.legend()
    
    # Plot 3D path
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(x_path, y_path, z_path, 'b-', label='Target Path')
    ax2.plot(path_x_output, path_y_output, path_z_output, 'r-', label='Actiral Path')
    ax2.set_title('3D Path')
    ax2.legend()
    
    plt.show()