import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters of the manipulator
L1 = 1.0  # Length of the first link
L2 = 1.0  # Length of the second link

# PID Gains
Kp = 5.5
Ki = 0.1
Kd = 0.25

# Initialize PID variables
integral_x, integral_y = 0, 0
prev_error_x, prev_error_y = 0, 0

# Trajectory (circle)
t = np.linspace(0, 2 * np.pi, 100)
trajectory_x = 1.0 + 0.5 * np.cos(t)
trajectory_y = 0.5 + 0.5 * np.sin(t)

# Inverse Kinematics (basic)
def inverse_kinematics(x, y):
    d = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    d = np.clip(d, -1, 1)  # Ensure within valid range
    theta2 = np.arctan2(np.sqrt(1 - d**2), d)
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    return theta1, theta2

# Forward Kinematics
def forward_kinematics(theta1, theta2):
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return [0, x1, x2], [0, y1, y2]

# PID control for x and y
def pid_control(x_target, y_target, x_current, y_current, dt=0.05):
    global integral_x, integral_y, prev_error_x, prev_error_y

    # Compute errors
    error_x = x_target - x_current
    error_y = y_target - y_current

    # Integral term
    integral_x += error_x * dt
    integral_y += error_y * dt

    # Derivative term
    derivative_x = (error_x - prev_error_x) / dt
    derivative_y = (error_y - prev_error_y) / dt

    # PID control output
    control_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
    control_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

    prev_error_x, prev_error_y = error_x, error_y

    # Return controlled position
    return x_current + control_x * dt, y_current + control_y * dt

# Function to initialize the figure and plot
def initialize_plot():
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.grid()
    ax.plot(trajectory_x, trajectory_y, 'r--', label="Desired Trajectory")

    # Initialize lines for the manipulator and actual trajectory
    link1, = ax.plot([], [], 'b-', lw=3, label="Link 1")
    link2, = ax.plot([], [], 'g-', lw=3, label="Link 2")
    joint, = ax.plot([], [], 'ko', markersize=5, label="Joints")
    end_effector, = ax.plot([], [], 'ro', markersize=5, label="End Effector")
    actual_traj, = ax.plot([], [], 'c-', lw=1, label="Actual Trajectory")
    
    return fig, ax, link1, link2, joint, end_effector, actual_traj

# Function to update the plot during the animation
def update(frame, actual_x, actual_y, link1, link2, joint, end_effector, actual_traj):
    x_target = trajectory_x[frame]
    y_target = trajectory_y[frame]

    # PID control to get the current position
    x_control, y_control = pid_control(x_target, y_target, actual_x[-1], actual_y[-1])

    # Append controlled position to actual trajectory
    actual_x.append(x_control)
    actual_y.append(y_control)

    # Calculate inverse kinematics and forward kinematics
    theta1, theta2 = inverse_kinematics(x_control, y_control)
    x_vals, y_vals = forward_kinematics(theta1, theta2)

    # Update the plot data
    link1.set_data([0, x_vals[1]], [0, y_vals[1]])  # Link 1
    link2.set_data([x_vals[1], x_vals[2]], [y_vals[1], y_vals[2]])  # Link 2
    joint.set_data([0, x_vals[1], x_vals[2]], [0, y_vals[1], y_vals[2]])  # Joints
    end_effector.set_data(x_vals[2], y_vals[2])  # End-effector

    # Update actual trajectory plot
    actual_traj.set_data(actual_x, actual_y)

    return link1, link2, joint, end_effector, actual_traj

# Main function to organize the flow of execution
def main():
    global trajectory_x, trajectory_y

    # Initialize the plot and lines
    fig, ax, link1, link2, joint, end_effector, actual_traj = initialize_plot()

    # Initialize starting position of the end-effector
    actual_x, actual_y = [trajectory_x[0]], [trajectory_y[0]]

    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(t), fargs=(actual_x, actual_y, link1, link2, joint, end_effector, actual_traj), interval=50, blit=True)

    # Show the plot
    plt.legend()
    plt.show()

# Run the main function
if __name__ == "__main__":
    main()
