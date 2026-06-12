"""Utility functions for bicycle model analysis and visualization."""

import numpy as np
from typing import Tuple, List
import matplotlib.pyplot as plt
from pathlib import Path


def save_trajectory_data(
    t_data: np.ndarray,
    x_data: np.ndarray,
    y_data: np.ndarray,
    v_data: np.ndarray,
    w_data: np.ndarray,
    output_path: str
) -> None:
    """Save trajectory data to file.
    
    Args:
        t_data: Time array
        x_data: X position array
        y_data: Y position array
        v_data: Velocity array
        w_data: Steering rate array
        output_path: Path to output file
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    data = np.vstack([t_data, x_data, y_data, v_data, w_data]).T
    np.savetxt(output_path, data, delimiter=', ', header='time, x, y, v, w')


def load_trajectory_data(file_path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load trajectory data from file.
    
    Args:
        file_path: Path to trajectory file
        
    Returns:
        Tuple of (t, x, y, v, w) arrays
    """
    data = np.loadtxt(file_path, delimiter=', ', skiprows=1)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3], data[:, 4]


def plot_trajectory(
    x_data: np.ndarray,
    y_data: np.ndarray,
    label: str = 'Trajectory',
    show: bool = True,
    save_path: str = None
) -> None:
    """Plot 2D trajectory.
    
    Args:
        x_data: X coordinates
        y_data: Y coordinates
        label: Label for trajectory
        show: Whether to display plot
        save_path: Optional path to save figure
    """
    plt.figure(figsize=(10, 10))
    plt.axis('equal')
    plt.plot(x_data, y_data, label=label, linewidth=2)
    plt.scatter(x_data[0], y_data[0], color='green', s=100, marker='o', label='Start')
    plt.scatter(x_data[-1], y_data[-1], color='red', s=100, marker='x', label='End')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Vehicle Trajectory')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    if save_path:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    if show:
        plt.show()
    else:
        plt.close()


def plot_comparison(
    x1: np.ndarray,
    y1: np.ndarray,
    x2: np.ndarray,
    y2: np.ndarray,
    label1: str = 'Model 1',
    label2: str = 'Model 2',
    show: bool = True,
    save_path: str = None
) -> None:
    """Plot two trajectories for comparison.
    
    Args:
        x1: X coordinates of first trajectory
        y1: Y coordinates of first trajectory
        x2: X coordinates of second trajectory
        y2: Y coordinates of second trajectory
        label1: Label for first trajectory
        label2: Label for second trajectory
        show: Whether to display plot
        save_path: Optional path to save figure
    """
    plt.figure(figsize=(12, 10))
    plt.axis('equal')
    plt.plot(x1, y1, label=label1, linewidth=2, color='blue')
    plt.plot(x2, y2, label=label2, linewidth=2, color='orange')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    if save_path:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    if show:
        plt.show()
    else:
        plt.close()


def calculate_path_length(x_data: np.ndarray, y_data: np.ndarray) -> float:
    """Calculate total path length.
    
    Args:
        x_data: X coordinates
        y_data: Y coordinates
        
    Returns:
        Total path length in meters
    """
    dx = np.diff(x_data)
    dy = np.diff(y_data)
    return np.sum(np.sqrt(dx**2 + dy**2))


def calculate_max_curvature(theta_data: np.ndarray, dt: float) -> float:
    """Calculate maximum curvature from heading data.
    
    Args:
        theta_data: Heading angles
        dt: Sample time
        
    Returns:
        Maximum curvature
    """
    dtheta = np.diff(theta_data)
    curvature = np.abs(dtheta) / dt
    return np.max(curvature)


def plot_control_inputs(
    t_data: np.ndarray,
    v_data: np.ndarray,
    w_data: np.ndarray,
    show: bool = True,
    save_path: str = None
) -> None:
    """Plot control inputs over time.
    
    Args:
        t_data: Time array
        v_data: Velocity array
        w_data: Steering rate array
        show: Whether to display plot
        save_path: Optional path to save figure
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1.plot(t_data, v_data, linewidth=2, color='blue')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('Control Inputs')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(t_data, w_data, linewidth=2, color='orange')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Steering Rate (rad/s)')
    ax2.grid(True, alpha=0.3)
    
    if save_path:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    if show:
        plt.show()
    else:
        plt.close()
