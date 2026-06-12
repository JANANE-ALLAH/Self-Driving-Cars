"""Example usage of bicycle kinematic model."""

import numpy as np
import matplotlib.pyplot as plt
from src.bicycle_model import Bicycle, BicycleModel
from src.config import BicycleConfig
from src import utils


def example_straight_line():
    """Example: Straight line motion."""
    print("\n=== Example 1: Straight Line Motion ===")
    
    config = BicycleConfig()
    model = Bicycle(config)
    
    # Simulate straight line motion
    n_steps = 200
    for i in range(n_steps):
        model.step(v=1.0, w=0.0)  # 1 m/s, no steering
    
    print(f"Final position: ({model.xc:.2f}, {model.yc:.2f})")
    print(f"Final heading: {np.degrees(model.theta):.2f}°")


def example_circular_motion():
    """Example: Circular motion."""
    print("\n=== Example 2: Circular Motion ===")
    
    config = BicycleConfig()
    model = BicycleModel(config)
    
    # Generate circular trajectory
    t_data, x_traj, y_traj, w_data = model.circular_path(
        radius=5.0,
        velocity=2.0,
        duration=15.0
    )
    
    print(f"Trajectory steps: {len(t_data)}")
    print(f"Path length: {utils.calculate_path_length(x_traj, y_traj):.2f} meters")
    
    # Save and plot
    utils.save_trajectory_data(
        t_data, x_traj, y_traj,
        np.ones_like(t_data) * 2.0, w_data,
        'output/circular_path.txt'
    )
    utils.plot_trajectory(x_traj, y_traj, label='Circular Path', show=False,
                         save_path='output/circular_path.png')


def example_figure8():
    """Example: Figure-8 trajectory."""
    print("\n=== Example 3: Figure-8 Trajectory ===")
    
    config = BicycleConfig()
    model = BicycleModel(config)
    
    # Generate figure-8 trajectory
    t_data, x_traj, y_traj, w_data = model.figure8_path(
        radius=4.0,
        velocity=1.5,
        duration=30.0
    )
    
    print(f"Trajectory steps: {len(t_data)}")
    print(f"Path length: {utils.calculate_path_length(x_traj, y_traj):.2f} meters")
    
    # Save and plot
    utils.save_trajectory_data(
        t_data, x_traj, y_traj,
        np.ones_like(t_data) * 1.5, w_data,
        'output/figure8_path.txt'
    )
    utils.plot_trajectory(x_traj, y_traj, label='Figure-8 Path', show=False,
                         save_path='output/figure8_path.png')


def example_custom_trajectory():
    """Example: Custom trajectory with predefined inputs."""
    print("\n=== Example 4: Custom Trajectory ===")
    
    config = BicycleConfig(sample_time=0.01)
    model = BicycleModel(config)
    
    # Create custom velocity and steering rate profiles
    n_steps = 500
    v_data = np.ones(n_steps) * 2.0  # Constant velocity
    w_data = np.zeros(n_steps)
    
    # Apply steering commands at specific times
    w_data[100:200] = 0.5   # Turn right
    w_data[300:400] = -0.5  # Turn left
    
    x_traj, y_traj = model.simulate_trajectory(v_data, w_data)
    
    print(f"Trajectory steps: {len(x_traj)}")
    print(f"Path length: {utils.calculate_path_length(x_traj, y_traj):.2f} meters")
    
    # Create plots
    t_data = np.arange(n_steps) * config.sample_time
    utils.plot_trajectory(x_traj, y_traj, label='Custom Path', show=False,
                         save_path='output/custom_path.png')
    utils.plot_control_inputs(t_data, v_data, w_data, show=False,
                             save_path='output/control_inputs.png')


def example_config_files():
    """Example: Loading and saving configurations."""
    print("\n=== Example 5: Configuration Files ===")
    
    # Create custom configuration
    config = BicycleConfig(
        L=2.5,
        lr=1.3,
        w_max=1.5,
        sample_time=0.01
    )
    
    # Save to JSON and YAML
    config.to_json('output/custom_config.json')
    config.to_yaml('output/custom_config.yaml')
    print("Configuration saved to JSON and YAML files")
    
    # Load back from files
    config_from_json = BicycleConfig.from_json('output/custom_config.json')
    config_from_yaml = BicycleConfig.from_yaml('output/custom_config.yaml')
    
    print(f"Loaded from JSON - L: {config_from_json.L}, lr: {config_from_json.lr}")
    print(f"Loaded from YAML - L: {config_from_yaml.L}, lr: {config_from_yaml.lr}")


def example_trajectory_comparison():
    """Example: Compare trajectories with different parameters."""
    print("\n=== Example 6: Trajectory Comparison ===")
    
    # Create two models with different configurations
    config1 = BicycleConfig(L=2.0)
    config2 = BicycleConfig(L=3.0)
    
    model1 = BicycleModel(config1)
    model2 = BicycleModel(config2)
    
    # Generate same trajectory with different models
    n_steps = 300
    v_data = np.ones(n_steps) * 1.5
    w_data = np.sin(np.linspace(0, 4*np.pi, n_steps)) * 0.5
    
    x1, y1 = model1.simulate_trajectory(v_data, w_data)
    x2, y2 = model2.simulate_trajectory(v_data, w_data)
    
    # Compare trajectories
    utils.plot_comparison(x1, y1, x2, y2,
                         label1=f'L={config1.L}m',
                         label2=f'L={config2.L}m',
                         show=False,
                         save_path='output/trajectory_comparison.png')
    
    print(f"Model 1 path length: {utils.calculate_path_length(x1, y1):.2f} m")
    print(f"Model 2 path length: {utils.calculate_path_length(x2, y2):.2f} m")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("Self-Driving Cars - Bicycle Model Examples")
    print("="*60)
    
    # Run all examples
    example_straight_line()
    example_circular_motion()
    example_figure8()
    example_custom_trajectory()
    example_config_files()
    example_trajectory_comparison()
    
    print("\n" + "="*60)
    print("All examples completed successfully!")
    print("Check 'output' directory for generated files.")
    print("="*60 + "\n")
