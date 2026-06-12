"""Kinematic bicycle model implementation."""

import numpy as np
from typing import Tuple, List
from src.config import BicycleConfig


class Bicycle:
    """Base bicycle kinematic model.
    
    This class implements a kinematic bicycle model that simulates the motion
    of a bicycle-like vehicle based on velocity and steering rate inputs.
    """
    
    def __init__(self, config: BicycleConfig = None):
        """Initialize bicycle model with configuration.
        
        Args:
            config: BicycleConfig instance. If None, uses default configuration.
        """
        if config is None:
            config = BicycleConfig()
        
        config.validate()
        self.config = config
        
        # State variables
        self.xc = config.initial_x
        self.yc = config.initial_y
        self.theta = config.initial_theta
        self.delta = config.initial_delta
        self.beta = 0.0  # Slip angle
        
        # History for analysis
        self._history = {
            'time': [],
            'x': [],
            'y': [],
            'theta': [],
            'delta': [],
            'beta': [],
            'v': [],
            'w': []
        }
    
    def reset(self) -> None:
        """Reset bicycle state to initial conditions."""
        self.xc = self.config.initial_x
        self.yc = self.config.initial_y
        self.theta = self.config.initial_theta
        self.delta = self.config.initial_delta
        self.beta = 0.0
        self._history = {
            'time': [],
            'x': [],
            'y': [],
            'theta': [],
            'delta': [],
            'beta': [],
            'v': [],
            'w': []
        }
    
    def step(self, v: float, w: float, record_history: bool = False) -> None:
        """Update bicycle state using kinematic model.
        
        Args:
            v: Longitudinal velocity (m/s)
            w: Steering rate (rad/s)
            record_history: Whether to record state in history
            
        Raises:
            ValueError: If inputs are invalid
        """
        if not isinstance(v, (int, float)) or not isinstance(w, (int, float)):
            raise ValueError("Velocity and steering rate must be numeric")
        
        # Limit steering rate
        w = np.clip(w, -self.config.w_max, self.config.w_max)
        
        # Calculate slip angle
        self.beta = np.arctan(self.config.lr * np.tan(self.delta) / self.config.L)
        
        # Kinematic equations (differential equations)
        xc_dot = v * np.cos(self.theta + self.beta)
        yc_dot = v * np.sin(self.theta + self.beta)
        theta_dot = (v / self.config.L) * (np.cos(self.beta) * np.tan(self.delta))
        delta_dot = w
        
        # Update state using Euler integration
        dt = self.config.sample_time
        self.xc += xc_dot * dt
        self.yc += yc_dot * dt
        self.theta += theta_dot * dt
        self.delta += delta_dot * dt
        
        # Record history if requested
        if record_history:
            time = len(self._history['time']) * dt
            self._history['time'].append(time)
            self._history['x'].append(self.xc)
            self._history['y'].append(self.yc)
            self._history['theta'].append(self.theta)
            self._history['delta'].append(self.delta)
            self._history['beta'].append(self.beta)
            self._history['v'].append(v)
            self._history['w'].append(w)
    
    def get_state(self) -> dict:
        """Get current bicycle state.
        
        Returns:
            Dictionary with state variables
        """
        return {
            'x': self.xc,
            'y': self.yc,
            'theta': self.theta,
            'delta': self.delta,
            'beta': self.beta
        }
    
    def set_state(self, x: float, y: float, theta: float, delta: float = 0.0) -> None:
        """Set bicycle state directly.
        
        Args:
            x: X position
            y: Y position
            theta: Heading angle
            delta: Steering angle (default: 0)
        """
        self.xc = float(x)
        self.yc = float(y)
        self.theta = float(theta)
        self.delta = float(delta)
    
    def get_history(self) -> dict:
        """Get recorded history.
        
        Returns:
            Dictionary with recorded state history
        """
        return self._history
    
    def clear_history(self) -> None:
        """Clear recorded history."""
        self._history = {
            'time': [],
            'x': [],
            'y': [],
            'theta': [],
            'delta': [],
            'beta': [],
            'v': [],
            'w': []
        }


class BicycleModel(Bicycle):
    """Extended bicycle model with trajectory planning capabilities."""
    
    def simulate_trajectory(
        self,
        v_data: np.ndarray,
        w_data: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Simulate trajectory given velocity and steering rate commands.
        
        Args:
            v_data: Array of velocities (m/s)
            w_data: Array of steering rates (rad/s)
            
        Returns:
            Tuple of (x_trajectory, y_trajectory) arrays
            
        Raises:
            ValueError: If input arrays have different lengths
        """
        if len(v_data) != len(w_data):
            raise ValueError("v_data and w_data must have same length")
        
        self.reset()
        self.clear_history()
        
        n = len(v_data)
        x_traj = np.zeros(n)
        y_traj = np.zeros(n)
        
        for i in range(n):
            x_traj[i] = self.xc
            y_traj[i] = self.yc
            self.step(float(v_data[i]), float(w_data[i]), record_history=True)
        
        return x_traj, y_traj
    
    def circular_path(
        self,
        radius: float,
        velocity: float,
        duration: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Generate circular path trajectory.
        
        Args:
            radius: Circle radius (meters)
            velocity: Constant velocity (m/s)
            duration: Simulation duration (seconds)
            
        Returns:
            Tuple of (t, x, y, w) arrays
        """
        dt = self.config.sample_time
        n_steps = int(duration / dt)
        
        # Calculate required steering angle for circular path
        delta_required = 0.993 * np.arctan(self.config.L / radius)
        
        # Calculate steering rate needed
        t_data = np.arange(n_steps) * dt
        w_data = np.zeros(n_steps)
        v_data = np.ones(n_steps) * velocity
        
        # Steering phase
        steer_steps = n_steps // 4
        w_data[:steer_steps] = self.config.w_max
        w_data[steer_steps:3*steer_steps] = -self.config.w_max
        w_data[3*steer_steps:] = 0
        
        x_traj, y_traj = self.simulate_trajectory(v_data, w_data)
        
        return t_data, x_traj, y_traj, w_data
    
    def figure8_path(
        self,
        radius: float,
        velocity: float,
        duration: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Generate figure-8 path trajectory.
        
        Args:
            radius: Lobe radius (meters)
            velocity: Constant velocity (m/s)
            duration: Simulation duration (seconds)
            
        Returns:
            Tuple of (t, x, y, w) arrays
        """
        dt = self.config.sample_time
        n_steps = int(duration / dt)
        
        delta_required = 0.993 * np.arctan(self.config.L / radius)
        
        t_data = np.arange(n_steps) * dt
        w_data = np.zeros(n_steps)
        v_data = np.ones(n_steps) * velocity
        
        # Figure 8 pattern
        eighth = n_steps // 8
        w_data[:eighth] = self.config.w_max
        w_data[eighth:5*eighth] = -self.config.w_max
        w_data[5*eighth:] = self.config.w_max
        
        x_traj, y_traj = self.simulate_trajectory(v_data, w_data)
        
        return t_data, x_traj, y_traj, w_data
    
    def square_path(
        self,
        side_length: float,
        velocity: float,
        duration: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Generate square path trajectory.
        
        Args:
            side_length: Side length of square (meters)
            velocity: Constant velocity (m/s)
            duration: Simulation duration (seconds)
            
        Returns:
            Tuple of (t, x, y, w) arrays
        """
        dt = self.config.sample_time
        n_steps = int(duration / dt)
        
        t_data = np.arange(n_steps) * dt
        w_data = np.zeros(n_steps)
        v_data = np.ones(n_steps) * velocity
        
        # Square turning points (corners)
        step_per_side = n_steps // 4
        corner_duration = step_per_side // 10
        
        for corner in range(4):
            start = corner * step_per_side
            corner_start = start + step_per_side - corner_duration
            corner_end = start + step_per_side
            
            if corner % 2 == 0:
                w_data[corner_start:corner_end] = self.config.w_max
            else:
                w_data[corner_start:corner_end] = -self.config.w_max
        
        x_traj, y_traj = self.simulate_trajectory(v_data, w_data)
        
        return t_data, x_traj, y_traj, w_data
