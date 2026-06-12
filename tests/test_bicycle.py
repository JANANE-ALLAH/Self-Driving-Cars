"""Unit tests for bicycle model."""

import pytest
import numpy as np
from src.bicycle_model import Bicycle, BicycleModel
from src.config import BicycleConfig


class TestBicycleConfig:
    """Test configuration management."""
    
    def test_default_config(self):
        """Test default configuration creation."""
        config = BicycleConfig()
        assert config.L == 2.0
        assert config.lr == 1.2
        assert config.w_max == 1.22
        assert config.sample_time == 0.01
    
    def test_custom_config(self):
        """Test custom configuration."""
        config = BicycleConfig(
            L=3.0,
            lr=1.5,
            w_max=2.0,
            sample_time=0.02
        )
        assert config.L == 3.0
        assert config.lr == 1.5
        assert config.w_max == 2.0
        assert config.sample_time == 0.02
    
    def test_config_validation_invalid_L(self):
        """Test validation of invalid wheelbase."""
        config = BicycleConfig(L=-1.0)
        with pytest.raises(ValueError):
            config.validate()
    
    def test_config_validation_invalid_lr(self):
        """Test validation of invalid lr."""
        config = BicycleConfig(lr=-1.0)
        with pytest.raises(ValueError):
            config.validate()
    
    def test_config_validation_invalid_w_max(self):
        """Test validation of invalid w_max."""
        config = BicycleConfig(w_max=-1.0)
        with pytest.raises(ValueError):
            config.validate()
    
    def test_config_validation_invalid_sample_time(self):
        """Test validation of invalid sample time."""
        config = BicycleConfig(sample_time=-0.01)
        with pytest.raises(ValueError):
            config.validate()
    
    def test_config_validation_valid(self):
        """Test validation of valid configuration."""
        config = BicycleConfig()
        assert config.validate() is True


class TestBicycleModel:
    """Test bicycle kinematic model."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.config = BicycleConfig()
        self.model = Bicycle(self.config)
    
    def test_initialization(self):
        """Test bicycle initialization."""
        assert self.model.xc == 0.0
        assert self.model.yc == 0.0
        assert self.model.theta == 0.0
        assert self.model.delta == 0.0
    
    def test_custom_initialization(self):
        """Test bicycle initialization with custom state."""
        config = BicycleConfig(
            initial_x=5.0,
            initial_y=3.0,
            initial_theta=np.pi/4
        )
        model = Bicycle(config)
        assert model.xc == 5.0
        assert model.yc == 3.0
        assert model.theta == np.pi/4
    
    def test_reset(self):
        """Test reset functionality."""
        self.model.step(1.0, 0.5)
        assert self.model.xc != 0.0 or self.model.yc != 0.0
        
        self.model.reset()
        assert self.model.xc == 0.0
        assert self.model.yc == 0.0
        assert self.model.theta == 0.0
    
    def test_step_straight_motion(self):
        """Test straight line motion."""
        # Move straight forward with no steering
        for _ in range(100):
            self.model.step(1.0, 0.0)
        
        # Should move mostly in positive X direction
        assert self.model.xc > 0.5
        assert abs(self.model.yc) < 0.1  # Should be close to Y=0
    
    def test_step_steering_rate_limitation(self):
        """Test steering rate is limited."""
        # Try to exceed max steering rate
        initial_delta = self.model.delta
        self.model.step(1.0, 10.0)  # Very large steering rate
        
        # Delta change should be limited by w_max
        max_delta_change = self.config.w_max * self.config.sample_time
        actual_delta_change = abs(self.model.delta - initial_delta)
        assert actual_delta_change <= max_delta_change + 1e-6
    
    def test_step_with_invalid_velocity(self):
        """Test step with invalid velocity input."""
        with pytest.raises(ValueError):
            self.model.step("invalid", 0.0)
    
    def test_step_with_invalid_steering_rate(self):
        """Test step with invalid steering rate input."""
        with pytest.raises(ValueError):
            self.model.step(1.0, "invalid")
    
    def test_get_state(self):
        """Test state retrieval."""
        self.model.set_state(1.0, 2.0, 0.5)
        state = self.model.get_state()
        
        assert state['x'] == 1.0
        assert state['y'] == 2.0
        assert state['theta'] == 0.5
        assert state['delta'] == 0.0
    
    def test_set_state(self):
        """Test state setting."""
        self.model.set_state(5.0, 3.0, np.pi/6, np.pi/12)
        
        assert self.model.xc == 5.0
        assert self.model.yc == 3.0
        assert self.model.theta == np.pi/6
        assert self.model.delta == np.pi/12
    
    def test_history_recording(self):
        """Test trajectory history recording."""
        for _ in range(10):
            self.model.step(1.0, 0.0, record_history=True)
        
        history = self.model.get_history()
        assert len(history['time']) == 10
        assert len(history['x']) == 10
        assert len(history['y']) == 10
    
    def test_history_clear(self):
        """Test history clearing."""
        for _ in range(10):
            self.model.step(1.0, 0.0, record_history=True)
        
        self.model.clear_history()
        history = self.model.get_history()
        assert len(history['time']) == 0


class TestBicycleModelTrajectory:
    """Test trajectory planning capabilities."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.config = BicycleConfig()
        self.model = BicycleModel(self.config)
    
    def test_simulate_trajectory(self):
        """Test trajectory simulation."""
        n = 100
        v_data = np.ones(n) * 1.0
        w_data = np.zeros(n)
        
        x_traj, y_traj = self.model.simulate_trajectory(v_data, w_data)
        
        assert len(x_traj) == n
        assert len(y_traj) == n
        assert x_traj[-1] > 0.5  # Should have moved forward
    
    def test_simulate_trajectory_mismatched_inputs(self):
        """Test trajectory simulation with mismatched input sizes."""
        v_data = np.ones(100)
        w_data = np.zeros(50)
        
        with pytest.raises(ValueError):
            self.model.simulate_trajectory(v_data, w_data)
    
    def test_circular_path(self):
        """Test circular path generation."""
        t, x, y, w = self.model.circular_path(
            radius=5.0,
            velocity=1.0,
            duration=10.0
        )
        
        assert len(t) > 0
        assert len(x) == len(y) == len(w)
    
    def test_figure8_path(self):
        """Test figure-8 path generation."""
        t, x, y, w = self.model.figure8_path(
            radius=5.0,
            velocity=1.0,
            duration=10.0
        )
        
        assert len(t) > 0
        assert len(x) == len(y) == len(w)
    
    def test_square_path(self):
        """Test square path generation."""
        t, x, y, w = self.model.square_path(
            side_length=10.0,
            velocity=1.0,
            duration=20.0
        )
        
        assert len(t) > 0
        assert len(x) == len(y) == len(w)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
