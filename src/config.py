"""Configuration management for bicycle model."""

import json
import yaml
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Union, Dict, Any


@dataclass
class BicycleConfig:
    """Configuration for bicycle kinematic model.
    
    Attributes:
        L (float): Wheelbase distance (meters)
        lr (float): Distance from rear axle to center of mass (meters)
        w_max (float): Maximum steering rate (rad/s)
        sample_time (float): Sampling time (seconds)
        initial_x (float): Initial x position (meters)
        initial_y (float): Initial y position (meters)
        initial_theta (float): Initial heading angle (radians)
        initial_delta (float): Initial steering angle (radians)
    """
    L: float = 2.0
    lr: float = 1.2
    w_max: float = 1.22
    sample_time: float = 0.01
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_theta: float = 0.0
    initial_delta: float = 0.0
    
    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "BicycleConfig":
        """Load configuration from JSON file.
        
        Args:
            path: Path to JSON configuration file
            
        Returns:
            BicycleConfig instance
            
        Raises:
            FileNotFoundError: If configuration file not found
            ValueError: If configuration is invalid
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {path}")
        
        with open(path, 'r') as f:
            data = json.load(f)
        
        return cls(**data)
    
    @classmethod
    def from_yaml(cls, path: Union[str, Path]) -> "BicycleConfig":
        """Load configuration from YAML file.
        
        Args:
            path: Path to YAML configuration file
            
        Returns:
            BicycleConfig instance
            
        Raises:
            FileNotFoundError: If configuration file not found
            ValueError: If configuration is invalid
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found: {path}")
        
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        return cls(**data)
    
    def to_json(self, path: Union[str, Path]) -> None:
        """Save configuration to JSON file.
        
        Args:
            path: Path to output JSON file
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, 'w') as f:
            json.dump(asdict(self), f, indent=2)
    
    def to_yaml(self, path: Union[str, Path]) -> None:
        """Save configuration to YAML file.
        
        Args:
            path: Path to output YAML file
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, 'w') as f:
            yaml.dump(asdict(self), f, default_flow_style=False)
    
    def validate(self) -> bool:
        """Validate configuration parameters.
        
        Returns:
            True if valid
            
        Raises:
            ValueError: If any parameter is invalid
        """
        if self.L <= 0:
            raise ValueError("Wheelbase (L) must be positive")
        if self.lr <= 0:
            raise ValueError("Distance to CM (lr) must be positive")
        if self.w_max <= 0:
            raise ValueError("Max steering rate (w_max) must be positive")
        if self.sample_time <= 0:
            raise ValueError("Sample time must be positive")
        
        return True
