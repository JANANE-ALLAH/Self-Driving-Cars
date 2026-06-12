from src.bicycle_model import Bicycle
from src.config import BicycleConfig

# Create model with default configuration
config = BicycleConfig()
model = Bicycle(config)

# Simulate motion
for _ in range(100):
    model.step(v=1.0, w=0.0)  # 1 m/s velocity, no steering

print(f"Position: ({model.xc:.2f}, {model.yc:.2f})")
print(f"Heading: {model.theta:.3f} rad")
