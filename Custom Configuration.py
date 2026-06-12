# Create custom configuration
config = BicycleConfig(
    L=2.5,           # Wheelbase: 2.5 meters
    lr=1.3,          # Distance to CM: 1.3 meters
    w_max=1.5,       # Max steering rate: 1.5 rad/s
    sample_time=0.01 # 10 ms sampling
)

# Validate before use
config.validate()

# Save configuration
config.to_yaml('my_config.yaml')
config.to_json('my_config.json')
