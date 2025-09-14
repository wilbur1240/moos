# Marine Adaptive Control with Residual RL - Complete Implementation Guide

## Project Overview

**Objective**: Develop a residual reinforcement learning system to enhance PID-based marine vessel control for smooth waypoint navigation in varying sea conditions.

**Key Innovation**: RL agent observes baseline MOOS PID commands and vessel state, then outputs improved control commands adapted to current environmental conditions.

**Target Application**: Smooth waypoint navigation in open sea areas with varying wind, wave, and current conditions.

## System Architecture

```
MOOS ←→ MOOS-ROS Bridge ←→ ROS/Gazebo Simulation Environment
                                           ↓
                                    Residual RL Controller
                                           ↓
                                    Enhanced Control Commands
```

## Phase 1: Environment Setup & Data Pipeline

### 1.1 Gazebo Marine Environment Configuration

#### Marine Physics Setup
- **Wave Models**: Implement JONSWAP wave spectrum with configurable parameters
- **Wind Field**: Realistic wind patterns with turbulence and gusts
- **Current Flow**: Tidal and current effects with spatial variation
- **Vessel Dynamics**: High-fidelity boat model matching real vessel characteristics

#### Sensor Simulation
```yaml
sensors:
  gps:
    - position: [x, y, z]
    - velocity: [vx, vy, vz] 
    - noise: realistic_gps_noise_model
  
  imu:
    - orientation: [roll, pitch, yaw]
    - angular_velocity: [wx, wy, wz]
    - linear_acceleration: [ax, ay, az]
  
  compass:
    - magnetic_heading: with_declination_correction
```

### 1.2 MOOS-ROS Bridge Configuration

#### MOOS Variable Mapping
```yaml
moos_to_ros_mapping:
  # Navigation State
  NAV_X: /moos/boat_pose/x
  NAV_Y: /moos/boat_pose/y  
  NAV_HEADING: /moos/boat_pose/heading
  NAV_SPEED: /moos/boat_speed/surge
  
  # PID Controller State
  PID_HEADING_KP: /moos/pid_params/heading_kp
  PID_HEADING_KI: /moos/pid_params/heading_ki
  PID_HEADING_KD: /moos/pid_params/heading_kd
  PID_SPEED_KP: /moos/pid_params/speed_kp
  PID_SPEED_KI: /moos/pid_params/speed_ki
  PID_SPEED_KD: /moos/pid_params/speed_kd
  
  # Control Commands  
  DESIRED_THRUST: /moos/desired_thrust
  DESIRED_RUDDER: /moos/desired_rudder
  
  # Navigation Context - UPDATED
  WPT_DIST: /moos/waypoint_distance
  WPT_ANGLE: /moos/waypoint_bearing
  XTE: /moos/cross_track_error
  DESIRED_HEADING: /moos/desired_heading
  DESIRED_SPEED: /moos/desired_speed
```

#### ROS to MOOS Mapping
```yaml
ros_to_moos_mapping:
  # RL Control Commands
  /rl/desired_thrust: RL_DESIRED_THRUST
  /rl/desired_rudder: RL_DESIRED_RUDDER
  
  # Alternative: Velocity Commands
  /rl/cmd_vel/linear/x: RL_DESIRED_SPEED
  /rl/cmd_vel/angular/z: RL_DESIRED_YAW_RATE
```

## Phase 2: Baseline PID Integration

### 2.1 PID Controller Interface

#### ROS Node: `baseline_pid_interface.py`
```python
class BaselinePIDInterface:
    def __init__(self):
        # Subscribers for MOOS PID state
        self.pid_params_sub = rospy.Subscriber('/moos/pid_params', 
                                               PIDParams, 
                                               self.pid_params_callback)
        self.thrust_cmd_sub = rospy.Subscriber('/moos/desired_thrust', 
                                               Float64, 
                                               self.thrust_callback)
        self.rudder_cmd_sub = rospy.Subscriber('/moos/desired_rudder', 
                                               Float64, 
                                               self.rudder_callback)
        
        # Additional navigation context subscribers - UPDATED
        self.desired_heading_sub = rospy.Subscriber('/moos/desired_heading',
                                                    Float64,
                                                    self.desired_heading_callback)
        self.desired_speed_sub = rospy.Subscriber('/moos/desired_speed',
                                                  Float64, 
                                                  self.desired_speed_callback)
        
        # Publishers for logging and analysis
        self.pid_performance_pub = rospy.Publisher('/analysis/pid_performance', 
                                                   PIDPerformance, 
                                                   queue_size=10)
        
        # Initialize navigation context variables
        self.desired_heading = 0.0
        self.desired_speed = 0.0
    
    def desired_heading_callback(self, msg):
        """Store desired heading from waypoint navigation"""
        self.desired_heading = msg.data
    
    def desired_speed_callback(self, msg):
        """Store desired speed from waypoint navigation"""
        self.desired_speed = msg.data
    
    def evaluate_pid_performance(self):
        """Calculate PID tracking performance metrics"""
        # Calculate current tracking error for history
        tracking_error = np.sqrt(
            (self.current_xte ** 2) + 
            (self.heading_error ** 2)
        )
        
        return {
            'cross_track_error': self.current_xte,
            'heading_error': self.heading_error,
            'control_effort': self.control_effort,
            'tracking_smoothness': self.calculate_smoothness(),
            'tracking_error': tracking_error  # For historical tracking
        }
```

### 2.2 Data Collection Framework

#### Performance Logging
```python
class MarineDataLogger:
    def __init__(self):
        self.data_buffer = {
            'timestamp': [],
            'boat_state': [],
            'pid_commands': [],
            'environmental_conditions': [],
            'performance_metrics': []
        }
    
    def log_episode_data(self):
        """Log complete episode for training dataset"""
        episode_data = {
            'conditions': self.environmental_state,
            'trajectory': self.boat_trajectory,
            'commands': self.control_commands,
            'performance': self.performance_metrics
        }
        self.save_episode(episode_data)
```

## Phase 3: Residual RL System Design

### 3.1 Observation Space Definition

```python
class MarineObservationSpace:
    def __init__(self):
        # Historical context window size
        self.history_window_size = 10
        
        self.observation_space = spaces.Dict({
            # Baseline Controller State (6 values)
            'pid_params': spaces.Box(low=0.0, high=10.0, shape=(6,)),
            
            # MOOS PID Desired Commands (2 values)
            'moos_commands': spaces.Box(low=-1.0, high=1.0, shape=(2,)),
            
            # Vessel State (6 values)
            'boat_pose': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),  # x, y, heading
            'boat_speed': spaces.Box(low=-10.0, high=10.0, shape=(3,)),     # surge, sway, sog
            
            # Navigation Context (5 values) - UPDATED
            'navigation_context': spaces.Box(low=-np.inf, high=np.inf, shape=(5,)),  # dist, bearing, xte, des_hdg, des_speed
            
            # Historical Context (sliding window - 30 values) - UPDATED
            'control_history': spaces.Box(low=-np.inf, high=np.inf, shape=(30,)), # last 10 thrust+rudder+tracking_error
        })
        
        # Initialize historical buffers
        self.recent_thrust_commands = deque(maxlen=self.history_window_size)
        self.recent_rudder_commands = deque(maxlen=self.history_window_size)
        self.recent_tracking_errors = deque(maxlen=self.history_window_size)
        
        # Fill with zeros initially
        for _ in range(self.history_window_size):
            self.recent_thrust_commands.append(0.0)
            self.recent_rudder_commands.append(0.0)
            self.recent_tracking_errors.append(0.0)
    
    def update_control_history(self, thrust_cmd, rudder_cmd, tracking_error):
        """Update historical control and performance data"""
        self.recent_thrust_commands.append(thrust_cmd)
        self.recent_rudder_commands.append(rudder_cmd)
        self.recent_tracking_errors.append(tracking_error)
    
    def get_observation(self):
        """Compile current observation vector"""
        # Prepare control history array
        control_history_array = []
        control_history_array.extend(list(self.recent_thrust_commands))      # 10 values
        control_history_array.extend(list(self.recent_rudder_commands))      # 10 values
        control_history_array.extend(list(self.recent_tracking_errors))      # 10 values
        
        return {
            'pid_params': np.array([
                self.current_pid_params['heading_kp'],
                self.current_pid_params['heading_ki'], 
                self.current_pid_params['heading_kd'],
                self.current_pid_params['speed_kp'],
                self.current_pid_params['speed_ki'],
                self.current_pid_params['speed_kd']
            ]),
            'moos_commands': np.array([
                self.moos_desired_thrust,
                self.moos_desired_rudder
            ]),
            'boat_pose': np.array([
                self.boat_pose.x,
                self.boat_pose.y, 
                self.boat_pose.heading
            ]),
            'boat_speed': np.array([
                self.boat_speed.surge,
                self.boat_speed.sway,
                self.boat_speed.speed_over_ground
            ]),
            'navigation_context': np.array([
                self.distance_to_waypoint,
                self.bearing_to_waypoint,
                self.cross_track_error,
                self.desired_heading,
                self.desired_speed
            ]),
            'control_history': np.array(control_history_array)
        }
```

### 3.2 Action Space Definition

```python
class MarineActionSpace:
    def __init__(self):
        # Direct Control Commands
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),   # [thrust, rudder]
            high=np.array([1.0, 1.0]),    # [thrust, rudder]
            dtype=np.float32
        )
    
    def process_action(self, action):
        """Convert RL action to ROS messages"""
        thrust_msg = Float64()
        rudder_msg = Float64()
        
        thrust_msg.data = np.clip(action[0], -1.0, 1.0)
        rudder_msg.data = np.clip(action[1], -1.0, 1.0)
        
        return thrust_msg, rudder_msg
```

### 3.3 Reward Function Design

```python
class MarineRewardFunction:
    def __init__(self):
        self.reward_weights = {
            'waypoint_progress': 2.0,
            'path_efficiency': 1.5,
            'control_smoothness': 1.0,
            'tracking_accuracy': 1.0,
            'comfort': 0.8,
            'fuel_efficiency': 0.5
        }
    
    def calculate_reward(self, state, action, next_state):
        """Calculate comprehensive reward for waypoint navigation"""
        
        # Primary Navigation Performance
        waypoint_progress = self.calculate_waypoint_progress_rate(state, next_state)
        path_efficiency = self.calculate_path_efficiency(state, next_state)
        
        # Control Quality
        control_smoothness = self.calculate_control_smoothness(action)
        tracking_accuracy = self.calculate_tracking_performance(state)
        
        # Secondary Objectives
        comfort_metric = self.calculate_passenger_comfort(next_state)
        fuel_efficiency = self.calculate_fuel_efficiency(action)
        
        # Penalties
        safety_penalty = self.calculate_safety_penalty(state, next_state)
        
        total_reward = (
            self.reward_weights['waypoint_progress'] * waypoint_progress +
            self.reward_weights['path_efficiency'] * path_efficiency +
            self.reward_weights['control_smoothness'] * control_smoothness +
            self.reward_weights['tracking_accuracy'] * tracking_accuracy +
            self.reward_weights['comfort'] * comfort_metric +
            self.reward_weights['fuel_efficiency'] * fuel_efficiency -
            safety_penalty
        )
        
        return total_reward
    
    def calculate_waypoint_progress_rate(self, state, next_state):
        """Reward for efficient progress toward waypoints"""
        distance_reduction = state['distance_to_waypoint'] - next_state['distance_to_waypoint']
        return np.tanh(distance_reduction * 2.0)  # Normalized progress bonus
    
    def calculate_path_efficiency(self, state, next_state):
        """Reward for maintaining efficient track to waypoint"""
        cross_track_penalty = -abs(state['cross_track_error']) * 0.1
        return cross_track_penalty
    
    def calculate_control_smoothness(self, action):
        """Reward for smooth control inputs"""
        if hasattr(self, 'previous_action'):
            control_change = np.linalg.norm(action - self.previous_action)
            smoothness_reward = -control_change * 0.5
        else:
            smoothness_reward = 0.0
        
        self.previous_action = action
        return smoothness_reward
```

## Phase 4: RL Training Pipeline

### 4.1 ROS-Gymnasium Environment Wrapper

```python
class MarineRLEnvironment(gym.Env):
    def __init__(self):
        super(MarineRLEnvironment, self).__init__()
        
        # Initialize ROS node
        rospy.init_node('marine_rl_environment')
        
        # Define spaces
        self.observation_space = MarineObservationSpace().observation_space
        self.action_space = MarineActionSpace().action_space
        
        # ROS Publishers/Subscribers
        self.setup_ros_interface()
        
        # Environment state
        self.observation_space = MarineObservationSpace()
        self.reset()
    
    def step(self, action):
        """Execute one RL step"""
        # Send action to vessel
        self.publish_action(action)
        
        # Wait for environment response
        rospy.sleep(0.2)  # 5 Hz control rate
        
        # Get new observation
        observation = self.get_observation()
        
        # Update control history with current action and performance
        current_tracking_error = self.calculate_current_tracking_error()
        self.observation_space.update_control_history(
            action[0],  # thrust command
            action[1],  # rudder command  
            current_tracking_error
        )
        
        # Calculate reward
        reward = self.reward_function.calculate_reward(
            self.previous_observation, action, observation
        )
        
        # Check if episode is done
        done = self.check_episode_termination()
        
        # Additional info
        info = self.get_step_info()
        
        self.previous_observation = observation
        return observation, reward, done, info
    
    def reset(self):
        """Reset environment to random initial state"""
        # Reset vessel position and waypoint
        self.reset_vessel_state()
        
        # Reset environmental conditions
        self.randomize_environment()
        
        # Wait for stabilization
        rospy.sleep(1.0)
        
        # Get initial observation
        observation = self.get_observation()
        self.previous_observation = observation
        
        # Reset control history
        self.observation_space.recent_thrust_commands.clear()
        self.observation_space.recent_rudder_commands.clear() 
        self.observation_space.recent_tracking_errors.clear()
        
        # Fill with zeros
        for _ in range(self.observation_space.history_window_size):
            self.observation_space.recent_thrust_commands.append(0.0)
            self.observation_space.recent_rudder_commands.append(0.0)
            self.observation_space.recent_tracking_errors.append(0.0)
        
        return observation
    
    def calculate_current_tracking_error(self):
        """Calculate current tracking error for history buffer"""
        # Get current navigation state
        distance_to_waypoint = self.get_distance_to_waypoint()
        cross_track_error = self.get_cross_track_error()
        heading_error = self.get_heading_error()
        
        # Combined tracking error metric
        tracking_error = np.sqrt(
            (cross_track_error ** 2) + 
            (heading_error ** 2) + 
            (distance_to_waypoint * 0.1) ** 2  # Scale distance component
        )
        
        return tracking_error
```

### 4.2 Training Configuration

#### PPO Training Setup
```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

def train_marine_rl():
    # Create environment
    env = MarineRLEnvironment()
    check_env(env)  # Validate environment
    
    # PPO Configuration
    model = PPO(
        "MultiInputPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        verbose=1,
        tensorboard_log="./marine_rl_tensorboard/"
    )
    
    # Training curriculum
    total_timesteps = 1000000
    
    # Stage 1: Calm conditions (200k steps)
    env.set_difficulty_level('calm')
    model.learn(total_timesteps=200000)
    
    # Stage 2: Moderate conditions (400k steps)  
    env.set_difficulty_level('moderate')
    model.learn(total_timesteps=400000)
    
    # Stage 3: Rough conditions (400k steps)
    env.set_difficulty_level('rough')
    model.learn(total_timesteps=400000)
    
    # Save trained model
    model.save("marine_rl_final_model")
    
    return model
```

### 4.3 Curriculum Learning Strategy

```python
class EnvironmentalCurriculum:
    def __init__(self):
        self.difficulty_levels = {
            'calm': {
                'wave_height': (0.0, 0.5),
                'wind_speed': (0, 5),
                'current_strength': (0, 0.2)
            },
            'moderate': {
                'wave_height': (0.5, 2.0), 
                'wind_speed': (5, 15),
                'current_strength': (0.2, 1.0)
            },
            'rough': {
                'wave_height': (2.0, 4.0),
                'wind_speed': (15, 30),
                'current_strength': (1.0, 2.0)
            }
        }
    
    def set_environmental_conditions(self, difficulty):
        """Configure Gazebo environment for training stage"""
        conditions = self.difficulty_levels[difficulty]
        
        # Set wave parameters
        self.set_wave_conditions(conditions['wave_height'])
        
        # Set wind conditions  
        self.set_wind_conditions(conditions['wind_speed'])
        
        # Set current conditions
        self.set_current_conditions(conditions['current_strength'])
```

## Phase 5: Integration & Deployment

### 5.1 Real-Time ROS Node Architecture

```python
class MarineRLController:
    def __init__(self):
        rospy.init_node('marine_rl_controller')
        
        # Load trained model
        self.model = PPO.load("marine_rl_final_model")
        
        # Initialize observation management
        self.obs_manager = ObservationManager()
        self.safety_monitor = SafetyMonitor()
        
        # ROS Interface
        self.setup_ros_interface()
        
        # Control loop timer
        self.control_timer = rospy.Timer(rospy.Duration(0.2), self.control_callback)
    
    def control_callback(self, event):
        """Main control loop - 5 Hz"""
        try:
            # Get current observation
            observation = self.obs_manager.get_current_observation()
            
            # Safety check
            if not self.safety_monitor.is_safe(observation):
                self.use_baseline_control()
                return
            
            # RL inference
            action, _ = self.model.predict(observation, deterministic=True)
            
            # Publish RL commands
            self.publish_rl_commands(action)
            
            # Log performance
            self.log_performance(observation, action)
            
        except Exception as e:
            rospy.logerr(f"RL Controller error: {e}")
            self.use_baseline_control()
    
    def use_baseline_control(self):
        """Fallback to baseline MOOS PID"""
        # Republish MOOS commands as RL commands
        thrust_msg = Float64()
        rudder_msg = Float64()
        
        thrust_msg.data = self.obs_manager.moos_desired_thrust
        rudder_msg.data = self.obs_manager.moos_desired_rudder
        
        self.rl_thrust_pub.publish(thrust_msg)
        self.rl_rudder_pub.publish(rudder_msg)
```

### 5.2 Safety & Monitoring System

```python
class SafetyMonitor:
    def __init__(self):
        self.safety_limits = {
            'max_cross_track_error': 50.0,  # meters
            'max_control_rate': 0.5,        # per second
            'observation_timeout': 2.0,      # seconds
            'performance_threshold': -10.0   # reward threshold
        }
        
        self.performance_history = deque(maxlen=100)
    
    def is_safe(self, observation):
        """Comprehensive safety check"""
        # Check observation freshness
        if not self.check_observation_fresh():
            return False
        
        # Check tracking performance
        if abs(observation['navigation'][2]) > self.safety_limits['max_cross_track_error']:
            return False
        
        # Check RL performance history
        if len(self.performance_history) > 50:
            recent_performance = np.mean(list(self.performance_history)[-20:])
            if recent_performance < self.safety_limits['performance_threshold']:
                return False
        
        return True
```

## Phase 6: Performance Evaluation & Validation

### 6.1 Evaluation Metrics

```python
class PerformanceEvaluator:
    def __init__(self):
        self.metrics = {
            'navigation_efficiency': [],
            'waypoint_accuracy': [],
            'control_smoothness': [],
            'fuel_consumption': [],
            'passenger_comfort': [],
            'environmental_adaptation': []
        }
    
    def evaluate_mission(self, mission_data):
        """Comprehensive mission performance evaluation"""
        results = {}
        
        # Navigation Efficiency
        optimal_distance = self.calculate_optimal_path_distance(mission_data['waypoints'])
        actual_distance = self.calculate_actual_distance(mission_data['trajectory'])
        results['navigation_efficiency'] = optimal_distance / actual_distance
        
        # Waypoint Accuracy
        waypoint_errors = self.calculate_waypoint_errors(mission_data)
        results['waypoint_accuracy'] = np.mean(waypoint_errors)
        
        # Control Smoothness
        control_variations = self.calculate_control_variations(mission_data['commands'])
        results['control_smoothness'] = 1.0 / (1.0 + np.std(control_variations))
        
        # Environmental Adaptation
        adaptation_score = self.evaluate_environmental_response(mission_data)
        results['environmental_adaptation'] = adaptation_score
        
        return results
```

### 6.2 Comparison Framework

```python
def compare_baseline_vs_rl():
    """Compare baseline PID vs RL-enhanced control"""
    test_scenarios = [
        'calm_straight_line',
        'moderate_seas_curved_path', 
        'rough_seas_complex_waypoints',
        'strong_current_navigation',
        'gusty_wind_conditions'
    ]
    
    results = {}
    
    for scenario in test_scenarios:
        # Run baseline PID
        baseline_results = run_scenario_baseline(scenario)
        
        # Run RL-enhanced
        rl_results = run_scenario_rl_enhanced(scenario)
        
        # Calculate improvements
        results[scenario] = {
            'time_improvement': (baseline_results['mission_time'] - rl_results['mission_time']) / baseline_results['mission_time'],
            'fuel_savings': (baseline_results['fuel_consumption'] - rl_results['fuel_consumption']) / baseline_results['fuel_consumption'],
            'comfort_improvement': rl_results['comfort_score'] - baseline_results['comfort_score'],
            'tracking_improvement': baseline_results['tracking_error'] - rl_results['tracking_error']
        }
    
    return results
```

## Deployment Checklist

### Pre-Deployment Validation
- [ ] Gazebo simulation validation across all environmental conditions
- [ ] MOOS-ROS bridge functionality verified
- [ ] Safety systems tested and validated
- [ ] Performance improvements demonstrated in simulation
- [ ] Fallback to baseline PID verified

### Real-World Integration
- [ ] Limited authority testing (RL commands scaled by safety factor)
- [ ] Gradual increase in RL authority based on performance
- [ ] Continuous monitoring and data collection
- [ ] Regular model updates based on real-world experience

### Success Metrics
- [ ] 10%+ improvement in mission efficiency
- [ ] 15%+ reduction in control effort
- [ ] 20%+ improvement in passenger comfort scores
- [ ] Robust performance across varying sea states
- [ ] Zero safety incidents during deployment

## Conclusion

This residual RL approach leverages proven PID control while learning to enhance performance in challenging marine environments. The system learns when to trust baseline commands and when environmental adaptation is needed, providing a robust and practical solution for real-world marine navigation.