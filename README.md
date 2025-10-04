# Advanced Automatic Control Systems Projects (2023-24)

## Course Information
- **Course**: "Advanced Techniques in Automatic Control Systems"
- **Academic Year**: 2023-24
- **Institution**: National Technical University of Athens (NTUA)
- **Department**: School of Electrical and Computer Engineering
- **Specialization**: Signals, Control and Robotics

## Laboratory Exercises

### Exercise 1: Boost Converter State-Space Control

#### Project Overview
Design and implementation of state-space controllers for a DC-DC boost converter circuit. The project focuses on modeling, linearization, and control design for power electronics applications.

#### Key Components
- **System Modeling**: Derivation of state-space equations for boost converter operation
- **Equilibrium Analysis**: Calculation of operating points for different duty cycles
- **Linearization**: Development of linearized models around operating points
- **Control Design**: Implementation of integral controllers and LQR control
- **Performance Analysis**: Simulation under varying input voltage conditions (7-12V)

#### Technical Specifications
- **Input Voltage**: 9V nominal (7-12V range)
- **Components**: L = 300 µH, C = 70 µF, R = 10 Ω
- **Target Output**: 22.5V
- **Control Methods**: Pole placement, Linear Quadratic Regulator (LQR)

### Exercise 2: Quadcopter Drone Control

#### Project Overview
Comprehensive control system design for a quadcopter drone, implementing both classical and modern control techniques for aerial vehicle stabilization and trajectory tracking.

#### Control Approaches

##### Cascaded PID Control
- **Attitude Control**: PID controllers for roll (φ), pitch (θ) angles
- **Altitude Control**: Height regulation with reference tracking
- **Position Control**: Cascaded PID for x, y, z coordinate tracking
- **Orientation Control**: Optional yaw (ψ) angle control

##### State-Space Control
- **Equilibrium Analysis**: Calculation of system equilibrium points
- **Linearization**: Development of linearized models around hover conditions
- **LQR Control**: Optimal state feedback control with Q, R matrix tuning
- **Trajectory Tracking**: Waypoint following capability implementation

#### System Parameters
- **Mass**: 0.800 kg
- **Arm Length**: 0.3 m
- **Moments of Inertia**: Ixx = 15.67×10⁻³, Iyy = 15.67×10⁻³, Izz = 28.34×10⁻³ kg/m²
- **Thrust Factor**: 192.32×10⁻⁷ Ns²
- **Maximum RPM**: 15,000

### Exercise 3: State Estimation Techniques

#### Project Overview
Implementation and comparison of advanced state estimation algorithms for nonlinear systems, with applications in both engineering systems and epidemiological modeling.

#### Estimation Methods

##### 1D System Estimation
- **Bayesian Filtering**: Numerical integration-based recursive estimation
- **Extended Kalman Filter (EKF)**: Linearization-based approximation
- **Particle Filtering**: Sequential Monte Carlo methods with resampling
- **Performance Comparison**: MSE analysis across different estimators

##### Epidemiological Modeling
- **Compartmental Model**: SIR/SEIR model implementation (Susceptible, Exposed, Infected, Recovered)
- **Particle Filter Application**: State estimation for disease spread dynamics
- **Forecasting**: 10-step ahead prediction capability
- **Stochastic Modeling**: Log-normal distributed process noise

#### System Characteristics
- **Nonlinear Dynamics**: Complex 1D system with trigonometric and polynomial terms
- **Measurement Models**: Noisy observations with varying parameters
- **Epidemiological Parameters**: Transmission rates, recovery rates, population dynamics

### Exercise 4: Optimal Control of Electric Train

#### Project Overview
Optimal control design for electric train operation, focusing on energy minimization while maintaining performance constraints and trajectory tracking.

#### Control Framework

##### Open-Loop Optimal Control
- **Minimum Time Problems**: Bang-bang control implementation
- **Quadratic Cost Minimization**: LQR-type problem formulation
- **Boundary Value Problems**: Numerical solution methods

##### Closed-Loop Implementation
- **System Linearization**: Development around optimal trajectory
- **Feedback Control Design**: Perturbation control law derivation
- **Robustness Analysis**: Performance under initial condition variations
- **Dynamic Programming**: Alternative solution approach

#### Train Model Parameters
- **Dynamics**: Second-order system with velocity-dependent friction
- **Electrical System**: DC motor with regenerative braking capability
- **Constraints**: Current limits (-2A to 2A)
- **Performance Criteria**: Position accuracy, velocity regulation, energy consumption

## Technical Skills Developed

### Theoretical Foundations
- State-space modeling and analysis
- Nonlinear system linearization
- Optimal control theory (LQR, minimum time)
- Stochastic estimation theory
- Numerical methods for control problems

### Implementation Capabilities
- MATLAB/Simulink programming
- Control system design and simulation
- State estimation algorithm implementation
- Performance analysis and validation
- Real-world application to engineering systems

### Applications
- Power electronics control
- Aerial vehicle stabilization
- Epidemiological forecasting
- Transportation system optimization
- Stochastic system estimation

## References
- Bryson and Ho "Applied Optimal Control"
- Technical documentation for boost converters and quadcopter dynamics
- Research papers on train control optimization
- Epidemiological compartmental models
- Kalman filtering and particle filtering literature