# Best Practices for Physics Parameter Tuning in Gazebo

## Introduction to Parameter Tuning

Physics parameter tuning in Gazebo is a critical skill for creating realistic and stable simulations. Properly tuned parameters ensure that simulated robots behave similarly to their real-world counterparts, enabling effective algorithm development and testing. This chapter provides comprehensive guidance on how to approach physics parameter tuning systematically.

## Understanding the Physics Pipeline

### Simulation Loop

Gazebo operates on a discrete simulation loop with the following steps:
1. **Update Sensors**: Collect sensor data from the current state
2. **Process Actuators**: Apply forces/torques based on control inputs
3. **Physics Step**: Integrate physics equations for a fixed timestep
4. **Update Graphics**: Render the new state (if visualization is enabled)

Understanding this pipeline is crucial for effective parameter tuning.

### Key Parameters

The most important parameters affecting simulation behavior include:
- **Simulation Timestep**: The time interval between physics updates
- **Real-time Factor**: How fast simulation time progresses relative to real time
- **Solver Iterations**: Number of iterations for constraint solving
- **Contact Parameters**: Stiffness, damping, and friction coefficients
- **ERP and CFM**: Error reduction and constraint force mixing parameters

## Systematic Tuning Approach

### Phase 1: Basic Stability

Start with ensuring basic simulation stability:

#### Timestep Selection
- **Default**: 0.001 seconds (1ms) is often a good starting point
- **For high-precision**: Use smaller timesteps (0.0001s)
- **For performance**: Larger timesteps (0.01s) but with reduced accuracy
- **Rule of thumb**: Timestep should be 1/100th or smaller of the fastest dynamic response in your system

#### Solver Configuration
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>  <!-- or "iterative" -->
      <iters>100</iters>  <!-- Start with 100, adjust as needed -->
      <sor>1.3</sor>      <!-- Successive over-relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>      <!-- Constraint Force Mixing -->
      <erp>0.2</erp>      <!-- Error Reduction Parameter -->
    </constraints>
  </ode>
</physics>
```

### Phase 2: Contact Parameter Tuning

#### Contact Stiffness and Damping
- **kp (Spring stiffness)**: Higher values make contacts more rigid but can cause instability
- **kd (Damping coefficient)**: Higher values reduce oscillations but may cause sluggish response
- **Start with**: kp=1e+7, kd=10, adjust based on behavior

#### Friction Parameters
- **mu (Primary friction)**: Static friction coefficient (0.0-10.0 typical range)
- **mu2 (Secondary friction)**: Friction in the second direction
- **fdir1**: Direction of primary friction (for anisotropic friction)

### Phase 3: Robot-Specific Tuning

#### Mass and Inertia Properties
- Ensure accurate mass properties from CAD models or measurements
- Verify inertia tensors are physically valid (positive and satisfy triangle inequality)
- Use consistent units (SI: kg, m, kg⋅m²)

#### Joint Properties
- **Damping**: Add realistic joint damping to match real-world behavior
- **Friction**: Include static and dynamic friction if applicable
- **Limits**: Set appropriate position, velocity, and effort limits

## Tuning Strategies by Robot Type

### Wheeled Robots

For wheeled robots, focus on wheel-ground interaction:

```xml
<collision name="wheel_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>    <!-- High friction for good traction -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>10000000</kp>  <!-- High stiffness for stable contact -->
        <kd>100</kd>        <!-- Adequate damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

#### Key Considerations:
- Wheel slip modeling for realistic traction
- Surface-dependent friction coefficients
- Proper wheel collision geometry

### Legged Robots

For legged robots, especially humanoids, stability is paramount:

```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>0.7</mu>    <!-- Moderate friction for stable walking -->
        <mu2>0.7</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>100000000</kp>  <!-- Very high stiffness for stable stance -->
        <kd>1000</kd>        <!-- High damping to prevent oscillations -->
      </ode>
    </contact>
  </surface>
</collision>
```

#### Key Considerations:
- Foot-ground contact stability
- Balance control parameter adjustment
- Impact modeling for foot strikes

### Manipulation Robots

For robotic arms and manipulators:

```xml
<collision name="end_effector_collision">
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu>    <!-- Moderate friction for manipulation -->
        <mu2>0.5</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>5000000</kp>  <!-- Moderate stiffness for safe manipulation -->
        <kd>50</kd>        <!-- Moderate damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

#### Key Considerations:
- Grasp stability and object handling
- Force control parameter tuning
- Collision detection for safe operation

## Validation Techniques

### Quantitative Validation

Compare simulation results with real-world data:

#### Position and Velocity Tracking
- Track end-effector positions in both simulation and reality
- Compare joint angle trajectories
- Validate velocity profiles

#### Force Measurements
- Use force/torque sensors in simulation and reality
- Compare contact forces during manipulation
- Validate ground reaction forces for legged robots

### Qualitative Validation

#### Visual Comparison
- Record side-by-side videos of simulation and reality
- Compare overall behavior patterns
- Validate that failure modes are similar

#### Behavioral Validation
- Test the same control algorithms in both domains
- Compare success rates for specific tasks
- Validate that system responses are qualitatively similar

## Common Tuning Scenarios

### Scenario 1: Robot Falling Through Ground

**Symptoms**: Robot falls through floor or other static objects

**Solutions**:
1. Check collision geometry exists for all links
2. Increase contact stiffness (kp parameter)
3. Verify mass properties are positive and reasonable
4. Check that `<static>true</static>` is set for ground plane

### Scenario 2: Excessive Jittering

**Symptoms**: Robot or objects vibrate or jitter during contact

**Solutions**:
1. Increase damping (kd parameter)
2. Reduce timestep if possible
3. Increase solver iterations
4. Adjust ERP/CFM parameters (higher ERP, lower CFM)

### Scenario 3: Unstable Simulation

**Symptoms**: Simulation "explodes" with objects flying apart

**Solutions**:
1. Reduce timestep significantly
2. Increase solver iterations
3. Reduce contact stiffness
4. Check for coincident geometry or invalid inertia tensors

### Scenario 4: Penetration Issues

**Symptoms**: Objects pass through each other or sink into surfaces

**Solutions**:
1. Increase contact stiffness (kp)
2. Increase ERP value
3. Use more accurate collision geometry
4. Reduce timestep

## Performance Optimization

### Balancing Accuracy and Speed

#### Real-time Factor Optimization
- **Target**: Real-time factor (RTF) of 1.0 for real-time simulation
- **Performance**: RTF > 1.0 means simulation runs faster than real-time
- **Adjustment**: If RTF &lt;&lt; 1.0, consider reducing accuracy parameters

#### Solver Parameter Optimization
- Start with conservative parameters
- Gradually reduce solver iterations until instability occurs
- Use the highest iteration count that maintains stability and performance

### Computational Efficiency

#### Geometry Simplification
- Use simplified collision geometry where high fidelity isn't needed
- Approximate complex shapes with primitive shapes
- Use mesh collision only where necessary

#### Update Rate Management
- Match sensor update rates to real hardware capabilities
- Use lower update rates for less critical sensors
- Consider variable update rates based on simulation state

## Advanced Tuning Techniques

### Adaptive Parameter Adjustment

For complex scenarios, consider adaptive parameter adjustment:

```cpp
// Example of adaptive parameter adjustment in a Gazebo plugin
class AdaptiveTuningPlugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world = _world;
    // Adjust parameters based on simulation state
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AdaptiveTuningPlugin::OnUpdate, this));
  }

  public: void OnUpdate()
  {
    // Example: Increase solver iterations during high-contact scenarios
    if (this->world->GetPhysicsEngine()->GetContacts().size() > 10)
    {
      this->world->GetPhysicsEngine()->SetParam("iters", 200);
    }
    else
    {
      this->world->GetPhysicsEngine()->SetParam("iters", 50);
    }
  }
};
```

### System Identification

Use system identification techniques to determine optimal parameters:

1. **Excite the System**: Apply known inputs to both simulation and reality
2. **Measure Response**: Collect output data from both systems
3. **Optimize Parameters**: Use optimization algorithms to minimize response differences
4. **Validate**: Test optimized parameters with different inputs

## Documentation and Reproducibility

### Parameter Documentation

Maintain detailed records of tuned parameters:

```markdown
# Physics Parameter Documentation

## Simulation Configuration
- Timestep: 0.001s
- Real-time update rate: 1000 Hz
- Solver: ODE Quick, 100 iterations

## Contact Parameters
- Ground-robot contact stiffness: 1e+8
- Ground-robot damping: 1000
- Ground friction: 0.8

## Validation Results
- Position tracking error: < 2cm
- Task success rate: 95% (vs 92% real robot)
```

### Version Control

- Store physics configurations in version control
- Document the Gazebo version used for tuning
- Track parameter changes and their effects

## Troubleshooting Checklist

Before finalizing parameter tuning, verify:

- [ ] Simulation is stable over long periods
- [ ] Robot behavior matches expectations qualitatively
- [ ] Quantitative metrics align with real-world data
- [ ] Performance meets requirements (RTF > 0.8)
- [ ] All joints and links have appropriate properties
- [ ] Collision detection works properly
- [ ] No objects fall through surfaces
- [ ] Contacts behave realistically

## Best Practices Summary

1. **Start Conservative**: Begin with stable parameters and optimize gradually
2. **Validate Quantitatively**: Use real-world data for validation when possible
3. **Document Everything**: Keep detailed records of parameter choices and rationale
4. **Iterate Systematically**: Make one change at a time and measure effects
5. **Consider Robot Type**: Different robot types require different tuning approaches
6. **Balance Performance**: Optimize for both accuracy and computational efficiency
7. **Test Extensively**: Validate with multiple scenarios and control algorithms

Proper physics parameter tuning is essential for creating useful and realistic robotic simulations. By following these best practices, you can create simulations that effectively support robotics development and validation.