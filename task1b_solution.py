def sysCall_init():
    sim = require('sim')

    # Initialization
    global right_joint, left_joint, body, Kp, Ki, Kd, integral_error, previous_error, max_velocity, desired_velocity, fall_threshold

    try:
        right_joint = sim.getObject('/right_joint')
        left_joint = sim.getObject('/left_joint')
        body = sim.getObject('/body')
    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"Initialization error: {str(e)}")
        return

    # PID control parameters for balancing
    Kp = 9000.0   # Starting proportional gain
    Ki = 8000.0   # Starting integral gain
    Kd = 8500.0   # Starting derivative gain
    max_velocity = 10.0  # Max wheel velocity
    desired_velocity = 4.0  # Forward velocity
    fall_threshold = 0.2  # Tilt threshold before aggressive correction

    # Initialize error terms for the PID controller
    integral_error = 0.0
    previous_error = 0.0

def sysCall_actuation():
    global integral_error, previous_error, right_joint, left_joint, body, Kp, Ki, Kd, max_velocity, desired_velocity, fall_threshold

    # Get the robot's current tilt angle (balance state)
    body_angle = sim.getObjectOrientation(body, -1)[1]  # Tilt angle along the Y-axis (pitch)

    # Desired angle for balance (upright = 0 angle)
    desired_angle = 0.0

    # Calculate error (how much the robot is tilted)
    error = desired_angle - body_angle
    integral_error += error
    derivative_error = error - previous_error

    # PID control for balancing
    control_signal = Kp * error + Ki * integral_error + Kd * derivative_error

    # Apply maximum correction if tilt exceeds the fall threshold
    if abs(body_angle) > fall_threshold:
        control_signal = max_velocity if body_angle > 0 else -max_velocity

    # Set velocities for circular motion
    # Make one wheel slightly faster than the other to create a circular motion
    left_velocity = desired_velocity  # Left wheel moves at normal speed
    right_velocity = desired_velocity * 0.6  # Right wheel moves slower to turn right in a circle

    # Saturate the control signal to the maximum allowed velocity
    left_velocity = max(min(left_velocity + control_signal, max_velocity), -max_velocity)
    right_velocity = max(min(right_velocity + control_signal, max_velocity), -max_velocity)

    # Apply velocities to the joints (wheel speeds)
    sim.setJointTargetVelocity(left_joint, left_velocity)
    sim.setJointTargetVelocity(right_joint, right_velocity)

    # Update previous error for the next control loop
    previous_error = error

def sysCall_sensing():
    # Optional sensing logic
    pass

def sysCall_cleanup():
    # Cleanup logic
    pass
