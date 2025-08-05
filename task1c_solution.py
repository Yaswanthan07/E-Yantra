def sysCall_init():
    sim = require('sim')

    # Initialization
    global right_joint, left_joint, body, Kp, Ki, Kd, integral_error, previous_error, max_velocity, desired_velocity, turn_velocity, fall_threshold, is_braking

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
    desired_velocity = -4.0  # Default forward velocity
    turn_velocity = 0.1  # Default turning speed
    fall_threshold = 0.2  # Tilt threshold before aggressive correction
    is_braking = False  # Brake state

    # Initialize PID error terms
    integral_error = 0.0
    previous_error = 0.0

def sysCall_actuation():
    global integral_error, previous_error, right_joint, left_joint, body, Kp, Ki, Kd, max_velocity, desired_velocity, turn_velocity, fall_threshold, is_braking

    # Get the robot's current tilt angle (balance state)
    body_angle = sim.getObjectOrientation(body, -1)[1]  # Tilt angle along the Y-axis (pitch)

    # Desired angle for balance (upright = 0 angle)
    desired_angle = 0.0

    # Calculate PID control error
    error = desired_angle - body_angle
    integral_error += error
    derivative_error = error - previous_error

    # PID control for balancing
    control_signal = Kp * error + Ki * integral_error + Kd * derivative_error

    # Apply maximum correction if tilt exceeds the fall threshold
    if abs(body_angle) > fall_threshold:
        control_signal = max_velocity if body_angle > 0 else -max_velocity

    # Apply velocity for circular motion and direction control, or stop if braking
    if is_braking:
        left_velocity = 0.0  # Apply brake (stop)
        right_velocity = 0.0  # Apply brake (stop)
    else:
        left_velocity = desired_velocity + turn_velocity  # Left wheel velocity
        right_velocity = desired_velocity - turn_velocity  # Right wheel velocity

    # Saturate the control signal to the maximum allowed velocity
    left_velocity = max(min(left_velocity + control_signal, max_velocity), -max_velocity)
    right_velocity = max(min(right_velocity + control_signal, max_velocity), -max_velocity)

    # Apply velocities to the joints (wheel speeds)
    sim.setJointTargetVelocity(left_joint, left_velocity)
    sim.setJointTargetVelocity(right_joint, right_velocity)

    # Update previous error for the next PID cycle
    previous_error = error

def sysCall_sensing():
    global desired_velocity, turn_velocity, is_braking

    # Capture key press events
    message, data, _ = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow (move forward)
            desired_velocity = 4.0  # Set forward velocity
            turn_velocity = 0.0
            is_braking = False  # Release brake
        elif data[0] == 2008:  # Down arrow (move backward)
            desired_velocity = -4.0  # Set backward velocity
            turn_velocity = 0.0
            is_braking = False  # Release brake
        elif data[0] == 2009:  # Left arrow (turn left)
            turn_velocity = 38.0  # Positive value for left turn
            desired_velocity = 4.0  # Keep moving forward while turning
            is_braking = False  # Release brake
        elif data[0] == 2010:  # Right arrow (turn right)
            turn_velocity = -38.0  # Negative value for right turn
            desired_velocity = 4.0  # Keep moving forward while turning
            is_braking = False  # Release brake
        elif data[0] == 32:  # Spacebar (apply brake)
            is_braking = True  # Engage brake (stop motion)
        else:
            desired_velocity = 0.0  # No key pressed, stop movement
            turn_velocity = 0.0
            is_braking = False  # Release brake

def sysCall_cleanup():
    # Cleanup logic
    pass
