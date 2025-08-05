def sysCall_init():
    sim = require('sim')

    # Initialization
    global right_joint, left_joint, body, arm_joint, prismatic_joint, Kp, Ki, Kd, integral_error, previous_error, max_velocity, desired_velocity, turn_velocity, fall_threshold, is_braking, manipulator_mass

    try:
        # Get object handles
        right_joint = sim.getObject('/right_joint')
        left_joint = sim.getObject('/left_joint')
        body = sim.getObject('/body')
        arm_joint = sim.getObject('/arm_joint')  # Arm for lifting
        prismatic_joint = sim.getObject('/Prismatic_joint')  # Gripper for holding
        sim.addLog(sim.verbosity_scriptinfos, "Handles initialized successfully.")
    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"Initialization error: {str(e)}")
        return

    # PID control parameters for balancing
    Kp = 500.0   # Reduced proportional gain
    Ki = 10.0    # Reduced integral gain
    Kd = 100.0   # Reduced derivative gain
    max_velocity = 10.0  # Max wheel velocity for smoother motion
    desired_velocity = -20.0  # Start at forward speed (positive value)
    turn_velocity = 0.0  # No turn initially
    fall_threshold = 0.4  # Tilt threshold for correction
    is_braking = False  # Brake state

    # Initialize PID error terms
    integral_error = 0.0
    previous_error = 0.0

    # Mass of the manipulator
    manipulator_mass = 0.08  # kg

    sim.addLog(sim.verbosity_scriptinfos, "PID parameters and state initialized.")

def sysCall_actuation():
    global integral_error, previous_error, right_joint, left_joint, body, Kp, Ki, Kd, max_velocity, desired_velocity, turn_velocity, fall_threshold, is_braking, manipulator_mass

    try:
        # Get the robot's current tilt angle (balance state)
        body_angle = sim.getObjectOrientation(body,-1)[0]  # Tilt angle along Y-axis (pitch)
    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"Error getting body orientation: {str(e)}")
        return

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

    # Adjust control signal for manipulator mass
    adjusted_velocity = control_signal * (1 + (manipulator_mass / (0.248 + manipulator_mass)))

    # Smooth forward and turning velocities
    left_velocity = desired_velocity + turn_velocity  # Left wheel velocity
    right_velocity = desired_velocity - turn_velocity  # Right wheel velocity

    # Smooth transition of velocities by limiting changes
    left_velocity = max(min(left_velocity + adjusted_velocity, max_velocity), -max_velocity)
    right_velocity = max(min(right_velocity + adjusted_velocity, max_velocity), -max_velocity)

    # Apply maximum constraints to avoid oscillations
    left_velocity = max(min(left_velocity, max_velocity), -max_velocity)
    right_velocity = max(min(right_velocity, max_velocity), -max_velocity)

    # Apply velocities to the joints (wheel speeds)
    try:
        sim.setJointTargetVelocity(left_joint, left_velocity)
        sim.setJointTargetVelocity(right_joint, right_velocity)
    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"Error setting joint velocities: {str(e)}")
        return

    # Update previous error for the next PID cycle
    previous_error = error

    sim.addLog(sim.verbosity_scriptinfos, "Actuation completed for this cycle.")

def sysCall_sensing():
    global desired_velocity, turn_velocity, is_braking

    try:
        # Capture key press events
        message, data, _ = sim.getSimulatorMessage()
    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"Error reading simulator message: {str(e)}")
        return

    if message == sim.message_keypress:
        if data[0] == 2007:  # Up Arrow - Move Forward
            desired_velocity = 4.0  # Move forward at moderate speed
            turn_velocity = 0.0  # No turn initially
            is_braking = False
        elif data[0] == 2008:  # Down Arrow - Move Backward
            desired_velocity = -4.0  # Move backward (reverse)
            turn_velocity = 0.0  # No turn initially
            is_braking = False
        elif data[0] == 2010:  # Right Arrow - Turn Right (forward)
            turn_velocity = 10.0  # Moderate right turn speed
            desired_velocity = 4.0  # Move forward
            is_braking = False
        elif data[0] == 2009:  # Left Arrow - Turn Left (forward)
            turn_velocity = -10.0  # Moderate left turn speed
            desired_velocity = 4.0  # Move forward
            is_braking = False
        elif data[0] == 113:  # 'q' - Close Gripper
            sim.setJointTargetVelocity(prismatic_joint, -0.04)
        elif data[0] == 101:  # 'e' - Open Gripper
            sim.setJointTargetVelocity(prismatic_joint, 0.04)
        elif data[0] == 115:  # 'w' - Raise Arm
            sim.setJointTargetVelocity(arm_joint, 1.0)
        elif data[0] == 119:  # 's' - Lower Arm
            sim.setJointTargetVelocity(arm_joint, -1.0)
        elif data[0] == 32:  # Spacebar - Apply Brake
            desired_velocity = 0.0
            turn_velocity = 0.0
            is_braking = True
        elif data[0] == 121:  # 'y' - Sharp Left Turn (forward)
            turn_velocity = -30.0  # Sharp left turn speed while moving forward
            desired_velocity = 4.0  # Move forward
            is_braking = False
        elif data[0] == 117:  # 'u' - Sharp Right Turn (forward)
            turn_velocity = 30.0  # Sharp right turn speed while moving forward
            desired_velocity = 4.0  # Move forward
            is_braking = False
    else:
        # Stop motion if no key is pressed
        desired_velocity = 0.0
        turn_velocity = 0.0
        is_braking = False

    sim.addLog(sim.verbosity_scriptinfos, "Key press sensing processed.")

def sysCall_cleanup():
    sim.addLog(sim.verbosity_scriptinfos, "Cleanup completed successfully.")