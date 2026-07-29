import math
import os
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("PID-BASED PATH-TRACKING MODULE STARTED")

# Define Data Directory
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
os.makedirs(DATA_DIR, exist_ok=True)

KP, KI, KD = 2.5, 0.15, 1.0
integral_error, last_error = 0.0, 0.0
dt = timestep / 1000.0  

gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
receiver = robot.getDevice("receiver")
gps.enable(timestep); compass.enable(timestep); receiver.enable(timestep)

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf')); rightMotor.setPosition(float('inf'))

CMD_X, CMD_Y = -1.2, 1.2
TARGET_X, TARGET_Y = -1.2, 1.2
BASE_SPEED = 4.0
GLOBAL_GOAL_X, GLOBAL_GOAL_Y = 1.39, -1.4
emergency_stop = False

# Initialize active_alpha as None until supervisor packet arrives
active_alpha = None
current_log_file = None
current_csv_path = None

start_sim_time = robot.getTime()

while robot.step(timestep) != -1:
    sim_time = robot.getTime() - start_sim_time

    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        parts = message.split()
        if parts[0] == "STOP": 
            emergency_stop = True
            if len(parts) > 1: active_alpha = float(parts[1])
        else:
            emergency_stop = False
            try:
                CMD_X, CMD_Y = float(parts[0]), float(parts[1])
                if len(parts) > 2: active_alpha = float(parts[2])
            except: pass

    # Open tracking file in write ("w") mode ONCE after receiving alpha packet
    if active_alpha is not None and current_log_file is None:
        current_csv_path = os.path.join(DATA_DIR, f"tracking_alpha_{active_alpha}.csv")
        current_log_file = open(current_csv_path, "w")
        current_log_file.write("time_s,reference_heading_deg,actual_heading_deg,heading_error_deg\n")
        current_log_file.flush()

    jump = math.sqrt((CMD_X - TARGET_X)**2 + (CMD_Y - TARGET_Y)**2)
    if jump < 0.4:
        TARGET_X += 0.2 * (CMD_X - TARGET_X)
        TARGET_Y += 0.2 * (CMD_Y - TARGET_Y)
    else:
        TARGET_X, TARGET_Y = CMD_X, CMD_Y

    x, y = gps.getValues()[0], gps.getValues()[1]
    if math.sqrt((GLOBAL_GOAL_X - x)**2 + (GLOBAL_GOAL_Y - y)**2) < 0.15:
        leftMotor.setVelocity(0.0); rightMotor.setVelocity(0.0)
        if current_log_file: 
            current_log_file.close()
        break

    if emergency_stop:
        leftMotor.setVelocity(0.0); rightMotor.setVelocity(0.0)
        continue

    dx, dy = TARGET_X - x, TARGET_Y - y
    heading = math.atan2(compass.getValues()[0], compass.getValues()[1])
    desired_heading = math.atan2(dy, dx)

    error = desired_heading - heading
    while error > math.pi: error -= 2 * math.pi
    while error < -math.pi: error += 2 * math.pi

    # Anti-windup integral clamp
    integral_error = max(-1.0, min(1.0, integral_error + error * dt))
    omega = (KP * error) + (KI * integral_error) + (KD * (error - last_error) / dt)
    last_error = error

    # Step-by-step PID Telemetry CSV Logging
    if current_log_file is not None:
        ref_deg = math.degrees(desired_heading)
        act_deg = math.degrees(heading)
        err_deg = math.degrees(error)
        current_log_file.write(f"{sim_time:.3f},{ref_deg:.2f},{act_deg:.2f},{err_deg:.2f}\n")
        current_log_file.flush()

    if abs(error) > math.radians(25.0):
        left_speed, right_speed = (-2.0, 2.0) if error > 0 else (2.0, -2.0)
    else:
        left_speed = BASE_SPEED - omega
        right_speed = BASE_SPEED + omega

    MAX_VELOCITY = 6.28
    left_speed = max(-MAX_VELOCITY, min(MAX_VELOCITY, left_speed))
    right_speed = max(-MAX_VELOCITY, min(MAX_VELOCITY, right_speed))

    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)