import math
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("PID-BASED PATH-TRACKING MODULE STARTED")

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

while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        if message == "STOP": emergency_stop = True
        else:
            emergency_stop = False
            try:
                parts = message.split()
                CMD_X, CMD_Y = float(parts[0]), float(parts[1])
            except: pass

    jump = math.sqrt((CMD_X - TARGET_X)**2 + (CMD_Y - TARGET_Y)**2)
    if jump < 0.4:
        TARGET_X += 0.2 * (CMD_X - TARGET_X)
        TARGET_Y += 0.2 * (CMD_Y - TARGET_Y)
    else:
        TARGET_X, TARGET_Y = CMD_X, CMD_Y

    x, y = gps.getValues()[0], gps.getValues()[1]
    if math.sqrt((GLOBAL_GOAL_X - x)**2 + (GLOBAL_GOAL_Y - y)**2) < 0.15:
        leftMotor.setVelocity(0.0); rightMotor.setVelocity(0.0)
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